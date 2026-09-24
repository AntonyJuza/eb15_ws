/**
 * EB-15 Robotic Arm - Production Firmware (Arduino Nano)
 * =======================================================
 * Actuator Architecture:
 *  - Planetary Actuators: EBA-17 with 38.4:1 gear ratio on Joints 1, 2, and 3
 *  - Encoders: AS5600 12-bit magnetic contactless encoders attached to the STEPPER REAR SHAFTS
 *    -> Motor shaft rotates 38.4x for 1x joint revolution
 *    -> TICKS_PER_JOINT_RAD = (4096 counts/rev * 38.4 gear ratio) / (2 * PI) = 25032.876 ticks/rad
 *    -> Multi-turn unwrapped tracking calculates true physical joint angles in radians
 * 
 * Features:
 *  - 5-DOF Control: 3x Steppers (Base, Shoulder, Elbow) + 2x Servos (Wrist, Gripper)
 *  - Closed-loop telemetry via AS5600 I2C magnetic encoders (TCA9548A mux)
 *  - Hardware limit switch protection on Shoulder (D3, D2) and Elbow (A1, A2)
 *  - Full collision-safe auto-calibration routine ($HOME / ALL)
 *  - Real-time non-blocking multi-axis position stepping for ROS 2 commands ($CMD)
 *  - Live 20 Hz telemetry streaming for RViz2 visualization ($STATE)
 * 
 * Communication Protocol (115200 baud):
 *  - Telemetry:  $STATE,<j1>,<j2>,<j3>,<sh_min>,<sh_max>,<el_min>,<el_max>,<cal_flag>\n
 *  - Motion:     $CMD,<q1>,<q2>,<q3>,<q_wrist>,<q_grip>\n
 *  - Commands:   $HOME, $HOME_SHOULDER, $HOME_ELBOW, $SET_ZERO, ENA, DIS, STAT, ALL
 */

#include <Wire.h>
#include <Servo.h>

// ═══════════════════════════════════════════════════════════════
//  PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════

// Limit Switches (Active LOW with internal pull-up)
#define SHOULDER_LIMIT_MIN_PIN   3   // Pin D3 — Shoulder 0° / Min
#define SHOULDER_LIMIT_MAX_PIN   2   // Pin D2 — Shoulder 180° / Max
#define ELBOW_LIMIT_MIN_PIN      A1  // Pin A1 — Elbow -2.700 rad / Min
#define ELBOW_LIMIT_MAX_PIN      A2  // Pin A2 — Elbow +2.700 rad / Max
#define LIMIT_SWITCH_ACTIVE_LOW  true

// Servos
#define WRIST_SERVO_PIN          13  // Pin D13 (Wrist Roll Servo)
#define GRIPPER_SERVO_PIN        A0  // Pin A0 / D14 (EBG-20 Gripper Servo)

// Stepper Drivers (TB6600)
#define STEP_PIN_1               4   // Base (Joint 1)
#define DIR_PIN_1                5
#define ENA_PIN_1                6

#define STEP_PIN_2               7   // Shoulder (Joint 2)
#define DIR_PIN_2                8
#define ENA_PIN_2                9

#define STEP_PIN_3               10  // Elbow (Joint 3)
#define DIR_PIN_3                11
#define ENA_PIN_3                12

#define TB6600_ENABLE_LEVEL      LOW
#define TB6600_DISABLE_LEVEL     HIGH

// Direction Mappings (configurable / reversible)
uint8_t base_dir_toward_pos      = HIGH;
uint8_t base_dir_toward_neg      = LOW;
uint8_t sh_dir_toward_min        = LOW;
uint8_t sh_dir_toward_max        = HIGH;
uint8_t el_dir_toward_min        = LOW;
uint8_t el_dir_toward_max        = HIGH;

// Motion Parameters
#define HOMING_SPEED_US          800    // Delay between pulses during homing
#define BACKOFF_STEPS            200    // Steps backed off from switch
#define BACKOFF_SPEED_US         1000   // Slower speed for backoff
#define HOMING_TIMEOUT_MS        30000  // 30 sec max timeout per search
#define TRAVERSE_TIMEOUT_MS      60000  // 60 sec max timeout for full 309 deg arc traverse

// Kinematic Physical Arc Constants (URDF Radians)
const float J2_MIN_RAD = -1.570796f; // -90 deg in URDF (physical 0 deg)
const float J2_MAX_RAD =  1.570796f; // +90 deg in URDF (physical 180 deg)
const float J3_MIN_RAD = -2.700000f; // -2.700 rad
const float J3_MAX_RAD =  2.700000f; // +2.700 rad

// ═══════════════════════════════════════════════════════════════
//  EBA-17 PLANETARY GEAR REDUCTION (38.4:1) & ENCODER (AS5600)
// ═══════════════════════════════════════════════════════════════
#define TCA9548A_ADDR            0x70
#define AS5600_ADDR              0x36
#define AS5600_RAW_ANGLE_REG     0x0C
#define NUM_JOINTS               3

const uint8_t ENCODER_CHANNELS[NUM_JOINTS] = {0, 1, 2};

// EBA-17 Planetary Gearbox: 38.4 motor turns = 1 joint turn
const float GEAR_RATIO          = 38.4f;
// 4096 counts/rev * 38.4 gear ratio / (2 * PI) = 25032.876 ticks per joint radian
const float TICKS_PER_JOINT_RAD = (4096.0f * GEAR_RATIO) / (2.0f * PI);

// ═══════════════════════════════════════════════════════════════
//  GLOBAL STATE & CONTROL
// ═══════════════════════════════════════════════════════════════
long     cumulative_ticks[NUM_JOINTS]    = {0, 0, 0};
long     zero_ticks[NUM_JOINTS]          = {0, 0, 0};
uint16_t last_raw_ticks[NUM_JOINTS]      = {0, 0, 0};
bool     encoder_initialized[NUM_JOINTS] = {false, false, false};

int8_t   encoder_directions[NUM_JOINTS]  = {1, 1, 1};
float    joint_positions[NUM_JOINTS]     = {0.0f, 0.0f, 0.0f};
float    target_positions[NUM_JOINTS]    = {0.0f, 0.0f, 0.0f};

// Stepper Step Tracking
long     target_steps[NUM_JOINTS]        = {0, 0, 0};
long     current_steps[NUM_JOINTS]       = {0, 0, 0};
// 38.4:1 ratio with 1/2 microstep (400 stp/rev) ~ 2444.6 steps/rad
float    steps_per_rad[NUM_JOINTS]       = {2444.6f, 2173.7f, 2173.7f};

// Calibration Flags
bool          sh_cal_ok                  = false;
bool          el_cal_ok                  = false;
bool          j1_zero_set                = false;
unsigned long total_sh_steps             = 0;
unsigned long total_el_steps             = 0;

// Servos
Servo wristServo;
Servo gripperServo;
int wrist_servo_angle = 90;   // 90 deg = 0.0 rad roll center
int gripper_servo_angle = 90; // 90 deg = nominal rest

// Stepper Timing
unsigned long last_step_time = 0;
unsigned int  step_interval_us = 800; // ~1250 steps/sec per active joint

// Telemetry Timing (20 Hz = 50ms)
unsigned long last_telemetry_time = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 50;

// ═══════════════════════════════════════════════════════════════
//  I2C & ENCODER HELPERS
// ═══════════════════════════════════════════════════════════════
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

uint16_t readAS5600(uint8_t channel) {
  tcaSelect(channel);
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_RAW_ANGLE_REG);
  if (Wire.endTransmission(false) != 0) return 9999;

  Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2);
  if (Wire.available() >= 2) {
    uint16_t high = Wire.read();
    uint16_t low  = Wire.read();
    return ((high & 0x0F) << 8) | low;
  }
  return 9999;
}

// Multi-turn encoder unwrap with 38.4:1 planetary gear ratio
void updateEncoders() {
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    uint16_t raw = readAS5600(ENCODER_CHANNELS[i]);
    if (raw <= 4095) {
      if (!encoder_initialized[i]) {
        last_raw_ticks[i] = raw;
        encoder_initialized[i] = true;
      } else {
        int16_t delta = (int16_t)raw - (int16_t)last_raw_ticks[i];
        if (delta > 2048)  delta -= 4096;
        if (delta < -2048) delta += 4096;
        cumulative_ticks[i] += delta;
        last_raw_ticks[i] = raw;
      }
      joint_positions[i] = ((float)(cumulative_ticks[i] - zero_ticks[i]) / TICKS_PER_JOINT_RAD) * encoder_directions[i];
    }
  }
}

// ═══════════════════════════════════════════════════════════════
//  LIMIT SWITCH HELPERS
// ═══════════════════════════════════════════════════════════════
bool isShMinHit() { return digitalRead(SHOULDER_LIMIT_MIN_PIN) == (LIMIT_SWITCH_ACTIVE_LOW ? LOW : HIGH); }
bool isShMaxHit() { return digitalRead(SHOULDER_LIMIT_MAX_PIN) == (LIMIT_SWITCH_ACTIVE_LOW ? LOW : HIGH); }
bool isElMinHit() { return digitalRead(ELBOW_LIMIT_MIN_PIN)    == (LIMIT_SWITCH_ACTIVE_LOW ? LOW : HIGH); }
bool isElMaxHit() { return digitalRead(ELBOW_LIMIT_MAX_PIN)    == (LIMIT_SWITCH_ACTIVE_LOW ? LOW : HIGH); }

// ═══════════════════════════════════════════════════════════════
//  MOTOR CONTROL HELPERS
// ═══════════════════════════════════════════════════════════════
void enableAllMotors(bool enable) {
  uint8_lvl = enable ? TB6600_ENABLE_LEVEL : TB6600_DISABLE_LEVEL;
  digitalWrite(ENA_PIN_1, lvl);
  digitalWrite(ENA_PIN_2, lvl);
  digitalWrite(ENA_PIN_3, lvl);
}

void stepSingleMotor(uint8_t step_pin, uint16_t delay_us = HOMING_SPEED_US) {
  digitalWrite(step_pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(step_pin, LOW);
  delayMicroseconds(delay_us);
}

// Non-blocking synchronized multi-axis stepper step generator
void stepJointsTowardTarget() {
  bool step1 = false;
  bool step2 = false;
  bool step3 = false;

  // Joint 1 (Base)
  if (target_steps[0] > current_steps[0]) {
    digitalWrite(DIR_PIN_1, base_dir_toward_pos);
    step1 = true;
    current_steps[0]++;
  } else if (target_steps[0] < current_steps[0]) {
    digitalWrite(DIR_PIN_1, base_dir_toward_neg);
    step1 = true;
    current_steps[0]--;
  }

  // Joint 2 (Shoulder) with Limit Switch Safety
  if (target_steps[1] > current_steps[1]) {
    if (!isShMaxHit()) {
      digitalWrite(DIR_PIN_2, sh_dir_toward_max);
      step2 = true;
      current_steps[1]++;
    } else {
      target_steps[1] = current_steps[1];
    }
  } else if (target_steps[1] < current_steps[1]) {
    if (!isShMinHit()) {
      digitalWrite(DIR_PIN_2, sh_dir_toward_min);
      step2 = true;
      current_steps[1]--;
    } else {
      target_steps[1] = current_steps[1];
    }
  }

  // Joint 3 (Elbow) with Limit Switch Safety
  if (target_steps[2] > current_steps[2]) {
    if (!isElMaxHit()) {
      digitalWrite(DIR_PIN_3, el_dir_toward_max);
      step3 = true;
      current_steps[2]++;
    } else {
      target_steps[2] = current_steps[2];
    }
  } else if (target_steps[2] < current_steps[2]) {
    if (!isElMinHit()) {
      digitalWrite(DIR_PIN_3, el_dir_toward_min);
      step3 = true;
      current_steps[2]--;
    } else {
      target_steps[2] = current_steps[2];
    }
  }

  // Trigger step pulses concurrently
  if (step1) digitalWrite(STEP_PIN_1, HIGH);
  if (step2) digitalWrite(STEP_PIN_2, HIGH);
  if (step3) digitalWrite(STEP_PIN_3, HIGH);

  if (step1 || step2 || step3) {
    delayMicroseconds(5);
    digitalWrite(STEP_PIN_1, LOW);
    digitalWrite(STEP_PIN_2, LOW);
    digitalWrite(STEP_PIN_3, LOW);
  }
}

// ═══════════════════════════════════════════════════════════════
//  CALIBRATION: JOINT 2 (SHOULDER)
// ═══════════════════════════════════════════════════════════════
bool calibrateShoulder() {
  Serial.println(F("# [CAL] Calibrating Joint 2 (Shoulder)..."));
  sh_cal_ok = false;
  enableAllMotors(true);

  // 1. Move toward MIN switch (D3)
  digitalWrite(DIR_PIN_2, sh_dir_toward_min);
  unsigned long timeout = millis() + HOMING_TIMEOUT_MS;
  while (!isShMinHit() && millis() < timeout) {
    stepSingleMotor(STEP_PIN_2, HOMING_SPEED_US);
  }
  if (!isShMinHit()) {
    Serial.println(F("# [ERROR] Shoulder MIN limit (D3) not reached!"));
    return false;
  }

  // Back off
  digitalWrite(DIR_PIN_2, sh_dir_toward_max);
  for (int i = 0; i < BACKOFF_STEPS; i++) stepSingleMotor(STEP_PIN_2, BACKOFF_SPEED_US);
  delay(300);
  updateEncoders();
  long ticks_at_min = cumulative_ticks[1];

  // 2. Traverse toward MAX switch (D2)
  digitalWrite(DIR_PIN_2, sh_dir_toward_max);
  unsigned long steps = 0;
  timeout = millis() + TRAVERSE_TIMEOUT_MS;
  while (!isShMaxHit() && millis() < timeout) {
    stepSingleMotor(STEP_PIN_2, HOMING_SPEED_US);
    steps++;
  }
  if (!isShMaxHit()) {
    Serial.println(F("# [ERROR] Shoulder MAX limit (D2) not reached!"));
    return false;
  }

  // Back off
  digitalWrite(DIR_PIN_2, sh_dir_toward_min);
  for (int i = 0; i < BACKOFF_STEPS; i++) stepSingleMotor(STEP_PIN_2, BACKOFF_SPEED_US);
  delay(300);
  updateEncoders();
  long ticks_at_max = cumulative_ticks[1];
  total_sh_steps = steps;

  encoder_directions[1] = (ticks_at_max > ticks_at_min) ? 1 : -1;
  steps_per_rad[1] = (float)total_sh_steps / PI; // 180 deg = PI rad

  // Return to 90° Midpoint (0.0 rad URDF vertical candle pose)
  unsigned long usable = total_sh_steps - BACKOFF_STEPS;
  unsigned long return_steps = usable / 2;
  digitalWrite(DIR_PIN_2, sh_dir_toward_min);
  for (unsigned long i = 0; i < return_steps; i++) {
    if (isShMinHit()) break;
    stepSingleMotor(STEP_PIN_2, HOMING_SPEED_US);
  }
  delay(300);

  updateEncoders();
  zero_ticks[1] = cumulative_ticks[1];
  current_steps[1] = 0;
  target_steps[1] = 0;
  target_positions[1] = 0.0f;
  sh_cal_ok = true;

  Serial.print(F("# Shoulder Calibrated. Steps: "));
  Serial.print(total_sh_steps);
  Serial.print(F(" | Steps/rad: "));
  Serial.println(steps_per_rad[1], 1);
  return true;
}

// ═══════════════════════════════════════════════════════════════
//  CALIBRATION: JOINT 3 (ELBOW)
// ═══════════════════════════════════════════════════════════════
bool calibrateElbow() {
  Serial.println(F("# [CAL] Calibrating Joint 3 (Elbow)..."));
  el_cal_ok = false;
  enableAllMotors(true);

  // 1. Move toward MIN switch (A1)
  digitalWrite(DIR_PIN_3, el_dir_toward_min);
  unsigned long timeout = millis() + HOMING_TIMEOUT_MS;
  while (!isElMinHit() && millis() < timeout) {
    stepSingleMotor(STEP_PIN_3, HOMING_SPEED_US);
  }
  if (!isElMinHit()) {
    Serial.println(F("# [ERROR] Elbow MIN limit (A1) not reached!"));
    return false;
  }

  // Back off
  digitalWrite(DIR_PIN_3, el_dir_toward_max);
  for (int i = 0; i < BACKOFF_STEPS; i++) stepSingleMotor(STEP_PIN_3, BACKOFF_SPEED_US);
  delay(300);
  updateEncoders();
  long ticks_at_min = cumulative_ticks[2];

  // 2. Traverse toward MAX switch (A2)
  digitalWrite(DIR_PIN_3, el_dir_toward_max);
  unsigned long steps = 0;
  timeout = millis() + TRAVERSE_TIMEOUT_MS;
  while (!isElMaxHit() && millis() < timeout) {
    stepSingleMotor(STEP_PIN_3, HOMING_SPEED_US);
    steps++;
    if (steps % 1000 == 0) {
      Serial.print(F("# Elbow step count: "));
      Serial.println(steps);
    }
  }
  if (!isElMaxHit()) {
    Serial.print(F("# [ERROR] Elbow MAX limit (A2) not reached after "));
    Serial.print(steps);
    Serial.println(F(" steps! Check wiring on A2 or switch alignment."));
    return false;
  }

  // Back off
  digitalWrite(DIR_PIN_3, el_dir_toward_min);
  for (int i = 0; i < BACKOFF_STEPS; i++) stepSingleMotor(STEP_PIN_3, BACKOFF_SPEED_US);
  delay(300);
  updateEncoders();
  long ticks_at_max = cumulative_ticks[2];
  total_el_steps = steps;

  encoder_directions[2] = (ticks_at_max > ticks_at_min) ? 1 : -1;
  float total_rad = J3_MAX_RAD - J3_MIN_RAD; // 5.400 rad
  steps_per_rad[2] = (float)total_el_steps / total_rad;

  // Return to Midpoint (0.0 rad URDF straight / facing UPWARDS)
  unsigned long usable = total_el_steps - BACKOFF_STEPS;
  unsigned long return_steps = usable / 2;
  digitalWrite(DIR_PIN_3, el_dir_toward_min);
  for (unsigned long i = 0; i < return_steps; i++) {
    if (isElMinHit()) break;
    stepSingleMotor(STEP_PIN_3, HOMING_SPEED_US);
  }
  delay(300);

  updateEncoders();
  zero_ticks[2] = cumulative_ticks[2];
  current_steps[2] = 0;
  target_steps[2] = 0;
  target_positions[2] = 0.0f;
  el_cal_ok = true;

  Serial.print(F("# Elbow Calibrated. Steps: "));
  Serial.print(total_el_steps);
  Serial.print(F(" | Steps/rad: "));
  Serial.println(steps_per_rad[2], 1);
  return true;
}

// ═══════════════════════════════════════════════════════════════
//  CALIBRATION: SERVOS & BASE
// ═══════════════════════════════════════════════════════════════
void calibrateServos() {
  if (!wristServo.attached())   wristServo.attach(WRIST_SERVO_PIN);
  if (!gripperServo.attached()) gripperServo.attach(GRIPPER_SERVO_PIN);

  wrist_servo_angle = 90;
  gripper_servo_angle = 90;
  wristServo.write(wrist_servo_angle);
  gripperServo.write(gripper_servo_angle);
  delay(300);
  Serial.println(F("# Wrist centered (90 deg), Gripper rest (90 deg)."));
}

void zeroBase() {
  updateEncoders();
  zero_ticks[0] = cumulative_ticks[0];
  current_steps[0] = 0;
  target_steps[0] = 0;
  target_positions[0] = 0.0f;
  j1_zero_set = true;
  Serial.println(F("# Base (Joint 1) zeroed to current pose (0.0000 rad)."));
}

void zeroShoulder() {
  updateEncoders();
  zero_ticks[1] = cumulative_ticks[1];
  current_steps[1] = 0;
  target_steps[1] = 0;
  target_positions[1] = 0.0f;
  sh_cal_ok = true;
  Serial.println(F("# Shoulder (Joint 2) zeroed to current pose (0.0000 rad)."));
}

void zeroElbow() {
  updateEncoders();
  zero_ticks[2] = cumulative_ticks[2];
  current_steps[2] = 0;
  target_steps[2] = 0;
  target_positions[2] = 0.0f;
  el_cal_ok = true;
  Serial.println(F("# Elbow (Joint 3) zeroed to current pose (0.0000 rad)."));
}

void setZeroAll() {
  updateEncoders();
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    zero_ticks[i] = cumulative_ticks[i];
    current_steps[i] = 0;
    target_steps[i] = 0;
    target_positions[i] = 0.0f;
  }
  j1_zero_set = true;
  sh_cal_ok = true;
  el_cal_ok = true;
  Serial.println(F("# All joint encoders zeroed to current upright pose (0, 0, 0)."));
}

void runFullArmCalibration() {
  Serial.println(F("# >>> STARTING FULL ARM AUTO-CALIBRATION <<<"));
  enableAllMotors(true);

  zeroBase();
  if (!calibrateShoulder()) {
    Serial.println(F("# [HALT] Shoulder homing failed! Aborting."));
    return;
  }
  if (!calibrateElbow()) {
    Serial.println(F("# [HALT] Elbow homing failed! Aborting."));
    return;
  }
  calibrateServos();
  Serial.println(F("# >>> FULL ARM AUTO-CALIBRATION COMPLETE (Candle Pose 0,0,0,0,0) <<<"));
}

// ═══════════════════════════════════════════════════════════════
//  DIAGNOSTIC STATUS
// ═══════════════════════════════════════════════════════════════
void printDiagnostics() {
  updateEncoders();
  Serial.println(F("\n--- EB-15 STATUS (38.4:1 GEARBOX) ---"));
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    Serial.print(F("J")); Serial.print(i + 1);
    Serial.print(F(": Pos=")); Serial.print(joint_positions[i], 4);
    Serial.print(F(" rad | Tgt=")); Serial.print(target_positions[i], 4);
    Serial.print(F(" | Steps=")); Serial.print(current_steps[i]);
    Serial.print(F("/")); Serial.print(target_steps[i]);
    Serial.print(F(" | Motor Revs="));
    Serial.println((float)(cumulative_ticks[i] - zero_ticks[i]) / 4096.0f, 2);
  }
  Serial.print(F("Switches: Sh_Min=")); Serial.print(isShMinHit() ? 1 : 0);
  Serial.print(F(" Sh_Max=")); Serial.print(isShMaxHit() ? 1 : 0);
  Serial.print(F(" El_Min=")); Serial.print(isElMinHit() ? 1 : 0);
  Serial.print(F(" El_Max=")); Serial.println(isElMaxHit() ? 1 : 0);
  Serial.print(F("Servos: Wrist=")); Serial.print(wrist_servo_angle);
  Serial.print(F(" deg | Grip=")); Serial.print(gripper_servo_angle); Serial.println(F(" deg"));
  Serial.println(F("-------------------------------------\n"));
}

// ═══════════════════════════════════════════════════════════════
//  COMMAND PARSER
// ═══════════════════════════════════════════════════════════════
void handleCommand(String cmd) {
  cmd.trim();

  // ROS 2 Motion Command: $CMD,q1,q2,q3,q_wrist,q_grip
  if (cmd.startsWith(F("$CMD,"))) {
    int c1 = cmd.indexOf(',', 5);
    int c2 = (c1 > 0) ? cmd.indexOf(',', c1 + 1) : -1;
    int c3 = (c2 > 0) ? cmd.indexOf(',', c2 + 1) : -1;
    int c4 = (c3 > 0) ? cmd.indexOf(',', c3 + 1) : -1;

    if (c1 > 0 && c2 > 0) {
      float q1 = cmd.substring(5, c1).toFloat();
      float q2 = cmd.substring(c1 + 1, c2).toFloat();
      float q3 = (c3 > 0) ? cmd.substring(c2 + 1, c3).toFloat() : cmd.substring(c2 + 1).toFloat();

      target_positions[0] = q1;
      target_positions[1] = q2;
      target_positions[2] = q3;

      target_steps[0] = (long)round(q1 * steps_per_rad[0]);
      target_steps[1] = (long)round(q2 * steps_per_rad[1]);
      target_steps[2] = (long)round(q3 * steps_per_rad[2]);

      // Wrist Servo
      if (c3 > 0) {
        float q_wrist = (c4 > 0) ? cmd.substring(c3 + 1, c4).toFloat() : cmd.substring(c3 + 1).toFloat();
        wrist_servo_angle = constrain((int)round(90.0f + q_wrist * (180.0f / PI)), 0, 180);
        wristServo.write(wrist_servo_angle);
      }

      // Gripper Servo: [-0.016 (open) .. +0.012 (close)] -> [140 deg .. 50 deg]
      if (c4 > 0) {
        float q_grip = cmd.substring(c4 + 1).toFloat();
        float t = constrain((q_grip + 0.016f) / 0.028f, 0.0f, 1.0f);
        gripper_servo_angle = constrain((int)round(140.0f - t * 90.0f), 45, 145);
        gripperServo.write(gripper_servo_angle);
      }
    }
    return;
  }

  // System Commands
  String ucmd = cmd;
  ucmd.toUpperCase();

  if (ucmd == "$HOME" || ucmd == "ALL" || ucmd == "CAL") {
    runFullArmCalibration();
  } else if (ucmd == "$HOME_SHOULDER" || ucmd == "J2" || ucmd == "CAL2") {
    calibrateShoulder();
  } else if (ucmd == "$HOME_ELBOW" || ucmd == "J3" || ucmd == "CAL3") {
    calibrateElbow();
  } else if (ucmd == "$SET_ZERO" || ucmd == "ZERO" || ucmd == "SET_ZERO" || ucmd == "Z" || ucmd == "Z_ALL") {
    setZeroAll();
  } else if (ucmd == "Z1") {
    zeroBase();
  } else if (ucmd == "Z2") {
    zeroShoulder();
  } else if (ucmd == "Z3") {
    zeroElbow();
  } else if (ucmd == "STAT") {
    printDiagnostics();
  } else if (ucmd == "ENA") {
    enableAllMotors(true);
    Serial.println(F("# Stepper drivers ENABLED"));
  } else if (ucmd == "DIS") {
    enableAllMotors(false);
    Serial.println(F("# Stepper drivers DISABLED"));
  } else if (ucmd == "REV1") {
    uint8_t t = base_dir_toward_pos; base_dir_toward_pos = base_dir_toward_neg; base_dir_toward_neg = t;
    Serial.println(F("# Base (J1) direction reversed"));
  } else if (ucmd == "REV2") {
    uint8_t t = sh_dir_toward_min; sh_dir_toward_min = sh_dir_toward_max; sh_dir_toward_max = t;
    Serial.println(F("# Shoulder (J2) direction reversed"));
  } else if (ucmd == "REV3") {
    uint8_t t = el_dir_toward_min; el_dir_toward_min = el_dir_toward_max; el_dir_toward_max = t;
    Serial.println(F("# Elbow (J3) direction reversed"));
  } else if (ucmd == "G_OPEN") {
    gripper_servo_angle = 140;
    gripperServo.write(gripper_servo_angle);
    Serial.println(F("# Gripper opened"));
  } else if (ucmd == "G_CLOSE") {
    gripper_servo_angle = 50;
    gripperServo.write(gripper_servo_angle);
    Serial.println(F("# Gripper closed"));
  }
}

// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  pinMode(SHOULDER_LIMIT_MIN_PIN, INPUT_PULLUP);
  pinMode(SHOULDER_LIMIT_MAX_PIN, INPUT_PULLUP);
  pinMode(ELBOW_LIMIT_MIN_PIN,    INPUT_PULLUP);
  pinMode(ELBOW_LIMIT_MAX_PIN,    INPUT_PULLUP);

  const uint8_t pins[] = {
    STEP_PIN_1, DIR_PIN_1, ENA_PIN_1,
    STEP_PIN_2, DIR_PIN_2, ENA_PIN_2,
    STEP_PIN_3, DIR_PIN_3, ENA_PIN_3
  };
  for (uint8_t p : pins) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
  enableAllMotors(true);

  wristServo.attach(WRIST_SERVO_PIN);
  gripperServo.attach(GRIPPER_SERVO_PIN);
  wristServo.write(90);
  gripperServo.write(90);

  // Initialize all joints to 0.0000 rad at upright candle pose
  setZeroAll();

  Serial.println(F("# EB-15 Robotic Arm Firmware Ready (EBA-17 38.4:1 Ratio)."));
  Serial.println(F("# Ready for ROS 2 commands ($CMD) or calibration ($HOME)."));
}

// ═══════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════════
void loop() {
  // 1. Read Multi-Turn Magnetic Encoders via 38.4:1 Gear Ratio
  updateEncoders();

  // 2. Non-blocking Multi-Axis Stepping
  unsigned long now_us = micros();
  if (now_us - last_step_time >= step_interval_us) {
    last_step_time = now_us;
    stepJointsTowardTarget();
  }

  // 3. Process Serial Commands
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }

  // 4. Stream Telemetry at 20 Hz (SI Joint Radians)
  unsigned long now_ms = millis();
  if (now_ms - last_telemetry_time >= TELEMETRY_INTERVAL_MS) {
    last_telemetry_time = now_ms;

    Serial.print(F("$STATE,"));
    Serial.print(joint_positions[0], 4);
    Serial.print(F(","));
    Serial.print(joint_positions[1], 4);
    Serial.print(F(","));
    Serial.print(joint_positions[2], 4);
    Serial.print(F(","));
    Serial.print(isShMinHit() ? 1 : 0);
    Serial.print(F(","));
    Serial.print(isShMaxHit() ? 1 : 0);
    Serial.print(F(","));
    Serial.print(isElMinHit() ? 1 : 0);
    Serial.print(F(","));
    Serial.print(isElMaxHit() ? 1 : 0);
    Serial.print(F(","));
    Serial.println((sh_cal_ok && el_cal_ok) ? 1 : 0);
  }
}
