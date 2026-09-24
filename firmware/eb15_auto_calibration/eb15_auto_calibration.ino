/**
 * EB-15 Robotic Arm - Complete Full Arm Auto-Calibration Firmware
 * ===============================================================
 * Actuator Architecture:
 *  - Planetary Actuators: EBA-17 with 38.4:1 gear ratio on Joints 1, 2, and 3
 *  - Encoders: AS5600 12-bit magnetic contactless encoders attached to the STEPPER REAR SHAFTS
 *    -> Motor shaft rotates 38.4x for 1x joint revolution
 *    -> TICKS_PER_JOINT_RAD = (4096 counts/rev * 38.4 gear ratio) / (2 * PI) = 25032.876 ticks/rad
 *    -> Multi-turn unwrapped tracking calculates true physical joint angles in radians
 * 
 * Full 5-DOF Calibration & Homing Routine:
 *  - Joint 1 (Base Yaw):        AS5600 Encoder on TCA Ch 0 (zeroes to forward pose)
 *  - Joint 2 (Shoulder Pitch):  Dual limit switches on D3 (MIN 0°) & D2 (MAX 180°), AS5600 on Ch 1
 *  - Joint 3 (Elbow Pitch):     Dual limit switches on A1 (MIN -2.700) & A2 (MAX +2.700), AS5600 on Ch 2
 *  - Joint 4 (Wrist Roll):      MG995 Servo on Pin D13 (centers to 90° / 0.0 rad)
 *  - Joint 5 (Gripper):         MG995 Servo on Pin A0 (calibrates open/close stroke)
 * 
 * Safe Collision-Free Sequence:
 *  1. [Base]:     Initialize zero reference.
 *  2. [Shoulder]: Seek D3 MIN, traverse to D2 MAX, calculate direction & steps,
 *                 return to 90° upright midpoint (0.0 rad URDF candle pose).
 *  3. [Elbow]:    Seek A1 MIN, traverse to A2 MAX, calculate direction & steps,
 *                 return to 0.0 rad straight midpoint (facing upwards in real world).
 *  4. [Servos]:   Center wrist to 90° (0 rad roll), gripper to 90° rest.
 *  5. Arm is now in perfect upright "candle" zero pose (0, 0, 0, 0, 0) with all encoders calibrated!
 * 
 * Hardware Pinout:
 *  - Limit Switches (Active LOW, internal pull-up):
 *      Shoulder MIN (0°):     Pin D3
 *      Shoulder MAX (180°):   Pin D2
 *      Elbow MIN (-2.700):    Pin A1
 *      Elbow MAX (+2.700):    Pin A2
 *  - Stepper Drivers (TB6600):
 *      Motor 1 (Base):        STEP = D4,  DIR = D5,  ENA = D6
 *      Motor 2 (Shoulder):    STEP = D7,  DIR = D8,  ENA = D9
 *      Motor 3 (Elbow):       STEP = D10, DIR = D11, ENA = D12
 *  - Servos:
 *      Wrist Roll:            Pin D13
 *      Gripper:               Pin A0
 *  - I2C:                     SDA = A4, SCL = A5 -> TCA9548A (0x70) -> AS5600 (0x36)
 * 
 * Serial Commands (115200 baud):
 *  - ALL / CAL : Run complete full-arm auto-calibration sequence
 *  - J2 / CAL2 : Calibrate Shoulder only
 *  - J3 / CAL3 : Calibrate Elbow only
 *  - Z / ZERO  : Zero all joints to current upright candle pose (0, 0, 0 rad)
 *  - Z1 / Z2/Z3: Zero individual joints
 *  - STAT      : Print detailed calibration diagnostics for all joints
 *  - ENA / DIS : Enable / Disable stepper drivers
 *  - 1+ / 1-   : Jog Joint 1 (Base) + / - 200 steps
 *  - 2+ / 2-   : Jog Joint 2 (Shoulder) + / - 200 steps
 *  - 3+ / 3-   : Jog Joint 3 (Elbow) + / - 200 steps
 *  - W+ / W-   : Jog Wrist servo + / - 10 degrees
 *  - G_OPEN    : Open gripper jaws
 *  - G_CLOSE   : Close gripper jaws
 *  - REV2/REV3 : Invert motor homing direction
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
#define STEP_PIN_1               4
#define DIR_PIN_1                5
#define ENA_PIN_1                6

#define STEP_PIN_2               7
#define DIR_PIN_2                8
#define ENA_PIN_2                9

#define STEP_PIN_3               10
#define DIR_PIN_3                11
#define ENA_PIN_3                12

#define TB6600_ENABLE_LEVEL      LOW
#define TB6600_DISABLE_LEVEL     HIGH

// Homing Direction Mapping (can be toggled live via REV2 / REV3)
uint8_t sh_dir_toward_min = LOW;
uint8_t sh_dir_toward_max = HIGH;

uint8_t el_dir_toward_min = LOW;
uint8_t el_dir_toward_max = HIGH;

// Motion Parameters
#define HOMING_SPEED_US          800    // Delay between pulses during homing
#define JOG_SPEED_US             700    // Delay between pulses during jog
#define BACKOFF_STEPS            200    // Steps backed off from switch
#define BACKOFF_SPEED_US         1000   // Slower speed for backoff
#define HOMING_TIMEOUT_MS        30000  // 30 sec max timeout per search
#define TRAVERSE_TIMEOUT_MS      60000  // 60 sec max timeout for full 309 deg arc traverse

// Kinematic Physical Arc Constants (Radians)
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
//  GLOBAL STATE
// ═══════════════════════════════════════════════════════════════
long     cumulative_ticks[NUM_JOINTS]    = {0, 0, 0};
long     zero_ticks[NUM_JOINTS]          = {0, 0, 0};
uint16_t last_raw_ticks[NUM_JOINTS]      = {0, 0, 0};
bool     encoder_initialized[NUM_JOINTS] = {false, false, false};

int8_t   encoder_directions[NUM_JOINTS]  = {1, 1, 1};
float    joint_positions[NUM_JOINTS]     = {0.0f, 0.0f, 0.0f};
float    target_positions[NUM_JOINTS]    = {0.0f, 0.0f, 0.0f};

// Shoulder (Joint 2) Calibration
bool          sh_cal_ok         = false;
unsigned long total_sh_steps    = 0;

// Elbow (Joint 3) Calibration
bool          el_cal_ok         = false;
unsigned long total_el_steps    = 0;

// Base (Joint 1) Calibration
bool          j1_zero_set       = false;

// Servos
Servo wristServo;
Servo gripperServo;
int wrist_servo_angle = 90;   // 90 deg = 0.0 rad center
int gripper_servo_angle = 90; // 90 deg = nominal rest

// Telemetry Timer (20 Hz = 50ms)
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
  uint8_t lvl = enable ? TB6600_ENABLE_LEVEL : TB6600_DISABLE_LEVEL;
  digitalWrite(ENA_PIN_1, lvl);
  digitalWrite(ENA_PIN_2, lvl);
  digitalWrite(ENA_PIN_3, lvl);
}

void stepMotor(uint8_t step_pin, uint16_t delay_us = HOMING_SPEED_US) {
  digitalWrite(step_pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(step_pin, LOW);
  delayMicroseconds(delay_us);
}

// ═══════════════════════════════════════════════════════════════
//  CALIBRATION: JOINT 2 (SHOULDER)
// ═══════════════════════════════════════════════════════════════
bool calibrateShoulder() {
  Serial.println(F("\n--------------------------------------------------------"));
  Serial.println(F(">>> CALIBRATING JOINT 2 (SHOULDER PITCH)..."));
  Serial.println(F("--------------------------------------------------------"));

  sh_cal_ok = false;
  enableAllMotors(true);

  // 1. Move toward MIN switch (D3)
  Serial.println(F("  [J2] Step 1: Seeking MIN limit switch (Pin D3)..."));
  digitalWrite(DIR_PIN_2, sh_dir_toward_min);
  unsigned long timeout = millis() + HOMING_TIMEOUT_MS;
  while (!isShMinHit() && millis() < timeout) {
    stepMotor(STEP_PIN_2, HOMING_SPEED_US);
  }
  if (!isShMinHit()) {
    Serial.println(F("  [J2] ERROR: MIN limit switch (D3) not reached! Check wiring or REV2."));
    return false;
  }

  // Back off
  Serial.println(F("  [J2]   MIN switch hit! Backing off 200 steps..."));
  digitalWrite(DIR_PIN_2, sh_dir_toward_max);
  for (int i = 0; i < BACKOFF_STEPS; i++) stepMotor(STEP_PIN_2, BACKOFF_SPEED_US);
  delay(300);
  updateEncoders();
  long ticks_at_min = cumulative_ticks[1];
  Serial.print(F("  [J2]   Motor ticks at MIN: "));
  Serial.println(ticks_at_min);

  // 2. Traverse toward MAX switch (D2)
  Serial.println(F("  [J2] Step 2: Traversing to MAX limit switch (Pin D2)..."));
  digitalWrite(DIR_PIN_2, sh_dir_toward_max);
  unsigned long steps = 0;
  timeout = millis() + TRAVERSE_TIMEOUT_MS;
  while (!isShMaxHit() && millis() < timeout) {
    stepMotor(STEP_PIN_2, HOMING_SPEED_US);
    steps++;
  }
  if (!isShMaxHit()) {
    Serial.println(F("  [J2] ERROR: MAX limit switch (D2) not reached! Check wiring."));
    return false;
  }

  // Back off
  Serial.print(F("  [J2]   MAX switch hit after "));
  Serial.print(steps);
  Serial.println(F(" steps! Backing off 200 steps..."));
  digitalWrite(DIR_PIN_2, sh_dir_toward_min);
  for (int i = 0; i < BACKOFF_STEPS; i++) stepMotor(STEP_PIN_2, BACKOFF_SPEED_US);
  delay(300);
  updateEncoders();
  long ticks_at_max = cumulative_ticks[1];
  total_sh_steps = steps;
  Serial.print(F("  [J2]   Motor ticks at MAX: "));
  Serial.println(ticks_at_max);

  // 3. Direction from multi-turn cumulative ticks
  encoder_directions[1] = (ticks_at_max > ticks_at_min) ? 1 : -1;
  Serial.print(F("  [J2]   Direction: "));
  Serial.println(encoder_directions[1] > 0 ? F("+1") : F("-1"));

  // 4. Return to 90° Midpoint (0.0 rad in URDF / Upright Candle Pose)
  unsigned long usable = total_sh_steps - BACKOFF_STEPS;
  unsigned long return_steps = usable / 2;
  Serial.print(F("  [J2] Step 3: Returning "));
  Serial.print(return_steps);
  Serial.println(F(" steps to 90 deg upright midpoint (0.0 rad URDF)..."));
  digitalWrite(DIR_PIN_2, sh_dir_toward_min);
  for (unsigned long i = 0; i < return_steps; i++) {
    if (isShMinHit()) break;
    stepMotor(STEP_PIN_2, HOMING_SPEED_US);
  }
  delay(300);

  // 5. Zero encoder at upright midpoint
  updateEncoders();
  zero_ticks[1] = cumulative_ticks[1];
  joint_positions[1] = 0.0f;
  sh_cal_ok = true;

  Serial.println(F("  [J2] >>> SHOULDER CALIBRATION COMPLETE & HOMED AT 0.0 RAD (UPRIGHT)! <<<"));
  return true;
}

// ═══════════════════════════════════════════════════════════════
//  CALIBRATION: JOINT 3 (ELBOW)
// ═══════════════════════════════════════════════════════════════
bool calibrateElbow() {
  Serial.println(F("\n--------------------------------------------------------"));
  Serial.println(F(">>> CALIBRATING JOINT 3 (ELBOW PITCH)..."));
  Serial.println(F("--------------------------------------------------------"));

  el_cal_ok = false;
  enableAllMotors(true);

  // 1. Move toward MIN switch (A1)
  Serial.println(F("  [J3] Step 1: Seeking MIN limit switch (Pin A1 / -2.700 rad)..."));
  digitalWrite(DIR_PIN_3, el_dir_toward_min);
  unsigned long timeout = millis() + HOMING_TIMEOUT_MS;
  while (!isElMinHit() && millis() < timeout) {
    stepMotor(STEP_PIN_3, HOMING_SPEED_US);
  }
  if (!isElMinHit()) {
    Serial.println(F("  [J3] ERROR: MIN limit switch (A1) not reached! Check wiring or REV3."));
    return false;
  }

  // Back off
  Serial.println(F("  [J3]   MIN switch hit! Backing off 200 steps..."));
  digitalWrite(DIR_PIN_3, el_dir_toward_max);
  for (int i = 0; i < BACKOFF_STEPS; i++) stepMotor(STEP_PIN_3, BACKOFF_SPEED_US);
  delay(300);
  updateEncoders();
  long ticks_at_min = cumulative_ticks[2];
  Serial.print(F("  [J3]   Motor ticks at MIN: "));
  Serial.println(ticks_at_min);

  // 2. Traverse toward MAX switch (A2)
  Serial.println(F("  [J3] Step 2: Traversing to MAX limit switch (Pin A2 / +2.700 rad)..."));
  digitalWrite(DIR_PIN_3, el_dir_toward_max);
  unsigned long steps = 0;
  timeout = millis() + TRAVERSE_TIMEOUT_MS;
  while (!isElMaxHit() && millis() < timeout) {
    stepMotor(STEP_PIN_3, HOMING_SPEED_US);
    steps++;
    if (steps % 1000 == 0) {
      Serial.print(F("  [J3]   Stepping: "));
      Serial.println(steps);
    }
  }
  if (!isElMaxHit()) {
    Serial.print(F("  [J3] ERROR: MAX limit switch (A2) not reached after "));
    Serial.print(steps);
    Serial.println(F(" steps! Check wiring or physical switch contact."));
    return false;
  }

  // Back off
  Serial.print(F("  [J3]   MAX switch hit after "));
  Serial.print(steps);
  Serial.println(F(" steps! Backing off 200 steps..."));
  digitalWrite(DIR_PIN_3, el_dir_toward_min);
  for (int i = 0; i < BACKOFF_STEPS; i++) stepMotor(STEP_PIN_3, BACKOFF_SPEED_US);
  delay(300);
  updateEncoders();
  long ticks_at_max = cumulative_ticks[2];
  total_el_steps = steps;
  Serial.print(F("  [J3]   Motor ticks at MAX: "));
  Serial.println(ticks_at_max);

  // 3. Direction from multi-turn cumulative ticks
  encoder_directions[2] = (ticks_at_max > ticks_at_min) ? 1 : -1;
  Serial.print(F("  [J3]   Direction: "));
  Serial.println(encoder_directions[2] > 0 ? F("+1") : F("-1"));

  // 4. Return to Midpoint (0.0 rad in URDF / Upright Straight Alignment)
  unsigned long usable = total_el_steps - BACKOFF_STEPS;
  unsigned long return_steps = usable / 2;
  Serial.print(F("  [J3] Step 3: Returning "));
  Serial.print(return_steps);
  Serial.println(F(" steps to 0.0 rad straight midpoint (UPRIGHT)..."));
  digitalWrite(DIR_PIN_3, el_dir_toward_min);
  for (unsigned long i = 0; i < return_steps; i++) {
    if (isElMinHit()) break;
    stepMotor(STEP_PIN_3, HOMING_SPEED_US);
  }
  delay(300);

  // 5. Zero encoder at midpoint
  updateEncoders();
  zero_ticks[2] = cumulative_ticks[2];
  joint_positions[2] = 0.0f;
  el_cal_ok = true;

  Serial.println(F("  [J3] >>> ELBOW CALIBRATION COMPLETE & HOMED AT 0.0 RAD (UPRIGHT)! <<<"));
  return true;
}

// ═══════════════════════════════════════════════════════════════
//  CALIBRATION: SERVOS
// ═══════════════════════════════════════════════════════════════
void calibrateServos() {
  Serial.println(F("\n--------------------------------------------------------"));
  Serial.println(F(">>> CALIBRATING SERVOS (WRIST ON D13 & GRIPPER ON A0)..."));
  Serial.println(F("--------------------------------------------------------"));

  if (!wristServo.attached())   wristServo.attach(WRIST_SERVO_PIN);
  if (!gripperServo.attached()) gripperServo.attach(GRIPPER_SERVO_PIN);

  wrist_servo_angle = 90;
  wristServo.write(wrist_servo_angle);
  Serial.println(F("  [Wrist]   Homed to 90 deg (0.0 rad roll)."));

  gripper_servo_angle = 90;
  gripperServo.write(gripper_servo_angle);
  Serial.println(F("  [Gripper] Homed to 90 deg (nominal rest position)."));
  delay(300);
}

// ═══════════════════════════════════════════════════════════════
//  ZERO JOINT POSITIONS TO CURRENT UPRIGHT POSE
// ═══════════════════════════════════════════════════════════════
void zeroBase() {
  updateEncoders();
  zero_ticks[0] = cumulative_ticks[0];
  joint_positions[0] = 0.0f;
  j1_zero_set = true;
  Serial.println(F("  [J1 Base] Zeroed to current pose (0.0000 rad)."));
}

void zeroShoulder() {
  updateEncoders();
  zero_ticks[1] = cumulative_ticks[1];
  joint_positions[1] = 0.0f;
  sh_cal_ok = true;
  Serial.println(F("  [J2 Shoulder] Zeroed to current pose (0.0000 rad)."));
}

void zeroElbow() {
  updateEncoders();
  zero_ticks[2] = cumulative_ticks[2];
  joint_positions[2] = 0.0f;
  el_cal_ok = true;
  Serial.println(F("  [J3 Elbow] Zeroed to current pose (0.0000 rad)."));
}

void setZeroAll() {
  updateEncoders();
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    zero_ticks[i] = cumulative_ticks[i];
    joint_positions[i] = 0.0f;
  }
  sh_cal_ok = true;
  el_cal_ok = true;
  j1_zero_set = true;
  Serial.println(F("\n>>> ALL JOINTS ZEROED TO CURRENT POSE (0.000, 0.000, 0.000 rad) <<<\n"));
}

// ═══════════════════════════════════════════════════════════════
//  MASTER CALIBRATION: FULL ARM
// ═══════════════════════════════════════════════════════════════
void runFullArmCalibration() {
  Serial.println(F("\n========================================================"));
  Serial.println(F("        STARTING FULL ARM AUTO-CALIBRATION              "));
  Serial.println(F("========================================================"));

  enableAllMotors(true);

  // Step 1: Base reference
  Serial.println(F("\n>> Step 1/4: Initializing Base (Joint 1)..."));
  zeroBase();

  // Step 2: Shoulder (Joint 2) - Safely moves arm upright
  Serial.println(F("\n>> Step 2/4: Homing Shoulder (Joint 2) to vertical..."));
  if (!calibrateShoulder()) {
    Serial.println(F("\n[HALT] Shoulder calibration failed! Aborting remaining joints."));
    return;
  }

  // Step 3: Elbow (Joint 3) - Safely moves elbow in clear upper airspace
  Serial.println(F("\n>> Step 3/4: Homing Elbow (Joint 3)..."));
  if (!calibrateElbow()) {
    Serial.println(F("\n[HALT] Elbow calibration failed!"));
    return;
  }

  // Step 4: Servos
  Serial.println(F("\n>> Step 4/4: Centering Wrist & Gripper Servos..."));
  calibrateServos();

  Serial.println(F("\n========================================================"));
  Serial.println(F("       FULL ARM AUTO-CALIBRATION SUCCESSFUL!            "));
  Serial.println(F("========================================================"));
  Serial.println(F("Arm is now positioned in the upright 'candle' zero pose:"));
  Serial.println(F("  - Joint 1 (Base):     0.0000 rad"));
  Serial.println(F("  - Joint 2 (Shoulder): 0.0000 rad (facing UPWARDS)"));
  Serial.println(F("  - Joint 3 (Elbow):    0.0000 rad (facing UPWARDS)"));
  Serial.println(F("  - Wrist Servo (D13):  0.0000 rad (90 deg roll)"));
  Serial.println(F("  - Gripper (A0):       Nominal rest"));
  Serial.println(F("========================================================\n"));
}

// ═══════════════════════════════════════════════════════════════
//  DIAGNOSTIC STATUS PRINT
// ═══════════════════════════════════════════════════════════════
void printFullStatus() {
  updateEncoders();
  Serial.println(F("\n========== EB-15 CURRENT ARM STATUS (38.4:1 GEARBOX) =========="));
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    float rad = joint_positions[i];
    float deg = rad * 180.0f / PI;
    long net_ticks = cumulative_ticks[i] - zero_ticks[i];
    float motor_revs = (float)net_ticks / 4096.0f;

    Serial.print(F("  Joint "));
    Serial.print(i + 1);
    Serial.print(F(": Angle="));
    Serial.print(rad, 4);
    Serial.print(F(" rad ("));
    Serial.print(deg, 1);
    Serial.print(F(" deg) | Motor Revs="));
    Serial.print(motor_revs, 2);
    Serial.print(F(" | Net Ticks="));
    Serial.print(net_ticks);
    Serial.print(F(" | Dir="));
    Serial.println(encoder_directions[i]);
  }

  Serial.print(F("  Limit Switches:"));
  Serial.print(F(" Sh_Min(D3)=")); Serial.print(isShMinHit() ? F("HIT ") : F("ok  "));
  Serial.print(F(" Sh_Max(D2)=")); Serial.print(isShMaxHit() ? F("HIT ") : F("ok  "));
  Serial.print(F(" El_Min(A1)=")); Serial.print(isElMinHit() ? F("HIT ") : F("ok  "));
  Serial.print(F(" El_Max(A2)=")); Serial.println(isElMaxHit() ? F("HIT") : F("ok"));

  Serial.print(F("  Wrist Servo (D13): ")); Serial.print(wrist_servo_angle); Serial.println(F(" deg"));
  Serial.print(F("  Gripper Servo(A0): ")); Serial.print(gripper_servo_angle); Serial.println(F(" deg"));
  Serial.print(F("  Calibration State: Shoulder="));
  Serial.print(sh_cal_ok ? F("OK") : F("NO"));
  Serial.print(F(" | Elbow="));
  Serial.println(el_cal_ok ? F("OK") : F("NO"));
  Serial.println(F("=================================================================\n"));
}

// ═══════════════════════════════════════════════════════════════
//  JOG STEPPERS
// ═══════════════════════════════════════════════════════════════
void jogStepper(uint8_t joint, int steps, uint8_t dir) {
  enableAllMotors(true);
  uint8_t step_pin = (joint == 1) ? STEP_PIN_1 : (joint == 2) ? STEP_PIN_2 : STEP_PIN_3;
  uint8_t dir_pin  = (joint == 1) ? DIR_PIN_1  : (joint == 2) ? DIR_PIN_2  : DIR_PIN_3;

  digitalWrite(dir_pin, dir);
  for (int i = 0; i < steps; i++) {
    if (joint == 2 && dir == sh_dir_toward_min && isShMinHit()) { Serial.println(F(">>> [HALT] Shoulder MIN hit!")); break; }
    if (joint == 2 && dir == sh_dir_toward_max && isShMaxHit()) { Serial.println(F(">>> [HALT] Shoulder MAX hit!")); break; }
    if (joint == 3 && dir == el_dir_toward_min && isElMinHit()) { Serial.println(F(">>> [HALT] Elbow MIN hit!")); break; }
    if (joint == 3 && dir == el_dir_toward_max && isElMaxHit()) { Serial.println(F(">>> [HALT] Elbow MAX hit!")); break; }
    stepMotor(step_pin, JOG_SPEED_US);
  }
}

// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  // Setup Limit Switches with internal pull-ups
  pinMode(SHOULDER_LIMIT_MIN_PIN, INPUT_PULLUP);
  pinMode(SHOULDER_LIMIT_MAX_PIN, INPUT_PULLUP);
  pinMode(ELBOW_LIMIT_MIN_PIN,    INPUT_PULLUP);
  pinMode(ELBOW_LIMIT_MAX_PIN,    INPUT_PULLUP);

  // Setup Stepper Pins
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

  // Setup Servos
  wristServo.attach(WRIST_SERVO_PIN);
  gripperServo.attach(GRIPPER_SERVO_PIN);
  wristServo.write(90);
  gripperServo.write(90);

  Serial.println(F("\n========================================================"));
  Serial.println(F("   EB-15 FULL AUTO-CALIBRATION (EBA-17 38.4:1 RATIO)    "));
  Serial.println(F("========================================================"));
  Serial.println(F("Commands:"));
  Serial.println(F("  ALL / CAL       - Run complete full arm auto-calibration"));
  Serial.println(F("  J2 / CAL2       - Calibrate Shoulder (Joint 2) only"));
  Serial.println(F("  J3 / CAL3       - Calibrate Elbow (Joint 3) only"));
  Serial.println(F("  Z / ZERO        - Zero ALL joints to current upright pose"));
  Serial.println(F("  Z1 / Z2 / Z3    - Zero individual joints"));
  Serial.println(F("  STAT            - Print live diagnostics & angle table"));
  Serial.println(F("  ENA / DIS       - Enable / Disable all stepper motors"));
  Serial.println(F("  1+ / 1-         - Jog Base +/- 200 steps"));
  Serial.println(F("  2+ / 2-         - Jog Shoulder +/- 200 steps"));
  Serial.println(F("  3+ / 3-         - Jog Elbow +/- 200 steps"));
  Serial.println(F("  W+ / W-         - Jog Wrist servo +/- 10 deg"));
  Serial.println(F("  G_OPEN / G_CLOSE- Open / Close Gripper"));
  Serial.println(F("  REV2 / REV3     - Reverse motor direction for homing"));
  Serial.println(F("========================================================\n"));

  // Initialize all joints to zero reference at upright candle pose
  setZeroAll();
  printFullStatus();
}

// ═══════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════════
void loop() {
  // 1. Update Multi-Turn Joint Readings (EBA-17 38.4:1 gear ratio)
  updateEncoders();

  // 2. Process Serial Commands
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "ALL" || cmd == "CAL") {
      runFullArmCalibration();
    } else if (cmd == "J2" || cmd == "CAL2") {
      calibrateShoulder();
    } else if (cmd == "J3" || cmd == "CAL3") {
      calibrateElbow();
    } else if (cmd == "Z1" || cmd == "ZERO1") {
      zeroBase();
    } else if (cmd == "Z2" || cmd == "ZERO2") {
      zeroShoulder();
    } else if (cmd == "Z3" || cmd == "ZERO3") {
      zeroElbow();
    } else if (cmd == "Z" || cmd == "ZERO" || cmd == "SET_ZERO" || cmd == "$SET_ZERO" || cmd == "Z_ALL") {
      setZeroAll();
    } else if (cmd == "STAT") {
      printFullStatus();
    } else if (cmd == "ENA") {
      enableAllMotors(true);
      Serial.println(F("[MOTORS] Drivers ENABLED."));
    } else if (cmd == "DIS") {
      enableAllMotors(false);
      Serial.println(F("[MOTORS] Drivers DISABLED (Freewheel)."));
    } else if (cmd == "1+") {
      jogStepper(1, 200, HIGH);
    } else if (cmd == "1-") {
      jogStepper(1, 200, LOW);
    } else if (cmd == "2+") {
      jogStepper(2, 200, sh_dir_toward_max);
    } else if (cmd == "2-") {
      jogStepper(2, 200, sh_dir_toward_min);
    } else if (cmd == "3+") {
      jogStepper(3, 200, el_dir_toward_max);
    } else if (cmd == "3-") {
      jogStepper(3, 200, el_dir_toward_min);
    } else if (cmd == "W+") {
      wrist_servo_angle = constrain(wrist_servo_angle + 10, 0, 180);
      wristServo.write(wrist_servo_angle);
      Serial.print(F("[Wrist] Angle: ")); Serial.println(wrist_servo_angle);
    } else if (cmd == "W-") {
      wrist_servo_angle = constrain(wrist_servo_angle - 10, 0, 180);
      wristServo.write(wrist_servo_angle);
      Serial.print(F("[Wrist] Angle: ")); Serial.println(wrist_servo_angle);
    } else if (cmd == "G_OPEN") {
      gripper_servo_angle = 140;
      gripperServo.write(gripper_servo_angle);
      Serial.println(F("[Gripper] Opened."));
    } else if (cmd == "G_CLOSE") {
      gripper_servo_angle = 50;
      gripperServo.write(gripper_servo_angle);
      Serial.println(F("[Gripper] Closed."));
    } else if (cmd == "REV2") {
      uint8_t tmp = sh_dir_toward_min; sh_dir_toward_min = sh_dir_toward_max; sh_dir_toward_max = tmp;
      Serial.println(F("[CONFIG] Shoulder (J2) direction reversed."));
    } else if (cmd == "REV3") {
      uint8_t tmp = el_dir_toward_min; el_dir_toward_min = el_dir_toward_max; el_dir_toward_max = tmp;
      Serial.println(F("[CONFIG] Elbow (J3) direction reversed."));
    } else if (cmd.startsWith("$CMD,")) {
      int c1 = cmd.indexOf(',', 5);
      int c2 = cmd.indexOf(',', c1 + 1);
      if (c1 > 0 && c2 > 0) {
        target_positions[0] = cmd.substring(5, c1).toFloat();
        target_positions[1] = cmd.substring(c1 + 1, c2).toFloat();
        target_positions[2] = cmd.substring(c2 + 1).toFloat();
      }
    } else {
      Serial.println(F("Commands: ALL, CAL2, CAL3, ZERO, Z1, Z2, Z3, STAT, ENA, DIS, 1+, 1-, 2+, 2-, 3+, 3-, W+, W-, G_OPEN, G_CLOSE"));
    }
  }

  // 3. Stream Telemetry at 20 Hz (SI Joint Radians for RViz2)
  unsigned long now = millis();
  if (now - last_telemetry_time >= TELEMETRY_INTERVAL_MS) {
    last_telemetry_time = now;

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
