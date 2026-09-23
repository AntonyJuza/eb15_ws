/**
 * EB-15 Robotic Arm - Dual-Joint Auto-Calibration Firmware
 * =======================================================
 * 
 * Automatic calibration for:
 *  - Shoulder (Joint 2): Dual limit switches on D3 (0°) and D2 (180°)
 *  - Elbow (Joint 3):    Dual limit switches on A1 (-2.700 rad) and A2 (+2.700 rad)
 * 
 * Calibration Sequence for each joint:
 *  1. Move toward MIN limit switch until triggered
 *  2. Back off from MIN switch, record encoder at MIN limit
 *  3. Traverse toward MAX limit switch, counting steps
 *  4. Back off from MAX switch, record encoder at MAX limit
 *  5. Compute encoder offset & direction mapping
 *  6. Return to midpoint position (URDF 0.0 rad home)
 *  7. Re-zero encoder so midpoint reads exactly 0.0 rad
 * 
 * Hardware:
 *  - 3x NEMA 17 Steppers with TB6600 Drivers
 *  - 3x AS5600 Magnetic Encoders via TCA9548A I2C Multiplexer
 *  - 2x Shoulder Limit Switches (MIN at D3, MAX at D2)
 *  - 2x Elbow Limit Switches (MIN at A1, MAX at A2)
 *  - 2x Servos (Wrist Roll at D13, Gripper at A0)
 * 
 * Communication (USB Serial @ 115200 baud):
 *  - Telemetry:  $STATE,<j1_rad>,<j2_rad>,<j3_rad>,<sh_min>,<sh_max>,<el_min>,<el_max>,<cal>
 *  - Commands:   $CMD,<j1_rad>,<j2_rad>,<j3_rad>
 *                $HOME          (re-run full dual auto-calibration)
 *                $HOME_SHOULDER (re-run shoulder calibration only)
 *                $HOME_ELBOW    (re-run elbow calibration only)
 *                $SET_ZERO      (set current encoder readings as zero)
 *                $STATUS        (print calibration diagnostics)
 */

#include <Wire.h>

// ═══════════════════════════════════════════════════════════════
//  PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════

// Shoulder (Joint 2) Limit Switches (Active LOW with internal pull-up)
#define SHOULDER_LIMIT_MIN_PIN   3   // Pin D3 — Home / 0° end (-1.5708 rad URDF)
#define SHOULDER_LIMIT_MAX_PIN   2   // Pin D2 — Far / 180° end (+1.5708 rad URDF)

// Elbow (Joint 3) Limit Switches (Active LOW with internal pull-up)
#define ELBOW_LIMIT_MIN_PIN      A1  // Pin A1 — Folded / -2.700 rad limit
#define ELBOW_LIMIT_MAX_PIN      A2  // Pin A2 — Extended / +2.700 rad limit
#define LIMIT_SWITCH_ACTIVE_LOW  true

// Servos (Wrist Roll & Gripper)
#define WRIST_SERVO_PIN          13  // Pin D13 (Wrist Roll Servo)
#define GRIPPER_SERVO_PIN        A0  // Pin A0 / D14 (EBG-20 Gripper Servo)

// TB6600 Motor 1 (Joint 1 / Base Yaw)
#define STEP_PIN_1               4
#define DIR_PIN_1                5
#define ENA_PIN_1                6

// TB6600 Motor 2 (Joint 2 / Shoulder Pitch)
#define STEP_PIN_2               7
#define DIR_PIN_2                8
#define ENA_PIN_2                9

// TB6600 Motor 3 (Joint 3 / Elbow Pitch)
#define STEP_PIN_3               10
#define DIR_PIN_3                11
#define ENA_PIN_3                12

// TB6600 Enable levels (LOW = enabled on TB6600)
#define TB6600_ENABLE_LEVEL      LOW
#define TB6600_DISABLE_LEVEL     HIGH

// ═══════════════════════════════════════════════════════════════
//  MOTOR DIRECTION MAPPING
//  If a motor moves the WRONG way during calibration,
//  swap these LOW and HIGH values.
// ═══════════════════════════════════════════════════════════════
#define SHOULDER_DIR_TOWARD_MIN  LOW
#define SHOULDER_DIR_TOWARD_MAX  HIGH

#define ELBOW_DIR_TOWARD_MIN     LOW
#define ELBOW_DIR_TOWARD_MAX     HIGH

// Joint 3 Physical Calibration Limits (radians)
const float J3_LIMIT_MIN_RAD = -2.700f;
const float J3_LIMIT_MAX_RAD =  2.700f;

// ═══════════════════════════════════════════════════════════════
//  CALIBRATION PARAMETERS
// ═══════════════════════════════════════════════════════════════
#define HOMING_SPEED_US          800    // Microseconds between step pulses (slow & safe)
#define BACKOFF_STEPS            200    // Steps to back off after hitting a switch
#define BACKOFF_SPEED_US         1000   // Slightly slower for back-off
#define HOMING_TIMEOUT_MS        20000  // 20 sec timeout per phase
#define TRAVERSE_TIMEOUT_MS      35000  // 35 sec timeout for MIN→MAX traverse

// ═══════════════════════════════════════════════════════════════
//  I2C MULTIPLEXER & ENCODER SETTINGS
// ═══════════════════════════════════════════════════════════════
#define TCA9548A_ADDR            0x70
#define AS5600_ADDR              0x36
#define AS5600_RAW_ANGLE_REG     0x0C
#define NUM_JOINTS               3

// TCA9548A channels for Joint 1, Joint 2, Joint 3
const uint8_t ENCODER_CHANNELS[NUM_JOINTS] = {0, 1, 2};

// ═══════════════════════════════════════════════════════════════
//  CALIBRATION STATE
// ═══════════════════════════════════════════════════════════════
float    encoder_offsets[NUM_JOINTS]    = {0.0, 0.0, 0.0};
int8_t   encoder_directions[NUM_JOINTS] = {1, 1, 1};
float    joint_positions[NUM_JOINTS]    = {0.0, 0.0, 0.0};
float    target_positions[NUM_JOINTS]   = {0.0, 0.0, 0.0};

// Shoulder calibration state
bool     shoulder_cal_ok       = false;
uint16_t raw_at_sh_min         = 0;
uint16_t raw_at_sh_max         = 0;
unsigned long total_sh_steps   = 0;

// Elbow calibration state
bool     elbow_cal_ok          = false;
uint16_t raw_at_el_min         = 0;
uint16_t raw_at_el_max         = 0;
unsigned long total_el_steps   = 0;

// Telemetry timer (20 Hz = 50ms)
unsigned long last_telemetry_time = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 50;

// ═══════════════════════════════════════════════════════════════
//  HELPER: Select TCA9548A I2C Channel
// ═══════════════════════════════════════════════════════════════
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// ═══════════════════════════════════════════════════════════════
//  HELPER: Read AS5600 12-bit Raw Angle (0..4095)
// ═══════════════════════════════════════════════════════════════
uint16_t readAS5600RawAngle(uint8_t channel) {
  tcaSelect(channel);
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_RAW_ANGLE_REG);
  if (Wire.endTransmission(false) != 0) {
    return 0;
  }
  Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2);
  if (Wire.available() >= 2) {
    uint16_t high_byte = Wire.read();
    uint16_t low_byte  = Wire.read();
    return ((high_byte & 0x0F) << 8) | low_byte;
  }
  return 0;
}

// Convert raw 12-bit count to radians [-PI, +PI]
float rawAngleToRadians(uint16_t raw) {
  float angle = ((float)raw / 4096.0) * 2.0 * PI;
  if (angle > PI) {
    angle -= 2.0 * PI;
  }
  return angle;
}

// ═══════════════════════════════════════════════════════════════
//  LIMIT SWITCH HELPERS
// ═══════════════════════════════════════════════════════════════
bool isShoulderMinHit() {
  return digitalRead(SHOULDER_LIMIT_MIN_PIN) == (LIMIT_SWITCH_ACTIVE_LOW ? LOW : HIGH);
}

bool isShoulderMaxHit() {
  return digitalRead(SHOULDER_LIMIT_MAX_PIN) == (LIMIT_SWITCH_ACTIVE_LOW ? LOW : HIGH);
}

bool isElbowMinHit() {
  return digitalRead(ELBOW_LIMIT_MIN_PIN) == (LIMIT_SWITCH_ACTIVE_LOW ? LOW : HIGH);
}

bool isElbowMaxHit() {
  return digitalRead(ELBOW_LIMIT_MAX_PIN) == (LIMIT_SWITCH_ACTIVE_LOW ? LOW : HIGH);
}

// ═══════════════════════════════════════════════════════════════
//  MOTOR CONTROL HELPERS
// ═══════════════════════════════════════════════════════════════
void enableMotors(bool enable) {
  uint8_t level = enable ? TB6600_ENABLE_LEVEL : TB6600_DISABLE_LEVEL;
  digitalWrite(ENA_PIN_1, level);
  digitalWrite(ENA_PIN_2, level);
  digitalWrite(ENA_PIN_3, level);
}

void stepMotor(uint8_t step_pin) {
  digitalWrite(step_pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(step_pin, LOW);
}

// ═══════════════════════════════════════════════════════════════
//  SHOULDER (JOINT 2) AUTO-CALIBRATION
// ═══════════════════════════════════════════════════════════════
void autoCalibrateShoulder() {
  Serial.println(F(""));
  Serial.println(F("# =========================================="));
  Serial.println(F("#  SHOULDER (JOINT 2) AUTO-CALIBRATION"));
  Serial.println(F("# =========================================="));

  shoulder_cal_ok = false;
  enableMotors(true);
  delay(100);

  // Phase 1: Seek MIN limit switch
  Serial.println(F("# [J2] Phase 1: Seeking MIN limit switch (D3)..."));
  if (!isShoulderMinHit()) {
    digitalWrite(DIR_PIN_2, SHOULDER_DIR_TOWARD_MIN);
    unsigned long timeout = millis() + HOMING_TIMEOUT_MS;
    while (!isShoulderMinHit() && millis() < timeout) {
      stepMotor(STEP_PIN_2);
      delayMicroseconds(HOMING_SPEED_US);
    }
    if (!isShoulderMinHit()) {
      Serial.println(F("# [J2] ERROR: MIN limit switch not reached! Check wiring on D3."));
      return;
    }
  }

  Serial.println(F("# [J2]   MIN limit reached. Backing off..."));
  digitalWrite(DIR_PIN_2, SHOULDER_DIR_TOWARD_MAX);
  for (int i = 0; i < BACKOFF_STEPS; i++) {
    stepMotor(STEP_PIN_2);
    delayMicroseconds(BACKOFF_SPEED_US);
  }
  delay(300);
  raw_at_sh_min = readAS5600RawAngle(ENCODER_CHANNELS[1]);
  Serial.print(F("# [J2]   Encoder raw at MIN (0 deg): "));
  Serial.println(raw_at_sh_min);

  // Phase 2: Traverse to MAX limit switch
  Serial.println(F("# [J2] Phase 2: Traversing to MAX limit switch (D2)..."));
  digitalWrite(DIR_PIN_2, SHOULDER_DIR_TOWARD_MAX);
  unsigned long step_count = 0;
  unsigned long timeout2 = millis() + TRAVERSE_TIMEOUT_MS;
  while (!isShoulderMaxHit() && millis() < timeout2) {
    stepMotor(STEP_PIN_2);
    delayMicroseconds(HOMING_SPEED_US);
    step_count++;
  }
  if (!isShoulderMaxHit()) {
    Serial.println(F("# [J2] ERROR: MAX limit switch not reached! Check wiring on D2."));
    return;
  }

  Serial.print(F("# [J2]   MAX limit reached after "));
  Serial.print(step_count);
  Serial.println(F(" steps. Backing off..."));
  digitalWrite(DIR_PIN_2, SHOULDER_DIR_TOWARD_MIN);
  for (int i = 0; i < BACKOFF_STEPS; i++) {
    stepMotor(STEP_PIN_2);
    delayMicroseconds(BACKOFF_SPEED_US);
  }
  delay(300);
  raw_at_sh_max = readAS5600RawAngle(ENCODER_CHANNELS[1]);
  total_sh_steps = step_count;
  Serial.print(F("# [J2]   Encoder raw at MAX (180 deg): "));
  Serial.println(raw_at_sh_max);

  // Phase 3: Compute calibration
  int16_t raw_diff = (int16_t)raw_at_sh_max - (int16_t)raw_at_sh_min;
  if (raw_diff > 2048)  raw_diff -= 4096;
  if (raw_diff < -2048) raw_diff += 4096;
  encoder_directions[1] = (raw_diff > 0) ? 1 : -1;

  // Phase 4: Return to 90° (midpoint / URDF 0.0 rad)
  unsigned long usable_range = step_count - BACKOFF_STEPS;
  unsigned long return_steps = usable_range / 2;
  Serial.print(F("# [J2] Phase 4: Returning "));
  Serial.print(return_steps);
  Serial.println(F(" steps to 90 deg midpoint..."));
  digitalWrite(DIR_PIN_2, SHOULDER_DIR_TOWARD_MIN);
  for (unsigned long i = 0; i < return_steps; i++) {
    if (isShoulderMinHit()) break;
    stepMotor(STEP_PIN_2);
    delayMicroseconds(HOMING_SPEED_US);
  }
  delay(300);

  // Phase 5: Re-zero encoder at 90°
  uint16_t raw_mid = readAS5600RawAngle(ENCODER_CHANNELS[1]);
  encoder_offsets[1] = rawAngleToRadians(raw_mid);
  shoulder_cal_ok = true;
  Serial.println(F("# [J2] Shoulder calibration COMPLETE (Homed at 0.0 rad)."));
}

// ═══════════════════════════════════════════════════════════════
//  ELBOW (JOINT 3) AUTO-CALIBRATION
// ═══════════════════════════════════════════════════════════════
void autoCalibrateElbow() {
  Serial.println(F(""));
  Serial.println(F("# =========================================="));
  Serial.println(F("#  ELBOW (JOINT 3) AUTO-CALIBRATION"));
  Serial.println(F("# =========================================="));

  elbow_cal_ok = false;
  enableMotors(true);
  delay(100);

  // Phase 1: Seek MIN limit switch on Pin A1 (-2.700 rad)
  Serial.println(F("# [J3] Phase 1: Seeking MIN limit switch (Pin A1 / -2.700 rad)..."));
  if (!isElbowMinHit()) {
    digitalWrite(DIR_PIN_3, ELBOW_DIR_TOWARD_MIN);
    unsigned long timeout = millis() + HOMING_TIMEOUT_MS;
    while (!isElbowMinHit() && millis() < timeout) {
      stepMotor(STEP_PIN_3);
      delayMicroseconds(HOMING_SPEED_US);
    }
    if (!isElbowMinHit()) {
      Serial.println(F("# [J3] ERROR: MIN limit switch not reached! Check switch on pin A1."));
      return;
    }
  }

  Serial.println(F("# [J3]   MIN limit reached (-2.700 rad). Backing off..."));
  digitalWrite(DIR_PIN_3, ELBOW_DIR_TOWARD_MAX);
  for (int i = 0; i < BACKOFF_STEPS; i++) {
    stepMotor(STEP_PIN_3);
    delayMicroseconds(BACKOFF_SPEED_US);
  }
  delay(300);
  raw_at_el_min = readAS5600RawAngle(ENCODER_CHANNELS[2]);
  Serial.print(F("# [J3]   Encoder raw at MIN (-2.700 rad): "));
  Serial.println(raw_at_el_min);

  // Phase 2: Traverse toward MAX limit switch on Pin A2 (+2.700 rad)
  Serial.println(F("# [J3] Phase 2: Traversing to MAX limit switch (Pin A2 / +2.700 rad)..."));
  digitalWrite(DIR_PIN_3, ELBOW_DIR_TOWARD_MAX);
  unsigned long step_count = 0;
  unsigned long timeout2 = millis() + TRAVERSE_TIMEOUT_MS;
  while (!isElbowMaxHit() && millis() < timeout2) {
    stepMotor(STEP_PIN_3);
    delayMicroseconds(HOMING_SPEED_US);
    step_count++;
  }
  if (!isElbowMaxHit()) {
    Serial.println(F("# [J3] ERROR: MAX limit switch not reached! Check switch on pin A2."));
    return;
  }

  Serial.print(F("# [J3]   MAX limit reached (+2.700 rad) after "));
  Serial.print(step_count);
  Serial.println(F(" steps. Backing off..."));
  digitalWrite(DIR_PIN_3, ELBOW_DIR_TOWARD_MIN);
  for (int i = 0; i < BACKOFF_STEPS; i++) {
    stepMotor(STEP_PIN_3);
    delayMicroseconds(BACKOFF_SPEED_US);
  }
  delay(300);
  raw_at_el_max = readAS5600RawAngle(ENCODER_CHANNELS[2]);
  total_el_steps = step_count;
  Serial.print(F("# [J3]   Encoder raw at MAX (+2.700 rad): "));
  Serial.println(raw_at_el_max);

  // Phase 3: Compute encoder direction & resolution
  int16_t raw_diff = (int16_t)raw_at_el_max - (int16_t)raw_at_el_min;
  if (raw_diff > 2048)  raw_diff -= 4096;
  if (raw_diff < -2048) raw_diff += 4096;
  encoder_directions[2] = (raw_diff > 0) ? 1 : -1;

  Serial.print(F("# [J3]   Encoder direction: "));
  Serial.println(encoder_directions[2] > 0 ? "POSITIVE" : "NEGATIVE");
  Serial.print(F("# [J3]   Total arc steps:   "));
  Serial.println(total_el_steps);

  // Phase 4: Return to midpoint (0.000 rad)
  // Range is symmetric: -2.700 rad to +2.700 rad => Midpoint is exactly 0.000 rad!
  unsigned long usable_range = step_count - BACKOFF_STEPS;
  unsigned long return_steps = usable_range / 2;
  Serial.print(F("# [J3] Phase 4: Returning "));
  Serial.print(return_steps);
  Serial.println(F(" steps to 0.0 rad midpoint..."));
  digitalWrite(DIR_PIN_3, ELBOW_DIR_TOWARD_MIN);
  for (unsigned long i = 0; i < return_steps; i++) {
    if (isElbowMinHit()) break;
    stepMotor(STEP_PIN_3);
    delayMicroseconds(HOMING_SPEED_US);
  }
  delay(300);

  // Phase 5: Re-zero encoder at midpoint (0.0 rad)
  uint16_t raw_mid = readAS5600RawAngle(ENCODER_CHANNELS[2]);
  encoder_offsets[2] = rawAngleToRadians(raw_mid);
  elbow_cal_ok = true;
  Serial.print(F("# [J3]   Encoder offset at 0.0 rad: "));
  Serial.println(encoder_offsets[2], 4);
  Serial.println(F("# [J3] Elbow calibration COMPLETE (Homed at 0.0 rad)."));
}

// ═══════════════════════════════════════════════════════════════
//  CALIBRATE ALL JOINTS
// ═══════════════════════════════════════════════════════════════
void autoCalibrateAll() {
  autoCalibrateShoulder();
  autoCalibrateElbow();
}

// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);  // 400kHz Fast I2C

  // Setup Shoulder Limit Switches with pull-ups
  pinMode(SHOULDER_LIMIT_MIN_PIN, INPUT_PULLUP);
  pinMode(SHOULDER_LIMIT_MAX_PIN, INPUT_PULLUP);

  // Setup Elbow Limit Switches with pull-ups
  pinMode(ELBOW_LIMIT_MIN_PIN, INPUT_PULLUP);
  pinMode(ELBOW_LIMIT_MAX_PIN, INPUT_PULLUP);

  // Setup Stepper Pins
  const uint8_t out_pins[] = {
    STEP_PIN_1, DIR_PIN_1, ENA_PIN_1,
    STEP_PIN_2, DIR_PIN_2, ENA_PIN_2,
    STEP_PIN_3, DIR_PIN_3, ENA_PIN_3
  };
  for (uint8_t p : out_pins) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }

  Serial.println(F(""));
  Serial.println(F("# EB-15 Dual-Joint Auto-Calibration Firmware Initialized."));
  Serial.println(F("# Joint 2 limits on D3 (MIN), D2 (MAX)."));
  Serial.println(F("# Joint 3 limits on A1 (MIN -2.700), A2 (MAX +2.700)."));
  delay(500);

  // ── RUN DUAL AUTO-CALIBRATION ON POWER-ON ──
  autoCalibrateAll();

  if (shoulder_cal_ok && elbow_cal_ok) {
    enableMotors(true);
    Serial.println(F("# All joints calibrated and homed at 0.0 rad. System ready."));
  } else {
    Serial.println(F("# WARNING: Calibration incomplete for one or more joints."));
    Serial.println(F("# Send $HOME to retry calibration."));
  }
}

// ═══════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════════
void loop() {
  // 1. Read Encoders
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    uint16_t raw = readAS5600RawAngle(ENCODER_CHANNELS[i]);
    float rad = rawAngleToRadians(raw);
    joint_positions[i] = (rad - encoder_offsets[i]) * encoder_directions[i];
  }

  // 2. Hardware Safety Check
  if (isShoulderMinHit() || isShoulderMaxHit() || isElbowMinHit() || isElbowMaxHit()) {
    // Safety check during operation
  }

  // 3. Process Incoming Serial Commands
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("$CMD,")) {
      // Format: $CMD,q1,q2,q3
      int c1 = cmd.indexOf(',', 5);
      int c2 = cmd.indexOf(',', c1 + 1);
      if (c1 > 0 && c2 > 0) {
        target_positions[0] = cmd.substring(5, c1).toFloat();
        target_positions[1] = cmd.substring(c1 + 1, c2).toFloat();
        target_positions[2] = cmd.substring(c2 + 1).toFloat();
      }
    }
    else if (cmd == "$HOME") {
      Serial.println(F("# $HOME received - running full calibration..."));
      autoCalibrateAll();
    }
    else if (cmd == "$HOME_SHOULDER") {
      autoCalibrateShoulder();
    }
    else if (cmd == "$HOME_ELBOW") {
      autoCalibrateElbow();
    }
    else if (cmd.startsWith("$SET_ZERO")) {
      for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        uint16_t raw = readAS5600RawAngle(ENCODER_CHANNELS[i]);
        encoder_offsets[i] = rawAngleToRadians(raw);
      }
      Serial.println(F("# Encoders zeroed to current pose."));
    }
    else if (cmd == "$STATUS") {
      Serial.println(F("# === CALIBRATION STATUS ==="));
      Serial.print(F("# J2 Calibrated: "));
      Serial.println(shoulder_cal_ok ? "YES" : "NO");
      Serial.print(F("# J2 Steps:      "));
      Serial.println(total_sh_steps);
      Serial.print(F("# J2 Offset:     "));
      Serial.println(encoder_offsets[1], 4);
      Serial.print(F("# J3 Calibrated: "));
      Serial.println(elbow_cal_ok ? "YES" : "NO");
      Serial.print(F("# J3 Steps:      "));
      Serial.println(total_el_steps);
      Serial.print(F("# J3 Offset:     "));
      Serial.println(encoder_offsets[2], 4);
    }
  }

  // 4. Stream Telemetry at 20 Hz
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
    Serial.print(isShoulderMinHit() ? 1 : 0);
    Serial.print(F(","));
    Serial.print(isShoulderMaxHit() ? 1 : 0);
    Serial.print(F(","));
    Serial.print(isElbowMinHit() ? 1 : 0);
    Serial.print(F(","));
    Serial.print(isElbowMaxHit() ? 1 : 0);
    Serial.print(F(","));
    Serial.println((shoulder_cal_ok && elbow_cal_ok) ? 1 : 0);
  }
}
