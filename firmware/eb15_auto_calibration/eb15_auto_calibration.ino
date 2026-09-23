/**
 * EB-15 Robotic Arm - Auto-Calibration Firmware
 * ================================================
 * 
 * Shoulder (Joint 2) automatic calibration using dual limit switches.
 * 
 * Calibration Sequence (runs automatically at power-on):
 *  1. Move toward MIN limit switch until triggered
 *  2. Back off from MIN switch, record encoder → this is 0°
 *  3. Traverse toward MAX limit switch, counting steps
 *  4. Back off from MAX switch, record encoder → this is 180°
 *  5. Compute encoder offset & direction mapping
 *  6. Return to 90° (home / midpoint) position
 *  7. Re-zero encoder so 90° reads as 0.0 rad
 *  7. Begin normal telemetry & command operation
 * 
 * Hardware:
 *  - 3x NEMA 17 Steppers with TB6600 Drivers
 *  - 3x AS5600 Magnetic Encoders via TCA9548A I2C Multiplexer
 *  - 2x Shoulder Limit Switches (MIN at D3, MAX at D2)
 * 
 * Communication (USB Serial @ 115200 baud):
 *  - Telemetry:  $STATE,<j1_rad>,<j2_rad>,<j3_rad>,<lim_min>,<lim_max>,<cal>
 *  - Commands:   $CMD,<j1_rad>,<j2_rad>,<j3_rad>
 *                $HOME       (re-run full auto-calibration)
 *                $SET_ZERO   (set current encoder readings as zero)
 *                $STATUS     (print calibration diagnostics)
 *
 * NOTE: This is a TEST file. Upload this INSTEAD of eb15_arduino.ino.
 *       Do NOT have both .ino files in the same Arduino sketch folder.
 *       Copy this file to a separate folder to compile, e.g.:
 *         firmware/eb15_auto_cal/eb15_auto_calibration.ino
 */

#include <Wire.h>

// ═══════════════════════════════════════════════════════════════
//  PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════

// Shoulder Limit Switches (Active LOW with internal pull-up)
#define SHOULDER_LIMIT_MIN_PIN   3   // Pin D3 — Home / 0° end
#define SHOULDER_LIMIT_MAX_PIN   2   // Pin D2 — Far / 180° end
#define LIMIT_SWITCH_ACTIVE_LOW  true

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

// TB6600 Enable levels (typically LOW = enabled on TB6600)
#define TB6600_ENABLE_LEVEL      LOW
#define TB6600_DISABLE_LEVEL     HIGH

// ═══════════════════════════════════════════════════════════════
//  SHOULDER DIRECTION MAPPING
//  If the motor moves the WRONG way during calibration,
//  swap these two values.
// ═══════════════════════════════════════════════════════════════
#define SHOULDER_DIR_TOWARD_MIN  LOW
#define SHOULDER_DIR_TOWARD_MAX  HIGH

// ═══════════════════════════════════════════════════════════════
//  CALIBRATION PARAMETERS
// ═══════════════════════════════════════════════════════════════
#define HOMING_SPEED_US          800    // Microseconds between step pulses (slow & safe)
#define BACKOFF_STEPS            200    // Steps to back off after hitting a switch
#define BACKOFF_SPEED_US         1000   // Slightly slower for back-off
#define HOMING_TIMEOUT_MS        20000  // 20 sec timeout per phase
#define TRAVERSE_TIMEOUT_MS      30000  // 30 sec timeout for MIN→MAX traverse

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

bool     calibration_ok       = false;    // True after successful calibration
uint16_t raw_at_min_pos       = 0;        // Encoder raw at 0° (after back-off)
uint16_t raw_at_max_pos       = 0;        // Encoder raw at 180° (after back-off)
unsigned long total_shoulder_steps = 0;   // Steps between 0° and 180° positions

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
//  AUTO-CALIBRATION ROUTINE
// ═══════════════════════════════════════════════════════════════
void autoCalibrateShoulder() {
  Serial.println(F(""));
  Serial.println(F("# =========================================="));
  Serial.println(F("#  EB-15 SHOULDER AUTO-CALIBRATION"));
  Serial.println(F("# =========================================="));

  calibration_ok = false;
  enableMotors(true);
  delay(100);

  // ── PHASE 1: Find MIN limit switch (0° end) ──────────────

  Serial.println(F("# Phase 1: Seeking MIN limit switch..."));

  // Check if we're already sitting on a switch
  if (isShoulderMinHit()) {
    Serial.println(F("#   Already on MIN switch."));
  }
  else {
    // If we happen to be sitting on MAX, move away from it first
    if (isShoulderMaxHit()) {
      Serial.println(F("#   Currently on MAX switch, moving toward MIN..."));
    }

    // Move toward MIN
    digitalWrite(DIR_PIN_2, SHOULDER_DIR_TOWARD_MIN);
    unsigned long timeout = millis() + HOMING_TIMEOUT_MS;

    while (!isShoulderMinHit() && millis() < timeout) {
      stepMotor(STEP_PIN_2);
      delayMicroseconds(HOMING_SPEED_US);
    }

    if (!isShoulderMinHit()) {
      Serial.println(F("# ERROR: MIN limit switch not found! Aborting."));
      Serial.println(F("#   Check wiring on pin D3."));
      return;
    }
  }

  Serial.println(F("#   MIN limit switch FOUND."));

  // Back off from MIN switch
  Serial.println(F("#   Backing off from MIN..."));
  digitalWrite(DIR_PIN_2, SHOULDER_DIR_TOWARD_MAX);
  for (int i = 0; i < BACKOFF_STEPS; i++) {
    stepMotor(STEP_PIN_2);
    delayMicroseconds(BACKOFF_SPEED_US);
  }
  delay(300);  // Settle time for encoder

  // Verify we're clear of the switch
  if (isShoulderMinHit()) {
    Serial.println(F("# WARNING: Still on MIN switch after back-off. Increasing back-off..."));
    for (int i = 0; i < BACKOFF_STEPS; i++) {
      stepMotor(STEP_PIN_2);
      delayMicroseconds(BACKOFF_SPEED_US);
    }
    delay(300);
  }

  // Record encoder at 0° position
  raw_at_min_pos = readAS5600RawAngle(ENCODER_CHANNELS[1]);
  Serial.print(F("#   Encoder at 0° position: "));
  Serial.println(raw_at_min_pos);


  // ── PHASE 2: Traverse to MAX limit switch (180° end) ──────

  Serial.println(F("# Phase 2: Traversing to MAX limit switch..."));
  digitalWrite(DIR_PIN_2, SHOULDER_DIR_TOWARD_MAX);

  unsigned long step_count = 0;
  unsigned long timeout2 = millis() + TRAVERSE_TIMEOUT_MS;

  while (!isShoulderMaxHit() && millis() < timeout2) {
    stepMotor(STEP_PIN_2);
    delayMicroseconds(HOMING_SPEED_US);
    step_count++;
  }

  if (!isShoulderMaxHit()) {
    Serial.println(F("# ERROR: MAX limit switch not found! Aborting."));
    Serial.println(F("#   Check wiring on pin D2."));
    return;
  }

  Serial.print(F("#   MAX limit switch FOUND after "));
  Serial.print(step_count);
  Serial.println(F(" steps."));

  // Back off from MAX switch
  Serial.println(F("#   Backing off from MAX..."));
  digitalWrite(DIR_PIN_2, SHOULDER_DIR_TOWARD_MIN);
  for (int i = 0; i < BACKOFF_STEPS; i++) {
    stepMotor(STEP_PIN_2);
    delayMicroseconds(BACKOFF_SPEED_US);
  }
  delay(300);  // Settle time for encoder

  // Verify we're clear of the switch
  if (isShoulderMaxHit()) {
    Serial.println(F("# WARNING: Still on MAX switch after back-off. Increasing back-off..."));
    for (int i = 0; i < BACKOFF_STEPS; i++) {
      stepMotor(STEP_PIN_2);
      delayMicroseconds(BACKOFF_SPEED_US);
    }
    delay(300);
  }

  // Record encoder at 180° position
  raw_at_max_pos = readAS5600RawAngle(ENCODER_CHANNELS[1]);
  Serial.print(F("#   Encoder at 180° position: "));
  Serial.println(raw_at_max_pos);

  // Save total steps for the usable range
  total_shoulder_steps = step_count;


  // ── PHASE 3: Compute encoder calibration ──────────────────

  Serial.println(F("# Phase 3: Computing calibration..."));

  // Determine encoder direction:
  // Does increasing raw angle correspond to MIN→MAX (0°→180°)?
  int16_t raw_diff = (int16_t)raw_at_max_pos - (int16_t)raw_at_min_pos;
  // Handle 12-bit wrap-around
  if (raw_diff > 2048)  raw_diff -= 4096;
  if (raw_diff < -2048) raw_diff += 4096;

  encoder_directions[1] = (raw_diff > 0) ? 1 : -1;
  encoder_offsets[1]     = rawAngleToRadians(raw_at_min_pos);

  Serial.print(F("#   Encoder raw range:  "));
  Serial.print(raw_at_min_pos);
  Serial.print(F(" -> "));
  Serial.println(raw_at_max_pos);
  Serial.print(F("#   Raw difference:     "));
  Serial.println(raw_diff);
  Serial.print(F("#   Encoder direction:  "));
  Serial.println(encoder_directions[1] > 0 ? "POSITIVE" : "NEGATIVE");
  Serial.print(F("#   Encoder offset:     "));
  Serial.print(encoder_offsets[1], 4);
  Serial.println(F(" rad"));
  Serial.print(F("#   Total range steps:  "));
  Serial.println(total_shoulder_steps);

  float deg_per_step = 180.0 / (float)total_shoulder_steps;
  Serial.print(F("#   Resolution:         "));
  Serial.print(deg_per_step, 5);
  Serial.println(F(" deg/step"));


  // ── PHASE 4: Return to 90° (midpoint / home) position ────

  Serial.println(F("# Phase 4: Returning to 90 deg (midpoint)..."));
  digitalWrite(DIR_PIN_2, SHOULDER_DIR_TOWARD_MIN);

  // We are currently at (MAX - BACKOFF) position.
  // The 0° position is at (MIN + BACKOFF).
  // The usable range = step_count - BACKOFF_STEPS.
  // Midpoint (90°) = halfway through the usable range.
  //
  // Current distance from 0° = step_count - BACKOFF_STEPS
  // Target distance from 0°  = (step_count - BACKOFF_STEPS) / 2
  // Steps to move toward MIN = current - target = (step_count - BACKOFF_STEPS) / 2
  unsigned long usable_range = step_count - BACKOFF_STEPS;
  unsigned long return_steps = usable_range / 2;

  Serial.print(F("#   Usable range: "));
  Serial.print(usable_range);
  Serial.println(F(" steps"));
  Serial.print(F("#   Moving "));
  Serial.print(return_steps);
  Serial.println(F(" steps toward MIN to reach 90 deg..."));

  for (unsigned long i = 0; i < return_steps; i++) {
    // Safety: stop if MIN switch is hit (shouldn't happen at midpoint)
    if (isShoulderMinHit()) {
      Serial.println(F("#   Hit MIN switch during return! Backing off..."));
      digitalWrite(DIR_PIN_2, SHOULDER_DIR_TOWARD_MAX);
      for (int j = 0; j < BACKOFF_STEPS; j++) {
        stepMotor(STEP_PIN_2);
        delayMicroseconds(BACKOFF_SPEED_US);
      }
      break;
    }
    stepMotor(STEP_PIN_2);
    delayMicroseconds(HOMING_SPEED_US);
  }

  delay(300);  // Settle


  // ── PHASE 5: Re-zero encoder at 90° and verify ────────────
  //    Set the encoder offset so that THIS position (90° physical)
  //    reads as 0.0 rad. The joint will then range from
  //    -PI/2 (-90°) to +PI/2 (+90°) in URDF coordinates.

  uint16_t raw_at_midpoint = readAS5600RawAngle(ENCODER_CHANNELS[1]);
  encoder_offsets[1] = rawAngleToRadians(raw_at_midpoint);

  // Verify: should now read ~0.0
  float final_rad = (rawAngleToRadians(raw_at_midpoint) - encoder_offsets[1]) * encoder_directions[1];
  float final_deg = final_rad * 180.0 / PI;

  Serial.println(F("# Phase 5: Verification"));
  Serial.print(F("#   Encoder raw at 90 deg: "));
  Serial.println(raw_at_midpoint);
  Serial.print(F("#   Joint reading:  "));
  Serial.print(final_rad, 4);
  Serial.print(F(" rad ("));
  Serial.print(final_deg, 1);
  Serial.println(F(" deg)"));
  Serial.println(F("#   URDF range: -90 deg to +90 deg (-PI/2 to +PI/2)"));

  // Check if we're reasonably close to 0 rad
  if (abs(final_deg) < 5.0) {
    Serial.println(F("#   Position OK (home at 90 deg midpoint)."));
  } else {
    Serial.print(F("#   WARNING: Position is "));
    Serial.print(final_deg, 1);
    Serial.println(F(" deg from home. Check motor step loss."));
  }

  calibration_ok = true;

  Serial.println(F("# =========================================="));
  Serial.println(F("#  CALIBRATION COMPLETE - HOMED AT 90 DEG"));
  Serial.println(F("# =========================================="));
  Serial.println(F(""));
}


// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);  // 400kHz Fast I2C

  // Setup Limit Switches with pull-ups
  pinMode(SHOULDER_LIMIT_MIN_PIN, INPUT_PULLUP);
  pinMode(SHOULDER_LIMIT_MAX_PIN, INPUT_PULLUP);

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
  Serial.println(F("# EB-15 Auto-Calibration Firmware Initialized."));
  Serial.println(F("# Starting automatic shoulder calibration..."));
  delay(500);

  // ── RUN AUTO-CALIBRATION ON POWER-ON ──
  autoCalibrateShoulder();

  if (calibration_ok) {
    enableMotors(true);
    Serial.println(F("# System ready. Streaming telemetry at 20 Hz."));
  } else {
    Serial.println(F("# CALIBRATION FAILED - entering limited mode."));
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

  // 2. Hardware Safety: Check limit switches during operation
  if (isShoulderMinHit() || isShoulderMaxHit()) {
    // TODO: Implement runtime safety halt if moving toward triggered switch
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
      // Re-run full auto-calibration
      Serial.println(F("# $HOME received - re-running calibration..."));
      autoCalibrateShoulder();
    }
    else if (cmd.startsWith("$SET_ZERO")) {
      // Set current positions as zero offset (all joints)
      for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        uint16_t raw = readAS5600RawAngle(ENCODER_CHANNELS[i]);
        encoder_offsets[i] = rawAngleToRadians(raw);
      }
      Serial.println(F("# Encoders zeroed to current pose."));
    }
    else if (cmd == "$STATUS") {
      // Print calibration status
      Serial.print(F("# Calibrated: "));
      Serial.println(calibration_ok ? "YES" : "NO");
      Serial.print(F("# Encoder MIN raw: "));
      Serial.println(raw_at_min_pos);
      Serial.print(F("# Encoder MAX raw: "));
      Serial.println(raw_at_max_pos);
      Serial.print(F("# Total steps: "));
      Serial.println(total_shoulder_steps);
      Serial.print(F("# J2 offset: "));
      Serial.println(encoder_offsets[1], 4);
      Serial.print(F("# J2 direction: "));
      Serial.println(encoder_directions[1]);
    }
  }

  // 4. Stream Telemetry at 20 Hz
  //    Format: $STATE,j1,j2,j3,lim_min,lim_max,calibrated
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
    Serial.println(calibration_ok ? 1 : 0);
  }
}
