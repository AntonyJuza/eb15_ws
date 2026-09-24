/**
 * EB-15 Elbow (Joint 3) Motor Movement & Auto-Calibration Tool
 * ============================================================
 * 
 * Hardware:
 *  - TB6600 Driver Motor 3: STEP = Pin 10, DIR = Pin 11, ENA = Pin 12
 *  - Elbow Limit Switches:  MIN (-2.700 rad) = Pin A1, MAX (+2.700 rad) = Pin A2
 *  - AS5600 Encoder:        TCA9548A Channel 2 (I2C 0x70 / 0x36)
 * 
 * Interactive Serial Commands (115200 baud):
 *  - CAL   : Run full automatic calibration (MIN -> MAX -> Center at 0.0 rad)
 *  - MIN   : Seek MIN limit switch on A1 (-2.700 rad) and back off
 *  - MAX   : Seek MAX limit switch on A2 (+2.700 rad) and back off
 *  - MID   : Return to 0.0 rad midpoint
 *  - +     : Jog forward 100 steps
 *  - -     : Jog backward 100 steps
 *  - ++    : Jog forward 500 steps
 *  - --    : Jog backward 500 steps
 *  - REV   : Invert motor direction (if motor runs backwards during homing)
 *  - STAT  : Print live encoder angle, limit switches, and calibration values
 *  - ENA   : Enable stepper driver
 *  - DIS   : Disable stepper driver (freewheel)
 */

#include <Wire.h>

// ─── Pin Definitions ─────────────────────────────────────────
#define STEP_PIN_3               10
#define DIR_PIN_3                11
#define ENA_PIN_3                12

#define ELBOW_LIMIT_MIN_PIN      A1  // Folded limit (-2.700 rad)
#define ELBOW_LIMIT_MAX_PIN      A2  // Extended limit (+2.700 rad)
#define LIMIT_SWITCH_ACTIVE_LOW  true

#define TB6600_ENABLE_LEVEL      LOW
#define TB6600_DISABLE_LEVEL     HIGH

// Direction levels (can be toggled in software with "REV")
uint8_t dir_toward_min = LOW;
uint8_t dir_toward_max = HIGH;

// Calibration parameters
#define HOMING_SPEED_US          800    // Step delay in microseconds
#define JOG_SPEED_US             700    // Jog step delay
#define BACKOFF_STEPS            200    // Steps to back off after hitting a switch
#define BACKOFF_SPEED_US         1000
#define HOMING_TIMEOUT_MS        25000  // 25 sec max per homing move

// Joint 3 limits in radians
const float J3_MIN_RAD = -2.700f;
const float J3_MAX_RAD =  2.700f;

// ─── I2C Multiplexer & Encoder Settings ─────────────────────
#define TCA9548A_ADDR            0x70
#define AS5600_ADDR              0x36
#define AS5600_RAW_ANGLE_REG     0x0C
#define ELBOW_ENCODER_CHANNEL    2

// ─── Calibration State ───────────────────────────────────────
bool          calibrated       = false;
uint16_t      raw_at_min       = 0;
uint16_t      raw_at_max       = 0;
unsigned long total_arc_steps  = 0;
float         encoder_offset   = 0.0f;
int8_t        encoder_dir      = 1;

// ─── I2C & Encoder Helpers ───────────────────────────────────
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

uint16_t readEncoderRaw() {
  tcaSelect(ELBOW_ENCODER_CHANNEL);
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

float rawToRadians(uint16_t raw) {
  if (raw > 4095) return 0.0f;
  float rad = ((float)raw / 4096.0f) * 2.0f * PI;
  if (rad > PI) rad -= 2.0f * PI;
  return rad;
}

float getJointAngleRad() {
  uint16_t raw = readEncoderRaw();
  if (raw > 4095) return 0.0f;
  float rad = rawToRadians(raw);
  return (rad - encoder_offset) * encoder_dir;
}

// ─── Motor Helpers ───────────────────────────────────────────
void enableMotor(bool enable) {
  digitalWrite(ENA_PIN_3, enable ? TB6600_ENABLE_LEVEL : TB6600_DISABLE_LEVEL);
}

void stepOnce(uint16_t delay_us) {
  digitalWrite(STEP_PIN_3, HIGH);
  delayMicroseconds(5);
  digitalWrite(STEP_PIN_3, LOW);
  delayMicroseconds(delay_us);
}

// ─── Limit Switch Checks ─────────────────────────────────────
bool isMinHit() {
  return digitalRead(ELBOW_LIMIT_MIN_PIN) == (LIMIT_SWITCH_ACTIVE_LOW ? LOW : HIGH);
}

bool isMaxHit() {
  return digitalRead(ELBOW_LIMIT_MAX_PIN) == (LIMIT_SWITCH_ACTIVE_LOW ? LOW : HIGH);
}

// ─── Seek MIN Limit Switch ───────────────────────────────────
bool seekMinLimit() {
  Serial.println(F("\n[STEP 1] Moving toward MIN limit switch (Pin A1)..."));
  enableMotor(true);
  digitalWrite(DIR_PIN_3, dir_toward_min);

  unsigned long timeout = millis() + HOMING_TIMEOUT_MS;
  while (!isMinHit() && millis() < timeout) {
    stepOnce(HOMING_SPEED_US);
  }

  if (!isMinHit()) {
    Serial.println(F("[ERROR] MIN limit switch not reached! Check wiring on Pin A1 or motor direction."));
    return false;
  }

  Serial.println(F("[HIT] MIN limit switch reached! Backing off 200 steps..."));
  digitalWrite(DIR_PIN_3, dir_toward_max);
  for (int i = 0; i < BACKOFF_STEPS; i++) {
    stepOnce(BACKOFF_SPEED_US);
  }
  delay(300);

  raw_at_min = readEncoderRaw();
  Serial.print(F("[OK] Encoder raw at MIN (-2.700 rad): "));
  Serial.println(raw_at_min);
  return true;
}

// ─── Seek MAX Limit Switch ───────────────────────────────────
bool seekMaxLimit(unsigned long &counted_steps) {
  Serial.println(F("\n[STEP 2] Moving toward MAX limit switch (Pin A2)..."));
  enableMotor(true);
  digitalWrite(DIR_PIN_3, dir_toward_max);

  counted_steps = 0;
  unsigned long timeout = millis() + HOMING_TIMEOUT_MS;
  while (!isMaxHit() && millis() < timeout) {
    stepOnce(HOMING_SPEED_US);
    counted_steps++;
  }

  if (!isMaxHit()) {
    Serial.println(F("[ERROR] MAX limit switch not reached! Check wiring on Pin A2."));
    return false;
  }

  Serial.print(F("[HIT] MAX limit switch reached after "));
  Serial.print(counted_steps);
  Serial.println(F(" steps! Backing off 200 steps..."));

  digitalWrite(DIR_PIN_3, dir_toward_min);
  for (int i = 0; i < BACKOFF_STEPS; i++) {
    stepOnce(BACKOFF_SPEED_US);
  }
  delay(300);

  raw_at_max = readEncoderRaw();
  Serial.print(F("[OK] Encoder raw at MAX (+2.700 rad): "));
  Serial.println(raw_at_max);
  return true;
}

// ─── Full Auto-Calibration Routine ───────────────────────────
void runAutoCalibration() {
  Serial.println(F("\n========================================================"));
  Serial.println(F("      STARTING ELBOW (JOINT 3) AUTO-CALIBRATION         "));
  Serial.println(F("========================================================"));

  calibrated = false;

  // Phase 1: Seek MIN
  if (!seekMinLimit()) return;

  // Phase 2: Traverse to MAX
  unsigned long steps = 0;
  if (!seekMaxLimit(steps)) return;

  total_arc_steps = steps;

  // Phase 3: Direction & Resolution
  int16_t diff = (int16_t)raw_at_max - (int16_t)raw_at_min;
  if (diff > 2048)  diff -= 4096;
  if (diff < -2048) diff += 4096;
  encoder_dir = (diff > 0) ? 1 : -1;

  float total_rad = J3_MAX_RAD - J3_MIN_RAD; // 5.400 rad
  float deg_total = total_rad * 180.0f / PI; // 309.4 deg
  float steps_per_deg = (float)total_arc_steps / deg_total;

  Serial.println(F("\n[STEP 3] Calibration Calculations:"));
  Serial.print(F("  - Total steps across arc: "));
  Serial.println(total_arc_steps);
  Serial.print(F("  - Encoder direction:      "));
  Serial.println(encoder_dir > 0 ? F("POSITIVE") : F("NEGATIVE"));
  Serial.print(F("  - Total motion range:     "));
  Serial.print(total_rad, 3);
  Serial.print(F(" rad ("));
  Serial.print(deg_total, 1);
  Serial.println(F(" deg)"));
  Serial.print(F("  - Steps / degree:         "));
  Serial.println(steps_per_deg, 2);

  // Phase 4: Return to 0.0 rad (Midpoint)
  unsigned long usable_range = total_arc_steps - BACKOFF_STEPS;
  unsigned long return_steps = usable_range / 2;

  Serial.print(F("\n[STEP 4] Moving "));
  Serial.print(return_steps);
  Serial.println(F(" steps to 0.0 rad midpoint..."));

  digitalWrite(DIR_PIN_3, dir_toward_min);
  for (unsigned long i = 0; i < return_steps; i++) {
    if (isMinHit()) {
      Serial.println(F("[WARN] MIN hit during return! Stopping."));
      break;
    }
    stepOnce(HOMING_SPEED_US);
  }
  delay(300);

  // Phase 5: Re-zero encoder at midpoint
  uint16_t raw_mid = readEncoderRaw();
  encoder_offset = rawToRadians(raw_mid);
  calibrated = true;

  float final_angle = getJointAngleRad();
  Serial.println(F("\n[STEP 5] Encoder Re-zeroed:"));
  Serial.print(F("  - Raw at midpoint:        "));
  Serial.println(raw_mid);
  Serial.print(F("  - Encoder offset:         "));
  Serial.print(encoder_offset, 4);
  Serial.println(F(" rad"));
  Serial.print(F("  - Verified current angle: "));
  Serial.print(final_angle, 4);
  Serial.println(F(" rad (~0.0 rad)"));

  Serial.println(F("\n========================================================"));
  Serial.println(F("           CALIBRATION COMPLETE & HOMED                 "));
  Serial.println(F("========================================================\n"));
}

// ─── Jog Motor ───────────────────────────────────────────────
void jogMotor(int steps, uint8_t dir) {
  enableMotor(true);
  digitalWrite(DIR_PIN_3, dir);
  for (int i = 0; i < steps; i++) {
    if (dir == dir_toward_min && isMinHit()) {
      Serial.println(F(">>> [STOP] Safety halt: Elbow MIN limit (A1) hit!"));
      break;
    }
    if (dir == dir_toward_max && isMaxHit()) {
      Serial.println(F(">>> [STOP] Safety halt: Elbow MAX limit (A2) hit!"));
      break;
    }
    stepOnce(JOG_SPEED_US);
  }
  printStatus();
}

// ─── Print Status ────────────────────────────────────────────
void printStatus() {
  uint16_t raw = readEncoderRaw();
  float rad = getJointAngleRad();
  float deg = rad * 180.0f / PI;

  Serial.print(F("[STATUS] Angle: "));
  Serial.print(rad, 3);
  Serial.print(F(" rad ("));
  Serial.print(deg, 1);
  Serial.print(F(" deg) | Raw: "));
  Serial.print(raw);

  Serial.print(F(" | MIN (A1): "));
  Serial.print(isMinHit() ? F("HIT (0)") : F("OPEN (1)"));

  Serial.print(F(" | MAX (A2): "));
  Serial.print(isMaxHit() ? F("HIT (0)") : F("OPEN (1)"));

  Serial.print(F(" | Calibrated: "));
  Serial.println(calibrated ? F("YES") : F("NO"));
}

// ─── Setup ───────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  Wire.begin();
  Wire.setClock(400000);

  // Setup motor pins
  pinMode(STEP_PIN_3, OUTPUT);
  pinMode(DIR_PIN_3,  OUTPUT);
  pinMode(ENA_PIN_3,  OUTPUT);
  enableMotor(true);

  // Setup limit switch pins
  pinMode(ELBOW_LIMIT_MIN_PIN, INPUT_PULLUP);
  pinMode(ELBOW_LIMIT_MAX_PIN, INPUT_PULLUP);

  Serial.println(F("\n========================================================"));
  Serial.println(F("    EB-15 ELBOW (JOINT 3) MOTOR TEST & CALIBRATION      "));
  Serial.println(F("========================================================"));
  Serial.println(F("Pins: STEP=10, DIR=11, ENA=12 | MIN_SW=A1, MAX_SW=A2"));
  Serial.println(F("Encoder: TCA9548A Ch 2 (AS5600 0x36)"));
  Serial.println(F("--------------------------------------------------------"));
  Serial.println(F("Commands:"));
  Serial.println(F("  CAL   - Run full auto-calibration (MIN -> MAX -> 0.0 rad)"));
  Serial.println(F("  MIN   - Seek MIN limit switch on Pin A1"));
  Serial.println(F("  MAX   - Seek MAX limit switch on Pin A2"));
  Serial.println(F("  +     - Jog toward MAX (+100 steps)"));
  Serial.println(F("  -     - Jog toward MIN (-100 steps)"));
  Serial.println(F("  ++    - Jog toward MAX (+500 steps)"));
  Serial.println(F("  --    - Jog toward MIN (-500 steps)"));
  Serial.println(F("  REV   - Reverse motor directions"));
  Serial.println(F("  STAT  - Print current angle and switch states"));
  Serial.println(F("  ENA   - Enable motor driver"));
  Serial.println(F("  DIS   - Disable motor driver"));
  Serial.println(F("========================================================\n"));

  printStatus();
}

// ─── Main Loop (Command Processor) ───────────────────────────
void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "CAL") {
      runAutoCalibration();
    } else if (cmd == "MIN") {
      seekMinLimit();
      printStatus();
    } else if (cmd == "MAX") {
      unsigned long s = 0;
      seekMaxLimit(s);
      printStatus();
    } else if (cmd == "+") {
      jogMotor(100, dir_toward_max);
    } else if (cmd == "-") {
      jogMotor(100, dir_toward_min);
    } else if (cmd == "++") {
      jogMotor(500, dir_toward_max);
    } else if (cmd == "--") {
      jogMotor(500, dir_toward_min);
    } else if (cmd == "REV") {
      uint8_t tmp = dir_toward_min;
      dir_toward_min = dir_toward_max;
      dir_toward_max = tmp;
      Serial.println(F("[CONFIG] Stepper directions REVERSED."));
    } else if (cmd == "STAT") {
      printStatus();
    } else if (cmd == "ENA") {
      enableMotor(true);
      Serial.println(F("[MOTOR] Driver ENABLED."));
    } else if (cmd == "DIS") {
      enableMotor(false);
      Serial.println(F("[MOTOR] Driver DISABLED (Freewheel)."));
    } else {
      Serial.println(F("Unknown command. Type CAL, MIN, MAX, +, -, ++, --, STAT, or REV."));
    }
  }
}
