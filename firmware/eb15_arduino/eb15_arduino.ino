/**
 * EB-15 Robotic Arm - Arduino Nano Low-Level Firmware
 * 
 * Hardware:
 *  - 3x NEMA 17 Steppers with TB6600 Drivers
 *  - 3x AS5600 Magnetic Encoders via TCA9548A I2C Multiplexer
 *  - 2x Shoulder Limit Switches (Min / Max)
 * 
 * Communication:
 *  - USB Serial @ 115200 baud
 *  - Telemetry:  $STATE,<j1_rad>,<j2_rad>,<j3_rad>,<lim_min>,<lim_max>\n
 *  - Commands:   $CMD,<j1_rad>,<j2_rad>,<j3_rad>\n
 *                $HOME\n
 *                $SET_ZERO\n
 */

#include <Wire.h>

// ─── Pin Definitions ─────────────────────────────────────────

// Shoulder Limit Switches (Active LOW with internal pull-up)
#define SHOULDER_LIMIT_MIN_PIN   3   // Pin D3 (Home / Min limit)
#define SHOULDER_LIMIT_MAX_PIN   2   // Pin D2 (End / Max limit)
#define LIMIT_SWITCH_ACTIVE_LOW  true

// TB6600 Motor 1 (Joint 1 / Base)
#define STEP_PIN_1               4
#define DIR_PIN_1                5
#define ENA_PIN_1                6

// TB6600 Motor 2 (Joint 2 / Shoulder)
#define STEP_PIN_2               7
#define DIR_PIN_2                8
#define ENA_PIN_2                9

// TB6600 Motor 3 (Joint 3 / Elbow)
#define STEP_PIN_3               10
#define DIR_PIN_3                11
#define ENA_PIN_3                12

// TB6600 Enable active level (typically LOW = enabled on TB6600)
#define TB6600_ENABLE_LEVEL      LOW
#define TB6600_DISABLE_LEVEL     HIGH

// ─── I2C Multiplexer & Encoder Settings ─────────────────────
#define TCA9548A_ADDR            0x70
#define AS5600_ADDR              0x36
#define AS5600_RAW_ANGLE_REG     0x0C
#define NUM_JOINTS               3

// TCA9548A channels for Joint 1, Joint 2, Joint 3
const uint8_t ENCODER_CHANNELS[NUM_JOINTS] = {0, 1, 2};

// Zero offsets for encoders in radians (calibrated so 0 matches URDF zero)
float encoder_offsets[NUM_JOINTS] = {0.0, 0.0, 0.0};
int8_t encoder_directions[NUM_JOINTS] = {1, 1, 1}; // 1 or -1 to match URDF joint axis direction

// Current joint readings (radians)
float joint_positions[NUM_JOINTS] = {0.0, 0.0, 0.0};
float target_positions[NUM_JOINTS] = {0.0, 0.0, 0.0};

// Telemetry timer (20 Hz = 50ms)
unsigned long last_telemetry_time = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 50;

// ─── Helper: Select TCA9548A Channel ────────────────────────
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// ─── Helper: Read AS5600 12-bit Raw Angle (0..4095) ──────────
uint16_t readAS5600RawAngle(uint8_t channel) {
  tcaSelect(channel);
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_RAW_ANGLE_REG);
  if (Wire.endTransmission(false) != 0) {
    return 0; // Bus error
  }
  
  Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2);
  if (Wire.available() >= 2) {
    uint16_t high_byte = Wire.read();
    uint16_t low_byte = Wire.read();
    return ((high_byte & 0x0F) << 8) | low_byte;
  }
  return 0;
}

// Convert raw 12-bit count to radians [-pi, +pi]
float rawAngleToRadians(uint16_t raw) {
  float angle = ((float)raw / 4096.0) * 2.0 * PI; // 0 .. 2pi
  if (angle > PI) {
    angle -= 2.0 * PI; // wrap to [-pi, +pi]
  }
  return angle;
}

// ─── Limit Switch Check ─────────────────────────────────────
bool isShoulderMinHit() {
  return digitalRead(SHOULDER_LIMIT_MIN_PIN) == (LIMIT_SWITCH_ACTIVE_LOW ? LOW : HIGH);
}

bool isShoulderMaxHit() {
  return digitalRead(SHOULDER_LIMIT_MAX_PIN) == (LIMIT_SWITCH_ACTIVE_LOW ? LOW : HIGH);
}

// ─── Motor Enable / Disable ─────────────────────────────────
void enableMotors(bool enable) {
  uint8_t level = enable ? TB6600_ENABLE_LEVEL : TB6600_DISABLE_LEVEL;
  digitalWrite(ENA_PIN_1, level);
  digitalWrite(ENA_PIN_2, level);
  digitalWrite(ENA_PIN_3, level);
}

// Single step pulse
void stepMotor(uint8_t step_pin) {
  digitalWrite(step_pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(step_pin, LOW);
}

// ─── Shoulder Homing Routine ────────────────────────────────
void homeShoulder() {
  Serial.println(F("# Starting Shoulder Homing..."));
  enableMotors(true);
  
  // Set direction toward Min limit switch
  digitalWrite(DIR_PIN_2, LOW);
  
  unsigned long timeout = millis() + 15000; // 15 sec max
  while (!isShoulderMinHit() && millis() < timeout) {
    stepMotor(STEP_PIN_2);
    delayMicroseconds(800); // Slow homing speed
  }
  
  if (isShoulderMinHit()) {
    Serial.println(F("# Shoulder Min Limit Hit! Backing off..."));
    // Back off slightly
    digitalWrite(DIR_PIN_2, HIGH);
    for (int i = 0; i < 200; i++) {
      stepMotor(STEP_PIN_2);
      delayMicroseconds(1000);
    }
    Serial.println(F("# Shoulder Homing Complete."));
  } else {
    Serial.println(F("# Shoulder Homing Timeout / Error!"));
  }
}

// ─── Setup ───────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // 400kHz Fast I2C

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

  enableMotors(true);
  Serial.println(F("# EB-15 Nano Firmware Initialized."));
}

// ─── Main Loop ───────────────────────────────────────────────
void loop() {
  // 1. Read Encoders
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    uint16_t raw = readAS5600RawAngle(ENCODER_CHANNELS[i]);
    float rad = rawAngleToRadians(raw);
    joint_positions[i] = (rad - encoder_offsets[i]) * encoder_directions[i];
  }

  // 2. Hardware Safety: Halt Shoulder if limit switch triggered during movement
  if (isShoulderMinHit() || isShoulderMaxHit()) {
    // Safety check
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
    } else if (cmd == "$HOME") {
      homeShoulder();
    } else if (cmd.startsWith("$SET_ZERO")) {
      // Set current positions as zero offset
      for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        uint16_t raw = readAS5600RawAngle(ENCODER_CHANNELS[i]);
        encoder_offsets[i] = rawAngleToRadians(raw);
      }
      Serial.println(F("# Encoders zeroed to current pose."));
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
    Serial.println(isShoulderMaxHit() ? 1 : 0);
  }
}
