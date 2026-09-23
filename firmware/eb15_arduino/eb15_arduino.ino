/**
 * EB-15 Robotic Arm - Arduino Nano Low-Level Firmware
 * 
 * Hardware:
 *  - 3x NEMA 17 Steppers with TB6600 Drivers
 *  - 3x AS5600 Magnetic Encoders via TCA9548A I2C Multiplexer
 *  - 2x Shoulder (Joint 2) Limit Switches (Min / Max) on D3, D2
 *  - 2x Elbow (Joint 3) Limit Switches (Min: -2.700 rad / Max: +2.700 rad) on A1, A2
 *  - 2x Servos (Wrist Roll on D13, Gripper on A0)
 * 
 * Communication:
 *  - USB Serial @ 115200 baud
 *  - Telemetry:  $STATE,<j1_rad>,<j2_rad>,<j3_rad>,<sh_min>,<sh_max>,<el_min>,<el_max>\n
 *  - Commands:   $CMD,<j1_rad>,<j2_rad>,<j3_rad>\n
 *                $HOME\n
 *                $HOME_SHOULDER\n
 *                $HOME_ELBOW\n
 *                $SET_ZERO\n
 */

#include <Wire.h>

// ─── Pin Definitions ─────────────────────────────────────────

// Shoulder (Joint 2) Limit Switches (Active LOW with internal pull-up)
#define SHOULDER_LIMIT_MIN_PIN   3   // Pin D3 (Home / 0° end)
#define SHOULDER_LIMIT_MAX_PIN   2   // Pin D2 (End / 180° end)

// Elbow (Joint 3) Limit Switches (Active LOW with internal pull-up)
#define ELBOW_LIMIT_MIN_PIN      A1  // Pin A1 (Min limit: -2.700 rad)
#define ELBOW_LIMIT_MAX_PIN      A2  // Pin A2 (Max limit: +2.700 rad)
#define LIMIT_SWITCH_ACTIVE_LOW  true

// Servos (Wrist Roll & Gripper)
#define WRIST_SERVO_PIN          13  // Pin D13 (Wrist Roll Servo)
#define GRIPPER_SERVO_PIN        A0  // Pin A0 / D14 (EBG-20 Gripper Servo)

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

// Stepper Homing Directions (swap LOW/HIGH if motor moves opposite way)
#define SHOULDER_DIR_TOWARD_MIN  LOW
#define SHOULDER_DIR_TOWARD_MAX  HIGH
#define ELBOW_DIR_TOWARD_MIN     LOW
#define ELBOW_DIR_TOWARD_MAX     HIGH

// Joint 3 Physical Calibration Limits (radians)
const float J3_LIMIT_MIN_RAD = -2.700f;
const float J3_LIMIT_MAX_RAD =  2.700f;

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

// ─── Limit Switch Checks ────────────────────────────────────
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

// ─── Shoulder (Joint 2) Homing Routine ──────────────────────
void homeShoulder() {
  Serial.println(F("# Starting Shoulder Homing..."));
  enableMotors(true);
  
  // Set direction toward Min limit switch
  digitalWrite(DIR_PIN_2, SHOULDER_DIR_TOWARD_MIN);
  
  unsigned long timeout = millis() + 15000; // 15 sec max
  while (!isShoulderMinHit() && millis() < timeout) {
    stepMotor(STEP_PIN_2);
    delayMicroseconds(800); // Slow homing speed
  }
  
  if (isShoulderMinHit()) {
    Serial.println(F("# Shoulder Min Limit Hit! Backing off..."));
    // Back off slightly
    digitalWrite(DIR_PIN_2, SHOULDER_DIR_TOWARD_MAX);
    for (int i = 0; i < 200; i++) {
      stepMotor(STEP_PIN_2);
      delayMicroseconds(1000);
    }
    Serial.println(F("# Shoulder Homing Complete."));
  } else {
    Serial.println(F("# Shoulder Homing Timeout / Error!"));
  }
}

// ─── Elbow (Joint 3) Homing Routine ─────────────────────────
void homeElbow() {
  Serial.println(F("# Starting Elbow (Joint 3) Homing..."));
  enableMotors(true);
  
  // Set direction toward Min limit switch (-2.700 rad)
  digitalWrite(DIR_PIN_3, ELBOW_DIR_TOWARD_MIN);
  
  unsigned long timeout = millis() + 20000; // 20 sec max
  while (!isElbowMinHit() && millis() < timeout) {
    stepMotor(STEP_PIN_3);
    delayMicroseconds(800); // Slow homing speed
  }
  
  if (isElbowMinHit()) {
    Serial.println(F("# Elbow Min Limit Hit (-2.700 rad)! Backing off..."));
    // Back off slightly
    digitalWrite(DIR_PIN_3, ELBOW_DIR_TOWARD_MAX);
    for (int i = 0; i < 200; i++) {
      stepMotor(STEP_PIN_3);
      delayMicroseconds(1000);
    }
    
    // Read encoder at backed-off MIN limit and calibrate offset
    delay(200);
    uint16_t raw = readAS5600RawAngle(ENCODER_CHANNELS[2]);
    float current_rad = rawAngleToRadians(raw);
    // Calibration equation: joint_position = (current_rad - offset) * dir = -2.700
    // => offset = current_rad - (-2.700 * dir)
    encoder_offsets[2] = current_rad - (J3_LIMIT_MIN_RAD * encoder_directions[2]);
    
    Serial.print(F("# Elbow Homing Complete. Calibrated to "));
    Serial.print(J3_LIMIT_MIN_RAD, 3);
    Serial.print(F(" rad. Offset: "));
    Serial.println(encoder_offsets[2], 4);
  } else {
    Serial.println(F("# Elbow Homing Timeout / Error! Check switch on pin A1."));
  }
}

// ─── Home All Joints ─────────────────────────────────────────
void homeAll() {
  homeShoulder();
  homeElbow();
}

// ─── Setup ───────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // 400kHz Fast I2C

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

  enableMotors(true);
  Serial.println(F("# EB-15 Nano Firmware Initialized."));
  Serial.println(F("# Joint 3 limit switches active on A1 (MIN -2.700) and A2 (MAX +2.700)."));
}

// ─── Main Loop ───────────────────────────────────────────────
void loop() {
  // 1. Read Encoders
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    uint16_t raw = readAS5600RawAngle(ENCODER_CHANNELS[i]);
    float rad = rawAngleToRadians(raw);
    joint_positions[i] = (rad - encoder_offsets[i]) * encoder_directions[i];
  }

  // 2. Hardware Safety: Check limit switches during movement
  if (isShoulderMinHit() || isShoulderMaxHit() || isElbowMinHit() || isElbowMaxHit()) {
    // Hardware safety stop can be triggered if moving toward active switch
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
      homeAll();
    } else if (cmd == "$HOME_SHOULDER") {
      homeShoulder();
    } else if (cmd == "$HOME_ELBOW") {
      homeElbow();
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
    Serial.print(isShoulderMaxHit() ? 1 : 0);
    Serial.print(F(","));
    Serial.print(isElbowMinHit() ? 1 : 0);
    Serial.print(F(","));
    Serial.println(isElbowMaxHit() ? 1 : 0);
  }
}
