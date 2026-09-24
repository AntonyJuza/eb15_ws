/**
 * EB-15 Hardware Diagnostic & I2C Scanner Tool
 * 
 * Use this sketch first to verify:
 *  1. TCA9548A I2C Multiplexer detection
 *  2. AS5600 Magnetic Encoder detection on Ch 0, 1, 2
 *  3. Live raw angles (0..4095) and degrees (0..360)
 *  4. Shoulder Limit Switches (Min / Max)
 */

#include <Wire.h>

#define SHOULDER_LIMIT_MIN_PIN   3
#define SHOULDER_LIMIT_MAX_PIN   2
#define ELBOW_LIMIT_MIN_PIN      A1
#define ELBOW_LIMIT_MAX_PIN      A2
#define LIMIT_SWITCH_ACTIVE_LOW  true

#define TCA9548A_ADDR            0x70
#define AS5600_ADDR              0x36
#define AS5600_RAW_ANGLE_REG     0x0C

void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

uint16_t readAS5600(uint8_t ch) {
  tcaSelect(ch);
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_RAW_ANGLE_REG);
  if (Wire.endTransmission(false) != 0) return 9999; // Error
  
  Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2);
  if (Wire.available() >= 2) {
    uint16_t high = Wire.read();
    uint16_t low = Wire.read();
    return ((high & 0x0F) << 8) | low;
  }
  return 9999;
}

void scanI2CBus() {
  Serial.println(F("\n--- Scanning Main I2C Bus ---"));
  byte count = 0;
  for (byte i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("Found I2C device at 0x"));
      if (i < 16) Serial.print(F("0"));
      Serial.println(i, HEX);
      count++;
    }
  }
  if (count == 0) Serial.println(F("No I2C devices found on main bus!"));

  Serial.println(F("\n--- Scanning TCA9548A Channels ---"));
  for (uint8_t ch = 0; ch < 3; ch++) {
    tcaSelect(ch);
    Wire.beginTransmission(AS5600_ADDR);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("Channel "));
      Serial.print(ch);
      Serial.println(F(": AS5600 FOUND (0x36) [OK]"));
    } else {
      Serial.print(F("Channel "));
      Serial.print(ch);
      Serial.println(F(": AS5600 NOT DETECTED! [CHECK WIRING]"));
    }
  }
  Serial.println(F("---------------------------------\n"));
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  Wire.begin();

  pinMode(SHOULDER_LIMIT_MIN_PIN, INPUT_PULLUP);
  pinMode(SHOULDER_LIMIT_MAX_PIN, INPUT_PULLUP);
  pinMode(ELBOW_LIMIT_MIN_PIN, INPUT_PULLUP);
  pinMode(ELBOW_LIMIT_MAX_PIN, INPUT_PULLUP);

  Serial.println(F("========================================"));
  Serial.println(F("       EB-15 HARDWARE DIAGNOSTIC        "));
  Serial.println(F("========================================"));
  
  scanI2CBus();
  delay(1000);
}

void loop() {
  // Read Encoders
  uint16_t raw0 = readAS5600(0);
  uint16_t raw1 = readAS5600(1);
  uint16_t raw2 = readAS5600(2);

  float deg0 = (raw0 != 9999) ? (raw0 * 360.0 / 4096.0) : -1.0;
  float deg1 = (raw1 != 9999) ? (raw1 * 360.0 / 4096.0) : -1.0;
  float deg2 = (raw2 != 9999) ? (raw2 * 360.0 / 4096.0) : -1.0;

  // Read Limit Switches (Active LOW with internal pull-up)
  bool sh_min = (digitalRead(SHOULDER_LIMIT_MIN_PIN) == (LIMIT_SWITCH_ACTIVE_LOW ? LOW : HIGH));
  bool sh_max = (digitalRead(SHOULDER_LIMIT_MAX_PIN) == (LIMIT_SWITCH_ACTIVE_LOW ? LOW : HIGH));
  bool el_min = (digitalRead(ELBOW_LIMIT_MIN_PIN) == (LIMIT_SWITCH_ACTIVE_LOW ? LOW : HIGH));
  bool el_max = (digitalRead(ELBOW_LIMIT_MAX_PIN) == (LIMIT_SWITCH_ACTIVE_LOW ? LOW : HIGH));

  Serial.print(F("J1: "));
  if (raw0 != 9999) { Serial.print(deg0, 1); Serial.print(F("°")); }
  else Serial.print(F("ERR"));

  Serial.print(F(" | J2: "));
  if (raw1 != 9999) { Serial.print(deg1, 1); Serial.print(F("°")); }
  else Serial.print(F("ERR"));

  Serial.print(F(" | J3: "));
  if (raw2 != 9999) { Serial.print(deg2, 1); Serial.print(F("°")); }
  else Serial.print(F("ERR"));

  Serial.print(F(" | Shldr: [Min="));
  Serial.print(sh_min ? F("HIT") : F("ok"));
  Serial.print(F(", Max="));
  Serial.print(sh_max ? F("HIT") : F("ok"));

  Serial.print(F("] | Elbow: [Min(A1)="));
  Serial.print(el_min ? F("HIT") : F("ok"));
  Serial.print(F(", Max(A2)="));
  Serial.print(el_max ? F("HIT") : F("ok"));
  Serial.println(F("]"));

  delay(200);
}
