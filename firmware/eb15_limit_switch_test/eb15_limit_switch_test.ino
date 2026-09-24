/**
 * EB-15 Elbow & Shoulder Limit Switch Interactive Test Tool
 * ========================================================
 * 
 * Hardware Wiring:
 *  - Elbow MIN Limit Switch (-2.700 rad):  Pin A1 to Switch (NO or COM) -> other terminal to GND
 *  - Elbow MAX Limit Switch (+2.700 rad):  Pin A2 to Switch (NO or COM) -> other terminal to GND
 *  - Shoulder MIN Limit Switch (0°):       Pin D3 to Switch -> other terminal to GND
 *  - Shoulder MAX Limit Switch (180°):     Pin D2 to Switch -> other terminal to GND
 * 
 * How it works:
 *  - Internal pull-ups are enabled (INPUT_PULLUP).
 *  - When switch is OPEN (unpressed): Pin reads HIGH (1 / 5V).
 *  - When switch is PRESSED:          Pin connects to GND and reads LOW (0 / 0V).
 * 
 * Output:
 *  - Instant event messages whenever you press or release a switch.
 *  - Live periodic status update every 500 ms.
 * 
 * Serial Monitor Settings:
 *  - Baud rate: 115200
 */

// ─── Pin Definitions ─────────────────────────────────────────
#define ELBOW_LIMIT_MIN_PIN      A1  // Pin A1 (Elbow -2.700 rad / folded)
#define ELBOW_LIMIT_MAX_PIN      A2  // Pin A2 (Elbow +2.700 rad / extended)
#define SHOULDER_LIMIT_MIN_PIN   3   // Pin D3 (Shoulder 0° / Min)
#define SHOULDER_LIMIT_MAX_PIN   2   // Pin D2 (Shoulder 180° / Max)

// Active level: switch to GND pulls pin LOW when hit
#define LIMIT_ACTIVE_STATE       LOW

// State tracking for edge detection
bool last_el_min = false;
bool last_el_max = false;
bool last_sh_min = false;
bool last_sh_max = false;

unsigned long last_display_time = 0;
const unsigned long DISPLAY_INTERVAL_MS = 500;

// Helper to read active status
bool isTriggered(uint8_t pin) {
  return digitalRead(pin) == LIMIT_ACTIVE_STATE;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Wait for Serial USB on Nano

  // Configure switch pins with internal pull-ups
  pinMode(ELBOW_LIMIT_MIN_PIN,    INPUT_PULLUP);
  pinMode(ELBOW_LIMIT_MAX_PIN,    INPUT_PULLUP);
  pinMode(SHOULDER_LIMIT_MIN_PIN, INPUT_PULLUP);
  pinMode(SHOULDER_LIMIT_MAX_PIN, INPUT_PULLUP);

  // Initialize previous states
  last_el_min = isTriggered(ELBOW_LIMIT_MIN_PIN);
  last_el_max = isTriggered(ELBOW_LIMIT_MAX_PIN);
  last_sh_min = isTriggered(SHOULDER_LIMIT_MIN_PIN);
  last_sh_max = isTriggered(SHOULDER_LIMIT_MAX_PIN);

  Serial.println(F("\n========================================================"));
  Serial.println(F("       EB-15 LIMIT SWITCH HARDWARE TEST TOOL            "));
  Serial.println(F("========================================================"));
  Serial.println(F("Configured Pins (Active LOW with internal pull-up):"));
  Serial.println(F("  - Elbow MIN Limit (-2.700 rad): Pin A1"));
  Serial.println(F("  - Elbow MAX Limit (+2.700 rad): Pin A2"));
  Serial.println(F("  - Shoulder MIN Limit (0 deg):   Pin D3"));
  Serial.println(F("  - Shoulder MAX Limit (180 deg): Pin D2"));
  Serial.println(F("--------------------------------------------------------"));
  Serial.println(F("Press and release the limit switches to test them now!"));
  Serial.println(F("========================================================\n"));
}

void loop() {
  unsigned long now = millis();

  // Read current states
  bool el_min = isTriggered(ELBOW_LIMIT_MIN_PIN);
  bool el_max = isTriggered(ELBOW_LIMIT_MAX_PIN);
  bool sh_min = isTriggered(SHOULDER_LIMIT_MIN_PIN);
  bool sh_max = isTriggered(SHOULDER_LIMIT_MAX_PIN);

  // ── Edge Detection: Elbow MIN (Pin A1) ─────────────────────
  if (el_min != last_el_min) {
    delay(10); // Simple debounce
    el_min = isTriggered(ELBOW_LIMIT_MIN_PIN);
    if (el_min != last_el_min) {
      last_el_min = el_min;
      if (el_min) {
        Serial.println(F(">>> [TRIGGERED] ELBOW MIN Switch (Pin A1) PRESSED! (Pin is LOW)"));
      } else {
        Serial.println(F("<<< [RELEASED]  ELBOW MIN Switch (Pin A1) Released. (Pin is HIGH)"));
      }
    }
  }

  // ── Edge Detection: Elbow MAX (Pin A2) ─────────────────────
  if (el_max != last_el_max) {
    delay(10); // Simple debounce
    el_max = isTriggered(ELBOW_LIMIT_MAX_PIN);
    if (el_max != last_el_max) {
      last_el_max = el_max;
      if (el_max) {
        Serial.println(F(">>> [TRIGGERED] ELBOW MAX Switch (Pin A2) PRESSED! (Pin is LOW)"));
      } else {
        Serial.println(F("<<< [RELEASED]  ELBOW MAX Switch (Pin A2) Released. (Pin is HIGH)"));
      }
    }
  }

  // ── Edge Detection: Shoulder MIN (Pin D3) ──────────────────
  if (sh_min != last_sh_min) {
    delay(10);
    sh_min = isTriggered(SHOULDER_LIMIT_MIN_PIN);
    if (sh_min != last_sh_min) {
      last_sh_min = sh_min;
      if (sh_min) {
        Serial.println(F(">>> [TRIGGERED] SHOULDER MIN Switch (Pin D3) PRESSED!"));
      } else {
        Serial.println(F("<<< [RELEASED]  SHOULDER MIN Switch (Pin D3) Released."));
      }
    }
  }

  // ── Edge Detection: Shoulder MAX (Pin D2) ──────────────────
  if (sh_max != last_sh_max) {
    delay(10);
    sh_max = isTriggered(SHOULDER_LIMIT_MAX_PIN);
    if (sh_max != last_sh_max) {
      last_sh_max = sh_max;
      if (sh_max) {
        Serial.println(F(">>> [TRIGGERED] SHOULDER MAX Switch (Pin D2) PRESSED!"));
      } else {
        Serial.println(F("<<< [RELEASED]  SHOULDER MAX Switch (Pin D2) Released."));
      }
    }
  }

  // ── Periodic Live Status Print ─────────────────────────────
  if (now - last_display_time >= DISPLAY_INTERVAL_MS) {
    last_display_time = now;

    Serial.print(F("[LIVE] "));
    Serial.print(F("Elbow MIN (A1): "));
    Serial.print(el_min ? F("TRIGGERED (0)") : F("OPEN (1)"));

    Serial.print(F(" | Elbow MAX (A2): "));
    Serial.print(el_max ? F("TRIGGERED (0)") : F("OPEN (1)"));

    Serial.print(F(" | Shldr MIN (D3): "));
    Serial.print(sh_min ? F("TRIGGERED (0)") : F("OPEN (1)"));

    Serial.print(F(" | Shldr MAX (D2): "));
    Serial.println(sh_max ? F("TRIGGERED (0)") : F("OPEN (1)"));
  }
}
