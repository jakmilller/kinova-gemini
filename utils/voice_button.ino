/*
 * Push-to-talk voice button for the Kinova-Gemini robot.
 *
 * Streams the button state over USB serial at 115200 baud:
 *   "1"  while the button is HELD
 *   "0"  while the button is RELEASED
 * emitted continuously (~every 50 ms) so the host always knows the current state.
 * arduino_trigger_node.py converts this level into press/release edges -> /voice_trigger.
 *
 * This is the deliberate replacement for the old firmware that sent a single pulse per
 * press (which forced a press-talk-press-again workflow). Now: hold to talk, release to send.
 *
 * WIRING: connect the button between BUTTON_PIN and GND. INPUT_PULLUP is used, so the pin
 * reads LOW when pressed and HIGH when released. Set BUTTON_PIN to match your wiring.
 */

const int BUTTON_PIN = 2;          // <-- set this to the pin your button is wired to
const unsigned long DEBOUNCE_MS = 25;
const unsigned long REPORT_INTERVAL_MS = 50;

int stableState = HIGH;            // debounced reading (HIGH = released, pull-up)
int lastReading = HIGH;
unsigned long lastChangeTime = 0;
unsigned long lastReportTime = 0;

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);     // lights while the button is held (upload sanity check)
  Serial.begin(115200);

  // Startup self-test: blink the on-board LED 3x so you can confirm the sketch is running
  // and LED_BUILTIN is the LED you're watching, independent of the button/pin wiring.
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(150);
    digitalWrite(LED_BUILTIN, LOW);
    delay(150);
  }
}

void loop() {
  int reading = digitalRead(BUTTON_PIN);

  // Debounce: only accept a reading once it has held steady past the debounce window.
  if (reading != lastReading) {
    lastChangeTime = millis();
    lastReading = reading;
  }
  if ((millis() - lastChangeTime) > DEBOUNCE_MS) {
    stableState = reading;
  }

  // On-board LED mirrors the button: lit while held. LOW = pressed with INPUT_PULLUP.
  digitalWrite(LED_BUILTIN, stableState == LOW ? HIGH : LOW);

  // Emit the current (debounced) state continuously. LOW = pressed with INPUT_PULLUP.
  if ((millis() - lastReportTime) >= REPORT_INTERVAL_MS) {
    lastReportTime = millis();
    Serial.println(stableState == LOW ? 1 : 0);
  }
}
