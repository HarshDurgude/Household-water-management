const int TANK1_PIN = 23;   // Tank 1 probe GPIO
const int TANK2_PIN = 22;   // Tank 2 probe GPIO
const int LED_PIN   = 2;    // Built-in blue LED on many ESP32 dev boards

bool prevTank1Full = false;
bool prevTank2Full = false;

// ---------- Helpers ----------

// Read a tank input with simple debouncing
bool isTankFull(int pin) {
  const int checks  = 15;
  const int delayMs = 5;
  int countLow = 0;

  for (int i = 0; i < checks; i++) {
    if (digitalRead(pin) == LOW) {
      countLow++;
    }
    delay(delayMs);
  }

  // if most samples are LOW → treat as water contact
  return countLow > (checks * 0.7);
}

// Blink LED 'count' times
void blinkLed(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
  delay(500); // small pause after pattern
}

// ---------- Main ----------

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);

  pinMode(TANK1_PIN, INPUT_PULLUP);
  pinMode(TANK2_PIN, INPUT_PULLUP);

  // Initial states
  prevTank1Full = isTankFull(TANK1_PIN);
  prevTank2Full = isTankFull(TANK2_PIN);
}

void loop() {
  bool tank1Full = isTankFull(TANK1_PIN);
  bool tank2Full = isTankFull(TANK2_PIN);

  // Detect transitions from NOT FULL -> FULL
  bool tank1JustFull = (tank1Full && !prevTank1Full);
  bool tank2JustFull = (tank2Full && !prevTank2Full);

  if (tank1JustFull && tank2JustFull) {
    // both hit full "now"
    Serial.println("Tank 1 AND Tank 2 FULL");
    blinkLed(3);   // 3 blinks for both
  } else if (tank1JustFull) {
    Serial.println("Tank 1 FULL");
    blinkLed(1);   // 1 blink
  } else if (tank2JustFull) {
    Serial.println("Tank 2 FULL");
    blinkLed(2);   // 2 blinks
  }

  prevTank1Full = tank1Full;
  prevTank2Full = tank2Full;

  delay(300);  // how often to check
}
