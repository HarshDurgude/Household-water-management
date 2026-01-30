#include <Arduino.h>

#define TRIG_PIN 18
#define ECHO_PIN 19

// ===== USER VARIABLE =====
// Measure your tank height in CM and put here
float TANK_HEIGHT_CM = 60.0;

// Filtering
float filteredDistance = 0;
float alpha = 0.2;   // 0.1–0.3 good range

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.println("Ultrasonic Water Level Test");
}

long getDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout

  if (duration == 0) return -1; // timeout / no echo

  long distance = duration * 0.034 / 2;
  return distance;
}

void loop() {
  long rawDistance = getDistanceCM();

  if (rawDistance > 0) {
    // Exponential smoothing filter
    filteredDistance = alpha * rawDistance +
                       (1 - alpha) * filteredDistance;

    float waterHeight = TANK_HEIGHT_CM - filteredDistance;
    if (waterHeight < 0) waterHeight = 0;
    if (waterHeight > TANK_HEIGHT_CM) waterHeight = TANK_HEIGHT_CM;

    float percent = (waterHeight / TANK_HEIGHT_CM) * 100.0;

    Serial.print("Raw Dist(cm): ");
    Serial.print(rawDistance);
    Serial.print(" | Filtered(cm): ");
    Serial.print(filteredDistance);
    Serial.print(" | Water Height(cm): ");
    Serial.print(waterHeight);
    Serial.print(" | Level(%): ");
    Serial.println(percent);
  } else {
    Serial.println("No Echo / Out of Range");
  }

  delay(500);
}