#include <WiFi.h>

const char* ssid     = "Tank-Network";     // 🔴 change this
const char* password = "12345678"; // 🔴 change this

void setup() {
  Serial.begin(115200);
  delay(500);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected!");
  Serial.print("Server ESP32 IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // nothing needed
}
