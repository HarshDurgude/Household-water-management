#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

const char* ssid = "ESP32-Control";
const char* password = "12345678";
Servo myServo;

WebServer server(80);

// --- HTML PAGE ---
void handleRoot() {
  String html =
    "<html><body style='font-family: Arial;'>"
    "<h2>Motor Control</h2>"
    "<a href='/motor1'><button style='padding:15px;font-size:18px;'>Motor 1</button></a>"
    "<br><br>"
    "<a href='/motor2'><button style='padding:15px;font-size:18px;'>Motor 2</button></a>"
    "</body></html>";

  server.send(200, "text/html", html);
}

// --- MOTOR 1 BUTTON ---
void handleMotor1() {

  // Your motor logic
  myServo.write(80);
  delay(1000);
  myServo.write(40);

  // After doing the action, just show main page again
  handleRoot();
}

// --- MOTOR 2 BUTTON ---
void handleMotor2() {

  // Your motor logic
  myServo.write(0);
  delay(1000);
  myServo.write(40);

  // After doing the action, just show main page again
  handleRoot();
}

void setup() {
  Serial.begin(115200);

  myServo.attach(4, 500, 2400);

  // WIFI HOTSPOT MODE
  WiFi.softAP(ssid, password);
  Serial.println("WiFi Started");
  Serial.print("Connect to: ");
  Serial.println(ssid);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  // ROUTES
  server.on("/", handleRoot);
  server.on("/motor1", handleMotor1);
  server.on("/motor2", handleMotor2);

  server.begin();
}

void loop() {
  server.handleClient();
}

// #include <ESP32Servo.h>

// Servo myServo;
// int servoPin = 4;



// void setup() {
//   myServo.attach(servoPin, 500, 2400);

//   // Keep servo at current angle
//   myServo.write(40);
//   // delay(1000);

//   // // Move to +40 degrees
//   // int newAngle = currentAngle + 40;
//   // if (newAngle > 180) newAngle = 180; // safety limit

//   // myServo.write(newAngle);
// }

// void loop() {
//   // EMPTY — does nothing
// }
