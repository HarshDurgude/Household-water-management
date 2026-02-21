#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include <ESP32Servo.h>
#include <WebServer.h>

/**
 * ---------------------------------------------------------------------------------------
 * PROJECT: Household Water Management System (Server Unit)
 * DESCRIPTION:
 * This unit acts as the central controller. It communicates with an Indicator unit 
 * via ESP-NOW to monitor water tank levels and control a motor (servo) for pump switching.
 * It also hosts a local HTTP server for status monitoring and manual control via a mobile app.
 * ---------------------------------------------------------------------------------------
 */

// Global HTTP Server instance on port 80
WebServer serverHTTP(80);

// Forward declaration
void sendCode(const uint8_t *mac, uint8_t code);

// ---------------------------------------------------------------------------------------
// GLOBAL CONSTANTS & PINS
// ---------------------------------------------------------------------------------------
#define DEBUG 1             // Set to 0 to disable all serial logs
#define BOOT_PIN 0          // ESP32 BOOT button (GPIO0)
#define LED_PIN 2           // Onboard LED
#define SERVO_PIN 15        // Servo motor signal pin
#define TOUCH_TEST_PIN 4    // Capacitive touch: Trigger ultrasonic query (Indicator)
#define TOUCH_TANK_PIN 13   // Capacitive touch: Query Tank status
#define TOUCH_MOTOR_PIN 32  // Capacitive touch: Local servo motor test

// ---------------------------------------------------------------------------------------
// LOGGING MACROS
// ---------------------------------------------------------------------------------------
#if DEBUG
  #define DBG(x) Serial.println(x)
  #define DBG2(x,y) { Serial.print(x); Serial.println(y); }
#else
  #define DBG(x)
  #define DBG2(x,y)
#endif

// ---------------------------------------------------------------------------------------
// DATA STRUCTURES (ESP-NOW)
// ---------------------------------------------------------------------------------------
/**
 * TankMessage: Used for general command/event codes
 * Codes used: 1=Tank Full, 3=Test Ping, 4=Range Test, 10=Status Query, 
 * 50=START Session, 51=READY, 52=STOP Session, 53=ACK STOP, 60=Ultra Query
 */
typedef struct {
  uint8_t tankId; 
} TankMessage;

/**
 * UltraData: Carries numeric sensor data from Indicator
 */
typedef struct {
  uint8_t type;    // 61 = distance_cm, 63 = percentage
  int16_t value;   // Scaled by 100
} UltraData;

/**
 * ControlMessage: Used for policy/handshake synchronization
 */
typedef struct {
  uint8_t type;    // 70 = policy
  uint8_t value;   // 0 = CONTINUOUS mode, 1 = NORMAL mode
} ControlMessage;

// ---------------------------------------------------------------------------------------
// GLOBAL STATE & SETTINGS
// ---------------------------------------------------------------------------------------
// Indicator MAC Address
uint8_t indicatorAddress[] = { 0x38, 0x18, 0x2B, 0x8B, 0x3E, 0x5C };

Servo myServo;

// Timing & Timeout Constants
const unsigned long HTTP_ULTRA_TIMEOUT_MS = 3000;
const unsigned long LED_COOLDOWN_MS = 2000;
const unsigned long START_PHASE_DURATION_MS = 20000;
const unsigned long START_SEND_PERIOD_MS = 200;
const unsigned long STOP_SEND_PERIOD_MS = 500;
const int MAX_STOP_SENDS = 20;
#define RANGE_LONG_PRESS_MS 2500

// Sensor & App Exposure State
float lastDistanceCm = -1;
float lastTankPercent = -1;
bool lastSessionActive = false;
bool lastAutoCutoff = false;
bool pumpIsOn = false; 

// HTTP Handshake State
volatile bool httpWaitingForUltrasonic = false;
unsigned long httpRequestStartMs = 0;
float httpDistanceCm = -1;
float httpTankPercent = -1;

// Session & Handshake State
bool waitingForReady = true;  
bool sessionActive = false;   
bool pumpOff = false;        
bool stopAcked = false;       

// Policy & Boot Window Logic
bool autoCutoffEnabled = false;
bool bootDecisionDone = false;
bool bootPressDetected = false;
unsigned long bootDetectStart = 0;
unsigned long bootPressStart = 0;
bool bootWindowLogged = false;

// Touch Calibration
uint16_t baseTest4, baseTank1_13, baseMotor32;
const uint16_t TOUCH_MARGIN = 20;
bool prevTouchTest = false, prevTouchT1 = false, prevTouchMotor = false;

// UI & Feedback
unsigned long ledCooldownUntil = 0;
bool bootPressConsumed = false;

// Internal Handshake Counters
unsigned long startPhaseStartMs = 0;
unsigned long lastStartSendMs = 0;
unsigned long lastStopSendMs = 0;
int stopSendCount = 0;

// Servo Animation State
volatile bool pendingPumpOn = false;
volatile bool pendingPumpOff = false;
static unsigned long servoActionAt = 0;
static int servoStep = 0;

// ---------------------------------------------------------------------------------------
// HELPER FUNCTIONS
// ---------------------------------------------------------------------------------------

/**
 * Utility to execute a block of code once based on a flag
 */
bool logOnce(bool &flag) {
  if (flag) return false;
  flag = true;
  return true;
}

/**
 * Sets CORS headers for HTTP responses (allows mobile app to connect)
 */
void sendCORS() {
  serverHTTP.sendHeader("Access-Control-Allow-Origin", "*");
  serverHTTP.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  serverHTTP.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

/**
 * Triggers a non-blocking Pump ON sequence
 */
void pumpOnFunc() {
  DBG("SERVER: Pump ON requested");
  pendingPumpOn = true;
}

/**
 * Triggers a non-blocking Pump OFF sequence
 */
void pumpOffFunc(const char* reason = "Manual") {
  DBG2("SERVER: Pump OFF requested, reason = ", reason);
  pendingPumpOff = true;
}

/**
 * Visual feedback via onboard LED
 */
void blinkPattern(int count, int onMs = 200, int offMs = 200) {
  if (millis() < ledCooldownUntil) return;
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(onMs);
    digitalWrite(LED_PIN, LOW);
    delay(offMs);
  }
  ledCooldownUntil = millis() + LED_COOLDOWN_MS;
}

/**
 * Ensures ESP-NOW peer is registered
 */
void ensurePeer(const uint8_t *mac) {
  if (!esp_now_is_peer_exist(mac)) {
    esp_now_peer_info_t p = {};
    memcpy(p.peer_addr, mac, 6);
    p.channel = 1; 
    p.encrypt = false;
    esp_now_add_peer(&p);
  }
}

/**
 * Sends a command code via ESP-NOW
 */
void sendCode(const uint8_t *mac, uint8_t code) {
  ensurePeer(mac);
  TankMessage msg;
  msg.tankId = code;
  esp_now_send(mac, (uint8_t*)&msg, sizeof(msg));
}

/**
 * Calibrates touch pins
 */
uint16_t calibrateTouchPin(uint8_t pin) {
  delay(300);
  uint32_t sum = 0;
  for (int i = 0; i < 20; i++) {
    sum += touchRead(pin);
    delay(30);
  }
  return sum / 20;
}

/**
 * Checks if a pin is touched
 */
bool isTouch(uint8_t pin, uint16_t baseline) {
  return (touchRead(pin) + TOUCH_MARGIN < baseline);
}

// ---------------------------------------------------------------------------------------
// HTTP ROUTE HANDLERS
// ---------------------------------------------------------------------------------------

/**
 * API: /status - Returns current system state as JSON
 */
void handleStatus() {
  serverHTTP.sendHeader("Access-Control-Allow-Origin", "*");
  serverHTTP.sendHeader("Access-Control-Allow-Methods", "GET, OPTIONS");
  serverHTTP.sendHeader("Access-Control-Allow-Headers", "Content-Type");

  // Trigger live measurement from Indicator
  httpDistanceCm = -1;
  httpTankPercent = -1;
  httpWaitingForUltrasonic = true;
  httpRequestStartMs = millis();
  sendCode(indicatorAddress, 60);

  // Wait for response (non-blocking yield)
  while (httpWaitingForUltrasonic) {
    if (millis() - httpRequestStartMs > HTTP_ULTRA_TIMEOUT_MS) break;
    delay(10);
    yield();
  }

  if (httpWaitingForUltrasonic) {
    DBG("HTTP: ultrasonic timeout");
    serverHTTP.send(503, "application/json", "{\"error\":\"indicator_not_responding\"}");
    return;
  }

  // Build response
  String json = "{";
  json += "\"tankPercent\":" + String(httpTankPercent, 1) + ",";
  json += "\"distanceCm\":" + String(httpDistanceCm, 1) + ",";
  json += "\"sessionActive\":" + String(lastSessionActive ? "true" : "false") + ",";
  json += "\"autoCutoff\":" + String(lastAutoCutoff ? "true" : "false");
  json += ",\"pumpOn\":" + String(pumpIsOn ? "true" : "false");
  json += "}";

  serverHTTP.send(200, "application/json", json);
}

// ---------------------------------------------------------------------------------------
// ESP-NOW CALLBACKS
// ---------------------------------------------------------------------------------------

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {}

void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  // 1. Numeric Sensor Data
  if (len == sizeof(UltraData)) {
    UltraData ultra;
    memcpy(&ultra, data, sizeof(ultra));
    if (ultra.type == 61) {
      httpDistanceCm = ultra.value / 100.0;
      lastDistanceCm = httpDistanceCm;
    } else if (ultra.type == 63) {
      httpTankPercent = ultra.value / 100.0;
      lastTankPercent = httpTankPercent;
    }
    if (httpWaitingForUltrasonic && httpDistanceCm >= 0 && httpTankPercent >= 0) {
      httpWaitingForUltrasonic = false;
    }
    return;
  }

  // 2. Command Messages
  if (len != sizeof(TankMessage)) return;
  TankMessage msg;
  memcpy(&msg, data, sizeof(msg));
  const uint8_t *sender = mac;

  switch (msg.tankId) {
    case 1: // Tank FULL from Indicator probes
      DBG("SERVER: Tank FULL notification received");
      blinkPattern(2);
      pumpOffFunc("Tank Full (Probes)");
      pumpOff = true;
      sendCode(sender, 1); // ACK
      break;

    case 3: // Range test from Indicator
      blinkPattern(3, 120, 120);
      sendCode(sender, 3); 
      break;

    case 4: // Range test ACK
      blinkPattern(3, 120, 120);
      break;

    case 11: // Query response: FULL
      blinkPattern(2);
      break;
    case 12: // Query response: NOT FULL
      blinkPattern(1);
      break;

    case 51: // Indicator READY
      static bool readyLogged = false;
      if (logOnce(readyLogged)) DBG("SERVER: Indicator READY → Session Active");
      waitingForReady = false;
      sessionActive = true;
      pumpOff = false;
      stopAcked = false;
      stopSendCount = 0;
      lastSessionActive = true;
      lastAutoCutoff = autoCutoffEnabled;
      blinkPattern(3, 120, 120);

      // Sync policy
      ControlMessage policy;
      policy.type = 70;
      policy.value = autoCutoffEnabled ? 1 : 0;
      esp_now_send(sender, (uint8_t*)&policy, sizeof(policy));
      break;

    case 53: // STOP Acknowledged
      static bool stopAckLogged = false;
      if (logOnce(stopAckLogged)) DBG("SERVER: STOP ACK received");
      stopAcked = true;
      sessionActive = false;
      lastSessionActive = false;
      waitingForReady = true;
      blinkPattern(4);
      break;

    default:
      break;
  }
}

// ---------------------------------------------------------------------------------------
// ARDUINO SETUP & LOOP
// ---------------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(300);
  DBG("SYSTEM BOOT");

  // 1. WiFi & AP Configuration
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("TankServer", "12345678", 1); // AP on Channel 1
  delay(100);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  // Static IP for consistency
  IPAddress local_IP(192, 168, 4, 10);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  IPAddress dns(8, 8, 8, 8);
  if (!WiFi.config(local_IP, gateway, subnet, dns)) DBG("STA Static IP Config Failed");

  WiFi.begin("Tank-Network", "12345678");
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  DBG("\nWiFi Connected. IP: " + WiFi.localIP().toString());

  // 2. HTTP Routes
  serverHTTP.on("/status", handleStatus);
  serverHTTP.on("/pump/on", HTTP_POST, []() {
    sendCORS(); pumpOnFunc();
    serverHTTP.send(200, "application/json", "{\"ok\":true,\"pumpOn\":true}");
  });
  serverHTTP.on("/pump/off", HTTP_POST, []() {
    sendCORS(); pumpOffFunc("Manual");
    serverHTTP.send(200, "application/json", "{\"ok\":true,\"pumpOn\":false}");
  });
  serverHTTP.onNotFound([]() {
    if (serverHTTP.method() == HTTP_OPTIONS) { sendCORS(); serverHTTP.send(204); }
    else serverHTTP.send(404, "text/plain", "Not found");
  });
  serverHTTP.begin();

  // 3. Hardware & Logic Init
  pinMode(BOOT_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  myServo.attach(SERVO_PIN, 500, 2400);
  delay(800); 
  pumpOffFunc("Boot"); // Initial safe state

  baseTest4 = calibrateTouchPin(TOUCH_TEST_PIN);
  baseTank1_13 = calibrateTouchPin(TOUCH_TANK_PIN);
  baseMotor32 = calibrateTouchPin(TOUCH_MOTOR_PIN);

  esp_now_init();
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  bootDetectStart = millis();
  startPhaseStartMs = millis();
}

void loop() {
  serverHTTP.handleClient();
  unsigned long now = millis();

  // --- BOOT WINDOW: POLICY DECISION ---
  if (!bootDecisionDone) {
    if (!bootWindowLogged) { DBG("BOOT: Decision window active"); bootWindowLogged = true; }
    bool bootPressed = (digitalRead(BOOT_PIN) == LOW);
    if (!bootPressDetected && bootPressed && (now - bootDetectStart <= 5000)) {
      bootPressDetected = true; bootPressStart = now;
    }
    if (bootPressDetected && bootPressed && (now - bootPressStart >= 3000)) autoCutoffEnabled = true;
    if ((!bootPressDetected && now - bootDetectStart > 5000) || (bootPressDetected && !bootPressed)) {
      bootDecisionDone = true;
      DBG2("BOOT: autoCutoffEnabled = ", autoCutoffEnabled);
      if (autoCutoffEnabled) blinkPattern(5);
    }
    if (!bootDecisionDone) return;
  }

  // --- HANDSHAKE: START SIGNALLING ---
  if (waitingForReady && !sessionActive) {
    if (now - startPhaseStartMs < START_PHASE_DURATION_MS) {
      if (now - lastStartSendMs > START_SEND_PERIOD_MS) {
        sendCode(indicatorAddress, 50);
        lastStartSendMs = now;
      }
    }
  }

  // --- SESSION: STOP SIGNALLING ---
  if (sessionActive && pumpOff && !stopAcked) {
    if (stopSendCount < MAX_STOP_SENDS && now - lastStopSendMs > STOP_SEND_PERIOD_MS) {
      sendCode(indicatorAddress, 52);
      lastStopSendMs = now;
      stopSendCount++;
    }
  }

  // --- UI: CAPACITIVE TOUCHES ---
  if (isTouch(TOUCH_TEST_PIN, baseTest4) && !prevTouchTest) { sendCode(indicatorAddress, 60); delay(200); }
  prevTouchTest = isTouch(TOUCH_TEST_PIN, baseTest4);

  if (isTouch(TOUCH_TANK_PIN, baseTank1_13) && !prevTouchT1) { sendCode(indicatorAddress, 10); delay(200); }
  prevTouchT1 = isTouch(TOUCH_TANK_PIN, baseTank1_13);

  if (isTouch(TOUCH_MOTOR_PIN, baseMotor32) && !prevTouchMotor) { pumpOffFunc("GPIO Test"); }
  prevTouchMotor = isTouch(TOUCH_MOTOR_PIN, baseMotor32);

  // --- UI: BOOT LONG PRESS ---
  bool bootPressed = (digitalRead(BOOT_PIN) == LOW);
  if (bootPressed) {
    if (bootPressStart == 0 && !bootPressConsumed) bootPressStart = now;
    if (!bootPressConsumed && (now - bootPressStart > RANGE_LONG_PRESS_MS)) {
      sendCode(indicatorAddress, 4); bootPressConsumed = true;
    }
  } else { bootPressStart = 0; bootPressConsumed = false; }

  // --- SERVO ANIMATION: PUMP ON ---
  if (pendingPumpOn) {
    if (servoStep == 0) { myServo.write(0); servoActionAt = now; servoStep = 1; }
    else if (servoStep == 1 && now - servoActionAt >= 500) {
      myServo.write(40); pendingPumpOn = false; servoStep = 0; pumpIsOn = true;
    }
  }

  // --- SERVO ANIMATION: PUMP OFF ---
  if (pendingPumpOff) {
    if (servoStep == 0) { myServo.write(75); servoActionAt = now; servoStep = 1; }
    else if (servoStep == 1 && now - servoActionAt >= 500) {
      myServo.write(40); pendingPumpOff = false; servoStep = 0; pumpIsOn = false;
    }
  }

  delay(100); 
}
