#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include <ESP32Servo.h>
#include <WebServer.h>
WebServer serverHTTP(80);


#define DEBUG 1   // set to 0 to disable all serial logs

#if DEBUG
  #define DBG(x) Serial.println(x)
  #define DBG2(x,y) { Serial.print(x); Serial.println(y); }
#else
  #define DBG(x)
  #define DBG2(x,y)
#endif

// ---- State exposed to app ----
float lastDistanceCm = -1;
float lastTankPercent = -1;
bool  lastSessionActive = false;
bool  lastAutoCutoff = false;

// ---- Pump policy decision (server) ----
bool autoCutoffEnabled = false;

unsigned long bootDetectStart = 0;
unsigned long bootPressStart  = 0;
bool bootPressDetected = false;
bool bootDecisionDone  = false;

// ---- BOOT button range test ----
#define BOOT_PIN 0
#define RANGE_LONG_PRESS_MS 2500

bool bootPressConsumed = false;

// LED cooldown
unsigned long ledCooldownUntil = 0;
const unsigned long LED_COOLDOWN_MS = 2000;



#define LED_PIN 2

// ---- Pins ----
#define SERVO_PIN        5    // Servo signal
#define TOUCH_TEST_PIN   4    // capacitive: server→indicator range test
#define TOUCH_TANK1_PIN 13    // capacitive: query tank1
#define TOUCH_TANK2_PIN 27    // capacitive: query tank2
#define TOUCH_MOTOR_PIN 32    // capacitive: local servo test

// ---- Indicator MAC (your indicator board) ----
// Replace if different:
uint8_t indicatorAddress[] = { 0x38, 0x18, 0x2B, 0x8A, 0x46, 0x08 };

typedef struct {
  uint8_t type;    // message type (70 = policy, 71 = ACK)
  uint8_t value;   // payload (0 or 1)
} ControlMessage;

// ---- Ultrasonic numeric data ----
typedef struct {
  uint8_t type;    // 61 = distance_cm, 63 = percent
  int16_t value;   // scaled by 100
} UltraData;

// ---- Message struct ----
typedef struct {
  uint8_t tankId;
} TankMessage;

Servo myServo;

// ---- Touch baselines ----
uint16_t baseTest4, baseTank1_13, baseTank2_27, baseMotor32;
const uint16_t TOUCH_MARGIN = 20;

bool prevTouchTest   = false;
bool prevTouchT1     = false;
bool prevTouchT2     = false;
bool prevTouchMotor  = false;

// ---- Session state ----
bool waitingForReady = true;   // waiting for READY after STARTs
bool sessionActive   = false;  // in pump mode
bool tank1Off        = false;
bool tank2Off        = false;
bool stopAcked       = false;

unsigned long startPhaseStartMs = 0;
unsigned long lastStartSendMs   = 0;

unsigned long lastStopSendMs    = 0;
int stopSendCount               = 0;
const int   MAX_STOP_SENDS      = 20;
const unsigned long STOP_SEND_PERIOD_MS  = 500;
const unsigned long START_PHASE_DURATION_MS = 20000;  // send START for 20s max
const unsigned long START_SEND_PERIOD_MS   = 200;     // every 200ms

// ---------------------------------
// HELPER FUNCTIONS
// ---------------------------------

void blinkPattern(int count, int onMs = 200, int offMs = 200) {
  // Prevent overlapping patterns
  if (millis() < ledCooldownUntil) return;

  for (int i = 0; i < count; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(onMs);
    digitalWrite(LED_PIN, LOW);
    delay(offMs);
  }

  // start cooldown
  ledCooldownUntil = millis() + LED_COOLDOWN_MS;
}

void handleStatus() {
  String json = "{";
  json += "\"tankPercent\":" + String(lastTankPercent, 1) + ",";
  json += "\"distanceCm\":" + String(lastDistanceCm, 1) + ",";
  json += "\"sessionActive\":" + String(lastSessionActive ? "true" : "false") + ",";
  json += "\"autoCutoff\":" + String(lastAutoCutoff ? "true" : "false");
  json += "}";

  serverHTTP.send(200, "application/json", json);
}

void ensurePeer(const uint8_t *mac) {
  if (!esp_now_is_peer_exist(mac)) {
    esp_now_peer_info_t p = {};
    memcpy(p.peer_addr, mac, 6);
    p.channel = 1;                     // 🔴 IMPORTANT
    p.encrypt = false;
    esp_now_add_peer(&p);
  }
}

void sendCode(const uint8_t *mac, uint8_t code) {
  ensurePeer(mac);
  DBG2("SERVER TX → code = ", code);
  TankMessage msg;
  msg.tankId = code;
  esp_now_send(mac, (uint8_t*)&msg, sizeof(msg));
}


uint16_t calibrateTouchPin(uint8_t pin) {
  delay(300);
  uint32_t sum = 0;
  for (int i = 0; i < 20; i++) {
    sum += touchRead(pin);
    delay(30);
  }
  return sum / 20;
}


bool isTouch(uint8_t pin, uint16_t baseline) {
  return (touchRead(pin) + TOUCH_MARGIN < baseline);
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  DBG("ESP-NOW: send callback");
}



// ---------------------------------
// RECEIVE CALLBACK
// ---------------------------------
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len){

  // ---- Ultrasonic numeric packets ----
  if (len == sizeof(UltraData)) {
    UltraData ultra;
    memcpy(&ultra, data, sizeof(ultra));

    if (ultra.type == 61) {
      lastDistanceCm = ultra.value / 100.0;
    } 
    else if (ultra.type == 63) {
      lastTankPercent = ultra.value / 100.0;
    }
    return;
  }

  if (len != sizeof(TankMessage)) return;

  DBG2("SERVER RX ← tankId = ", ((TankMessage*)data)->tankId);

  TankMessage msg;
  memcpy(&msg, data, sizeof(msg));


  const uint8_t *sender = mac;

  switch (msg.tankId) {
    case 1:
      blinkPattern(1);
      myServo.write(80); delay(1000); myServo.write(40);
      tank1Off = true;
      sendCode(sender, 1);
      break;


    case 2:   // Tank2 FULL or timeout
      blinkPattern(2);
      myServo.write(0); delay(1000); myServo.write(40);
      tank2Off = true;
      sendCode(sender, 2);   // ACK 2 back
      break;

    // ----- Indicator range test -----
    case 3:   // indicator test ping
      blinkPattern(3,120,120);
      sendCode(sender, 3);   // ACK 3
      break;

    case 4:   // ACK for server test
      blinkPattern(3,120,120);
      break;

    // ----- Query replies from indicator -----
    case 11:  // tank1 FULL
      blinkPattern(2);       // 2 blinks = FULL
      break;
    case 12:  // tank1 NOT full
      blinkPattern(1);       // 1 blink = NOT full
      break;

    case 21:  // tank2 FULL
      blinkPattern(2);
      break;
    case 22:  // tank2 NOT full
      blinkPattern(1);
      break;

    // ----- Pump session handshake -----
    case 51:  // READY from indicator
      waitingForReady     = false;
      sessionActive       = true;
      lastSessionActive   = true;
      lastAutoCutoff      = autoCutoffEnabled;

      blinkPattern(3,120,120);

      ControlMessage policy;
      policy.type  = 70;
      policy.value = autoCutoffEnabled ? 1 : 0;

      esp_now_send(sender, (uint8_t*)&policy, sizeof(policy));
      break;


    case 53:  // ACK_STOP from indicator
      DBG("SERVER: ACK_STOP received");
      stopAcked = true;
      blinkPattern(4);
      sessionActive = false;
      lastSessionActive = false;   // ✅ ADD THIS
      break;


    case 61:
      DBG("SERVER: Ultrasonic distance received (see indicator serial)");
      break;

    case 62:
      DBG("SERVER: Water height received (see indicator serial)");
      break;

    case 63:
      DBG("SERVER: Water percentage received (see indicator serial)");
      break;
    default:
      // ignore unknown
      break;
  }
}

void setup() {

  Serial.begin(115200);
  delay(300);
  DBG("BOOT");

  // 1️⃣ WiFi FIRST (this initializes lwIP / TCPIP task)
  WiFi.mode(WIFI_AP_STA);   // 🔴 KEEP STA ALIVE FOR ESP-NOW
  WiFi.softAP("TankOS", "12345678");
  
  // 🔴 FORCE CHANNEL (must match indicator)
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  // 2️⃣ THEN start WebServer
  serverHTTP.on("/status", handleStatus);
  serverHTTP.begin();
  DBG("HTTP server started");

  // 3️⃣ THEN everything else (ESP-NOW, GPIO, logic)
  pinMode(BOOT_PIN, INPUT_PULLUP);


  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  myServo.attach(SERVO_PIN, 500, 2400);
  myServo.write(40);

  // Calibrate capacitive touch pins
  baseTest4    = calibrateTouchPin(TOUCH_TEST_PIN);
  baseTank1_13 = calibrateTouchPin(TOUCH_TANK1_PIN);
  baseTank2_27 = calibrateTouchPin(TOUCH_TANK2_PIN);
  baseMotor32  = calibrateTouchPin(TOUCH_MOTOR_PIN);


  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  bootDetectStart = millis();

  esp_now_init();
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  // Session state
  waitingForReady = true;
  sessionActive   = false;
  tank1Off = tank2Off = false;
  stopAcked = false;
  stopSendCount = 0;

  startPhaseStartMs = millis();
  lastStartSendMs   = 0;
}

void loop() {

  serverHTTP.handleClient();

  // -------- BOOT BUTTON POLICY DECISION WINDOW --------
  if (!bootDecisionDone) {

    bool bootPressed = (digitalRead(BOOT_PIN) == LOW);

    // Detect start of press (must START within first 5s)
    if (!bootPressDetected && bootPressed && (millis() - bootDetectStart <= 5000)) {
      bootPressDetected = true;
      bootPressStart = millis();
    }

    // Measure continuous press (can extend beyond 5s)
    if (bootPressDetected && bootPressed) {
      if (millis() - bootPressStart >= 3000) {
        autoCutoffEnabled = true;
      }
    }

    // Finalize decision
    if (
        // Either no press ever started in 5s
        (!bootPressDetected && millis() - bootDetectStart > 5000)
        ||
        // Or press started, and either completed or released
        (bootPressDetected && !bootPressed)
      ) {

      bootDecisionDone = true;

      // Visual feedback ONLY ONCE
      if (autoCutoffEnabled) {
        blinkPattern(5);   // ← THIS is where blinkPattern(5) belongs
      }
    }

    // During decision window → do NOTHING else
    if (!bootDecisionDone) return;
  }
  unsigned long now = millis();

  // ---- START handshake phase (on boot) ----
  if (waitingForReady && !sessionActive) {
    if (now - startPhaseStartMs < START_PHASE_DURATION_MS) {
      if (now - lastStartSendMs > START_SEND_PERIOD_MS) {
        sendCode(indicatorAddress, 50);
        lastStartSendMs = now;
      }
    }
  }


  // ---- STOP sending logic ----
  if (sessionActive && tank1Off && tank2Off && !stopAcked) {
    if (stopSendCount < MAX_STOP_SENDS && now - lastStopSendMs > STOP_SEND_PERIOD_MS) {
      sendCode(indicatorAddress, 52);   // STOP
      lastStopSendMs = now;
      stopSendCount++;
    }
  }




  // ---- Capacitive touches on server ----

  // 1) ultrasonic querry (GPIO4)
  bool touchUltra = isTouch(TOUCH_TEST_PIN, baseTest4);
  if (touchUltra && !prevTouchTest) {
    DBG("SERVER: Ultrasonic query");
    sendCode(indicatorAddress, 60);
    delay(200);
  }
  prevTouchTest = touchUltra;

  // 2) Query Tank1 (GPIO13)
  bool touchT1 = isTouch(TOUCH_TANK1_PIN, baseTank1_13);
  if (touchT1 && !prevTouchT1) {
    sendCode(indicatorAddress, 10);      // ask tank1 status
    delay(200);
  }
  prevTouchT1 = touchT1;

  // 3) Query Tank2 (GPIO27)
  bool touchT2 = isTouch(TOUCH_TANK2_PIN, baseTank2_27);
  if (touchT2 && !prevTouchT2) {
    sendCode(indicatorAddress, 20);      // ask tank2 status
    delay(200);
  }
  prevTouchT2 = touchT2;

  // 4) Local servo test (GPIO32)
  bool touchMotor = isTouch(TOUCH_MOTOR_PIN, baseMotor32);
  if (touchMotor && !prevTouchMotor) {
    myServo.write(80); delay(500); myServo.write(40); delay(250);
    myServo.write(0);  delay(500); myServo.write(40);
  }
  prevTouchMotor = touchMotor;

  bool bootPressed = (digitalRead(BOOT_PIN) == LOW);
  // unsigned long now = millis();

  if (bootPressed) {
    if (bootPressStart == 0 && !bootPressConsumed) {
      bootPressStart = now;
    }

    if (!bootPressConsumed && (now - bootPressStart > RANGE_LONG_PRESS_MS)) {
      DBG("SERVER: BOOT long press → RANGE TEST");
      sendCode(indicatorAddress, 4);   // SAME code as before
      bootPressConsumed = true;
    }
  } else {
    // reset when released
    bootPressStart = 0;
    bootPressConsumed = false;
  }

  delay(100);
}

