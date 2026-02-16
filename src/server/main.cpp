#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include <ESP32Servo.h>
#include <WebServer.h>

WebServer serverHTTP(80);

void sendCode(const uint8_t *mac, uint8_t code);

bool bootWindowLogged = false;

bool logOnce(bool &flag) {
  if (flag) return false;
  flag = true;
  return true;
}


volatile bool pendingPumpOn  = false;
volatile bool pendingPumpOff = false;


bool pumpIsOn = false;   // 🔴 single source of truth



// ---- HTTP live ultrasonic handling ----
volatile bool httpWaitingForUltrasonic = false;
unsigned long httpRequestStartMs = 0;

float httpDistanceCm = -1;
float httpTankPercent = -1;

const unsigned long HTTP_ULTRA_TIMEOUT_MS = 3000;

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
#define SERVO_PIN        15    // Servo signal
#define TOUCH_TEST_PIN   4    // capacitive: server→indicator range test
#define TOUCH_TANK_PIN 13    // capacitive: query tank
#define TOUCH_MOTOR_PIN 32    // capacitive: local servo test

// ---- Indicator MAC (your indicator board) ----
uint8_t indicatorAddress[] = { 
  0x38, 0x18, 0x2B, 0x8B, 0x3E, 0x5C 
};


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
uint16_t baseTest4, baseTank1_13, baseMotor32;
const uint16_t TOUCH_MARGIN = 20;

bool prevTouchTest   = false;
bool prevTouchT1     = false;
bool prevTouchMotor  = false;

// ---- Session state ----
bool waitingForReady = true;   // waiting for READY after STARTs
bool sessionActive   = false;  // in pump mode
bool pumpOff        = false;
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

void sendCORS() {
  serverHTTP.sendHeader("Access-Control-Allow-Origin", "*");
  serverHTTP.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  serverHTTP.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}


void pumpOnFunc() {
  DBG("SERVER: Pump ON requested");
  pendingPumpOn = true;
}

void pumpOffFunc(const char* reason = "Manual") {
  DBG2("SERVER: Pump OFF requested, reason = ", reason);
  pendingPumpOff = true;
}




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

// void handleCorsOptions() {
//   serverHTTP.sendHeader("Access-Control-Allow-Origin", "*");
//   serverHTTP.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
//   serverHTTP.sendHeader("Access-Control-Allow-Headers", "Content-Type");
//   serverHTTP.send(204);
// }


void handleStatus() {

  // ✅ CORS HEADERS (THIS IS THE FIX)
  serverHTTP.sendHeader("Access-Control-Allow-Origin", "*");
  serverHTTP.sendHeader("Access-Control-Allow-Methods", "GET, OPTIONS");
  serverHTTP.sendHeader("Access-Control-Allow-Headers", "Content-Type");

  // DBG("HTTP: /status requested → triggering live ultrasonic");

  // Reset live values
  httpDistanceCm = -1;
  httpTankPercent = -1;
  httpWaitingForUltrasonic = true;
  httpRequestStartMs = millis();

  // Send ultrasonic query to indicator
  sendCode(indicatorAddress, 60);

  // Wait (blocking, but max 3s)
  while (httpWaitingForUltrasonic) {
    if (millis() - httpRequestStartMs > HTTP_ULTRA_TIMEOUT_MS) {
      break;
    }
    delay(10);
    yield();
  }

  // Timeout → indicator not responding
  if (httpWaitingForUltrasonic) {
    DBG("HTTP: ultrasonic timeout → indicator not responding");

    serverHTTP.send(
      503,
      "application/json",
      "{\"error\":\"indicator_not_responding\"}"
    );
    return;
  }

  // Success → send live data
  // DBG("HTTP: ultrasonic response received → sending JSON");

  Serial.print("HTTP: Tank ");
  Serial.print(httpTankPercent, 1);
  Serial.print("% | Distance ");
  Serial.print(httpDistanceCm, 1);
  Serial.println(" cm");


  String json = "{";
  json += "\"tankPercent\":" + String(httpTankPercent, 1) + ",";
  json += "\"distanceCm\":" + String(httpDistanceCm, 1) + ",";
  json += "\"sessionActive\":" + String(lastSessionActive ? "true" : "false") + ",";
  json += "\"autoCutoff\":" + String(lastAutoCutoff ? "true" : "false");
  json += ",\"pumpOn\":" + String(pumpIsOn ? "true" : "false");
  json += "}";

  DBG2("HTTP STATUS: pumpIsOn = ", pumpIsOn ? "ON" : "OFF");

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
  if (code == 60) {
    // DBG("SERVER: Requesting ultrasonic measurement from indicator");
  } 
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
  // DBG("ESP-NOW: send callback");
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
      httpDistanceCm = ultra.value / 100.0;
      lastDistanceCm = httpDistanceCm;   // keep last-known in sync

      // DBG2("SERVER RX: Ultrasonic distance cm = ", httpDistanceCm);
    } 
    else if (ultra.type == 63) {
      httpTankPercent = ultra.value / 100.0;
      lastTankPercent = httpTankPercent; // keep last-known in sync

      // DBG2("SERVER RX: Tank level % = ", httpTankPercent);
    }

    // If HTTP is waiting and both values are ready → mark done
    if (httpWaitingForUltrasonic &&
        httpDistanceCm >= 0 &&
        httpTankPercent >= 0) {

      httpWaitingForUltrasonic = false;
      // DBG("SERVER: Ultrasonic response COMPLETE → HTTP can reply");
    }

    return;
  }
  if (len != sizeof(TankMessage)) return;
  TankMessage msg;

  // if (msg.tankId == 51) {
  //   DBG("SERVER: Indicator is awake → pump session established");

  //   sessionActive = true;
  //   waitingForReady = false;
  // }

  memcpy(&msg, data, sizeof(msg));


  const uint8_t *sender = mac;

  switch (msg.tankId) {
    case 1:  // Tank FULL
      DBG("SERVER: Tank FULL → turning pump OFF");
      blinkPattern(2);
      pumpOffFunc("Tank Full (Probes)");
      // DBG("SERVER: Pump Off servo run (Tank Full By Probes) ");
      pumpOff = true;
      sendCode(sender, 1); // ACK
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


    // ----- Pump session handshake -----
    case 51:
      static bool readyLogged = false;
      if (logOnce(readyLogged)) {
        DBG("SERVER: READY received → session established");
      }


      waitingForReady    = false;
      sessionActive      = true;

      pumpOff       = false;
      stopAcked     = false;
      stopSendCount = 0;

      lastSessionActive = true;
      lastAutoCutoff    = autoCutoffEnabled;

      blinkPattern(3,120,120);

      ControlMessage policy;
      policy.type  = 70;
      policy.value = autoCutoffEnabled ? 1 : 0;
      esp_now_send(sender, (uint8_t*)&policy, sizeof(policy));
      break;



    case 53:
      static bool stopAckLogged = false;
      if (logOnce(stopAckLogged)) {
        DBG("SERVER: STOP acknowledged by indicator");
      }


      stopAcked = true;

      sessionActive = false;
      lastSessionActive = false;
      waitingForReady = true;

      blinkPattern(4);
      break;



    // case 61:
    //   DBG("SERVER: Ultrasonic distance received (see indicator serial)");
    //   break;

    // case 62:
    //   DBG("SERVER: Water height received (see indicator serial)");
    //   break;

    // case 63:
    //   DBG("SERVER: Water percentage received (see indicator serial)");
    //   break;
    default:
      // ignore unknown
      break;
  }
}

void setup() {

  Serial.begin(115200);
  delay(300);
  DBG("BOOT - Uploaded via OTA");


  // 2️⃣ THEN start WebServer
  serverHTTP.on("/status", handleStatus);

  serverHTTP.on("/pump/on", HTTP_POST, []() {
    DBG("HTTP: Pump ON request received");
    sendCORS();

    pumpOnFunc();

    serverHTTP.send(
      200,
      "application/json",
      "{\"ok\":true,\"pumpOn\":true}"
    );
  });

  serverHTTP.on("/pump/off", HTTP_POST, []() {
    DBG("HTTP: Pump OFF request received");
    sendCORS();

    pumpOffFunc("Manual");

    serverHTTP.send(
      200,
      "application/json",
      "{\"ok\":true,\"pumpOn\":false}"
    );
  });





  WiFi.mode(WIFI_AP_STA);

  // 🔴 FORCE CHANNEL 1 BY CREATING AP
  WiFi.softAP("TankServer", "12345678", 1);
  delay(100);

  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  // Optional: connect to router AFTER AP is up

  // ---------- FIXED STATIC IP FOR ESP32 SERVER ----------
  IPAddress local_IP(192, 168, 4, 10);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  IPAddress dns(8, 8, 8, 8);

  // Must be called BEFORE WiFi.begin()
  if (!WiFi.config(local_IP, gateway, subnet, dns)) {
    Serial.println("STA Failed to configure static IP");
  }


  WiFi.begin("Tank-Network", "12345678");


  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }



  Serial.println();
  Serial.print("Connected. IP: ");
  Serial.println(WiFi.localIP());

  // serverHTTP.on("/pump/on", HTTP_OPTIONS, handleCorsOptions);
  // serverHTTP.on("/pump/off", HTTP_OPTIONS, handleCorsOptions);
  // serverHTTP.on("/status", HTTP_OPTIONS, handleCorsOptions);


  // 🔴 REQUIRED FOR CORS PREFLIGHT
  serverHTTP.onNotFound([]() {
    if (serverHTTP.method() == HTTP_OPTIONS) {
      sendCORS();
      serverHTTP.send(204); // No Content
    } else {
      serverHTTP.send(404, "text/plain", "Not found");
    }
  });

  serverHTTP.begin();
  DBG("HTTP server started");




  // 3️⃣ THEN everything else (ESP-NOW, GPIO, logic)
  pinMode(BOOT_PIN, INPUT_PULLUP);


  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  myServo.attach(SERVO_PIN, 500, 2400);
  delay(800);                 // 🔴 CRITICAL: let servo initialize
  pumpOffFunc("Boot");   // or "Tank Full", "Boot", etc.


  DBG("SERVER: Pump Off on Boot (servo initialized)");



  // Calibrate capacitive touch pins
  baseTest4    = calibrateTouchPin(TOUCH_TEST_PIN);
  baseTank1_13 = calibrateTouchPin(TOUCH_TANK_PIN);
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
  pumpOff = false;
  stopAcked = false;
  stopSendCount = 0;

  startPhaseStartMs = millis();
  lastStartSendMs   = 0;
}

void loop() {

  serverHTTP.handleClient();

  // -------- BOOT BUTTON POLICY DECISION WINDOW --------
  if (!bootDecisionDone) {

    if (!bootWindowLogged) {
      DBG("BOOT: decision window started (0–5s)");
      bootWindowLogged = true;
    }

    // DBG("BOOT: decision window active (0–5 seconds)");

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
        DBG2("BOOT: autoCutoffEnabled = ", autoCutoffEnabled);
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

    // Log ONCE when START phase begins
    static bool startPhaseLogged = false;
    if (!startPhaseLogged) {
      DBG("SERVER: Waiting for indicator to wake (START phase)");
      startPhaseLogged = true;
    }

    if (now - startPhaseStartMs < START_PHASE_DURATION_MS) {
      if (now - lastStartSendMs > START_SEND_PERIOD_MS) {

        static bool startPhasePrinted = false;
        if (logOnce(startPhasePrinted)) {
          DBG("SERVER: START phase → waking indicator");
        }

        sendCode(indicatorAddress, 50);

        lastStartSendMs = now;
      }
    }
  }

  // ---- STOP sending logic ----
  if (sessionActive && pumpOff && !stopAcked) {
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
    DBG("SERVER: Ultrasonic query via GPIO4");
    sendCode(indicatorAddress, 60);
    delay(200);
  }
  prevTouchTest = touchUltra;

  // 2) Query Tank1 (GPIO13)
  bool touchT1 = isTouch(TOUCH_TANK_PIN, baseTank1_13);
  if (touchT1 && !prevTouchT1) {
    sendCode(indicatorAddress, 10);      // ask tank1 status
    delay(200);
  }
  prevTouchT1 = touchT1;


  // 4) Local servo test (GPIO32)
  bool touchMotor = isTouch(TOUCH_MOTOR_PIN, baseMotor32);
  if (touchMotor && !prevTouchMotor) {
    pumpOffFunc("GPIO Test");

    DBG("SERVER: GPIO32 Local servo Test ");
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



  static unsigned long servoActionAt = 0;
  static int servoStep = 0;

  if (pendingPumpOn) {
    if (servoStep == 0) {
      myServo.write(0);
      servoActionAt = millis();
      servoStep = 1;
    } else if (servoStep == 1 && millis() - servoActionAt >= 500) {
      myServo.write(40);
      pendingPumpOn = false;
      servoStep = 0;

      pumpIsOn = true;   // ✅ ADD THIS LINE

      DBG("SERVER: Pump ON sequence complete");
    }
  }


  if (pendingPumpOff) {
    if (servoStep == 0) {
      myServo.write(75);
      servoActionAt = millis();
      servoStep = 1;
    } else if (servoStep == 1 && millis() - servoActionAt >= 500) {
      myServo.write(40);
      pendingPumpOff = false;
      servoStep = 0;

      pumpIsOn = false;  // ✅ ADD THIS LINE

      DBG("SERVER: Pump OFF sequence complete");
    }
  }




  delay(100);
}

