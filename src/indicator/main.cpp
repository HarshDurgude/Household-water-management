
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_sleep.h>
#include <esp_wifi.h>

bool firstBootWindow = true;

// add near the top with other globals
const uint8_t ESP_NOW_CHANNEL = 1; // fixed channel for AP and indicator

volatile bool callback_wants_blink = false;   // set by callback, handled in loop()
volatile uint8_t callback_blink_count = 0;

volatile bool callback_send_ack = false;      // for ACKs that must be sent outside callback
volatile uint8_t ack_value_to_send = 0;

// for debug: store last-received MAC
uint8_t last_sender_mac[6] = {0,0,0,0,0,0};

#define DEBUG 1   // set to 0 to disable all serial logs

#if DEBUG
  #define DBG(x) Serial.println(x)
  #define DBG2(x,y) { Serial.print(x); Serial.println(y); }
#else
  #define DBG(x)
  #define DBG2(x,y)
#endif



// ---- Ultrasonic sensor pins ----
#define ULTRA_TRIG_PIN 18
#define ULTRA_ECHO_PIN 19

// ---- Tank geometry ----
#define TANK_HEIGHT_CM 41.5


// ---- BOOT button range test (indicator side) ----
#define BOOT_PIN 0
#define RANGE_LONG_PRESS_MS 2500

bool bootPressConsumed = false;
unsigned long bootPressStart = 0;


unsigned long ledCooldownUntil = 0;
const unsigned long LED_COOLDOWN_MS = 2000;


// test comment

// ----- Pins -----
const int TANK1_PIN = 23;
const int TANK2_PIN = 22;
const int LED_PIN   = 2;
const int TOUCH_PIN = 4;    // capacitive touch for range-test

// ----- RTC state: are we in pump mode? -----
RTC_DATA_ATTR bool rtcInPumpMode = false;

// ----- Tank detection state -----
bool prevTank1Full = false;
bool prevTank2Full = false;

// ----- Pump-mode flags -----
bool tank1Sent = false;  // did we already send tank1 full/timeout?
bool tank2Sent = false;  // did we already send tank2 full/timeout?
bool stopRequested = false;

// Timeouts (ms)
const unsigned long TANK1_TIMEOUT_MS = 17UL * 60UL * 1000UL;  // 17 minutes
const unsigned long TANK2_TIMEOUT_MS = 27UL * 60UL * 1000UL;  // 27 minutes
unsigned long pumpModeStartMs = 0;

// Idle listening window
const unsigned long IDLE_LISTEN_MS        = 2000;             // ~1s listen
const unsigned long IDLE_WAKE_INTERVAL_US = 5ULL * 1000000ULL; // wake every 5s

// Idle state
bool startLatched = false;
bool idleInitDone   = false;
bool startReceived  = false;
unsigned long idleStartMs = 0;

// Server MAC (room ESP32) – change if needed
uint8_t serverAddress[] = { 0x00, 0x4B, 0x12, 0x2F, 0xFA, 0x08 };



// ---- Ultrasonic numeric data ----
typedef struct {
  uint8_t type;    // 61 = distance_cm, 63 = percent
  int16_t value;   // scaled by 100
} UltraData;

// Message struct
typedef struct {
  uint8_t tankId;
} TankMessage;

// ---- Touch calibration ----
uint16_t touchBaseline = 0;
const uint16_t TOUCH_MARGIN = 20;
bool prevTouchActive = false;

// ---------------------------------
// HELPERS
// ---------------------------------


void blinkLed(int count, int onMs = 200, int offMs = 200) {
  if (millis() < ledCooldownUntil) return;

  for (int i = 0; i < count; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(onMs);
    digitalWrite(LED_PIN, LOW);
    delay(offMs);
  }

  ledCooldownUntil = millis() + LED_COOLDOWN_MS;
}

void sendCode(uint8_t code) {
  DBG2("INDICATOR TX → code = ", code);
  TankMessage msg;
  msg.tankId = code;
  esp_now_send(serverAddress, (uint8_t*)&msg, sizeof(msg));
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // DBG("ESP-NOW: send callback");
}


// Ultra-sensitive tank detection (your version)
bool isTankFull(int pin) {
  const int checks  = 50;
  const int delayMs = 10;
  int lowCount = 0;

  for (int i = 0; i < checks; i++) {
    if (digitalRead(pin) == LOW) {
      lowCount++;
    }
    delay(delayMs);
  }

  return (lowCount >= 1);
}

float measureUltrasonicCm() {
  // Ensure clean trigger
  digitalWrite(ULTRA_TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Trigger pulse
  digitalWrite(ULTRA_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRA_TRIG_PIN, LOW);

  // Read echo (timeout ~30 ms)
  long duration = pulseIn(ULTRA_ECHO_PIN, HIGH, 30000);

  if (duration == 0) {
    return -1.0;  // No echo / out of range
  }

  // Speed of sound: 343 m/s → 0.0343 cm/us
  float distanceCm = (duration * 0.0343) / 2.0;
  return distanceCm;
}

void computeWaterLevel(float distanceCm,
                       float &waterHeight,
                       float &percent) {
  if (distanceCm <= 0) {
    waterHeight = -1;
    percent = -1;
    return;
  }

  waterHeight = TANK_HEIGHT_CM - distanceCm;

  if (waterHeight < 0) waterHeight = 0;
  if (waterHeight > TANK_HEIGHT_CM) waterHeight = TANK_HEIGHT_CM;

  percent = (waterHeight / TANK_HEIGHT_CM) * 100.0;
}

void calibrateTouch() {
  delay(500);
  uint32_t sum = 0;
  const int samples = 20;
  for (int i = 0; i < samples; i++) {
    sum += touchRead(TOUCH_PIN);
    delay(50);
  }
  touchBaseline = sum / samples;
}

bool isTouchActive() {
  uint16_t val = touchRead(TOUCH_PIN);
  return (val + TOUCH_MARGIN < touchBaseline);
}

// ---------------------------------
// RECEIVE CALLBACK
// ---------------------------------
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {

  // // 1️⃣ ControlMessage (policy)
  // if (len == sizeof(ControlMessage)) {
  //   ControlMessage ctrl;
  //   memcpy(&ctrl, data, sizeof(ctrl));

  //   if (ctrl.type == 70) {
  //     allowIdleExit = (ctrl.value == 1);
  //     DBG2("INDICATOR: allowIdleExit = ", allowIdleExit);

  //     // ACK back
  //     ControlMessage ack = {71, ctrl.value};
  //     esp_now_send(serverAddress, (uint8_t*)&ack, sizeof(ack));
  //   }
  //   return;
  // }

  // 2️⃣ TankMessage
  if (len != sizeof(TankMessage)) return;

  TankMessage msg;
  memcpy(&msg, data, sizeof(msg));

  // DBG2("INDICATOR RX ← tankId = ", msg.tankId);

  switch (msg.tankId) {
    // ---- ACKs for tank events ----
    case 1:
      // ACK Tank1 full
      blinkLed(1);
      break;
    case 2:
      // ACK Tank2 full
      blinkLed(2);
      break;

    // ---- ACK for indicator's own test ----
    case 3:
      blinkLed(3, 120, 120);
      break;

    // ---- Server range test ----
    case 4:
      // Server is testing range: blink 3 and ACK 4 back
      blinkLed(3, 120, 120);
      sendCode(4);
      break;

    // ---- Server tank1 status query ----
    case 10: {
      bool full = isTankFull(TANK1_PIN);
      if (full) sendCode(11);
      else      sendCode(12);
      break;
    }

    // ---- Server tank2 status query ----
    case 20: {
      bool full = isTankFull(TANK2_PIN);
      if (full) sendCode(21);
      else      sendCode(22);
      break;
    }

    // ---- Pump-session handshake: START / STOP ----
    case 50:  // START from server
      DBG("INDICATOR: START packet received");

      if (!rtcInPumpMode && !startReceived) {
        DBG("INDICATOR: START accepted");
        startReceived = true;
      } else {
        DBG("INDICATOR: START ignored");
      }
      break;


    case 52:  // STOP from server
      DBG("INDICATOR: STOP received");

      // Pump session end requested
      stopRequested = true;
      break;

    case 60: {  // Ultrasonic query
      DBG("INDICATOR: Ultrasonic query (60) → measuring");

      float dist = measureUltrasonicCm();
      float waterH = 0, percent = 0;
      computeWaterLevel(dist, waterH, percent);

      if (dist < 0) {
        DBG("INDICATOR: Ultrasonic out of range");
        break;
      }

      // ---- DEBUG (still useful) ----
      DBG2("INDICATOR: Distance (cm): ", dist);
      DBG2("INDICATOR: Level (%): ", percent);

      UltraData msg;

      // Send distance (cm)
      msg.type  = 61;
      msg.value = (int16_t)(dist * 100);
      esp_now_send(serverAddress, (uint8_t*)&msg, sizeof(msg));
      DBG("INDICATOR: Ultrasonic data sent to server");
      delay(40);

      // Send percentage (%)
      msg.type  = 63;
      msg.value = (int16_t)(percent * 100);
      esp_now_send(serverAddress, (uint8_t*)&msg, sizeof(msg));

      break;
    }

    default:
      break;
  }
}

// ---------------------------------
// SETUP
// ---------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  DBG("BOOT");

  pinMode(BOOT_PIN, INPUT_PULLUP);


  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(TANK1_PIN, INPUT_PULLUP);
  pinMode(TANK2_PIN, INPUT_PULLUP);

  pinMode(ULTRA_TRIG_PIN, OUTPUT);
  pinMode(ULTRA_ECHO_PIN, INPUT);
  digitalWrite(ULTRA_TRIG_PIN, LOW);

  WiFi.mode(WIFI_STA);
  // 🔴 FORCE SAME CHANNEL AS SERVER
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  esp_now_init();

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, serverAddress, 6);
  peerInfo.channel = 1;              // 🔴 IMPORTANT
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  calibrateTouch();

  // Initial states (used before we know mode)
  prevTank1Full = isTankFull(TANK1_PIN);
  prevTank2Full = isTankFull(TANK2_PIN);

  idleInitDone    = false;
  startReceived   = false;
  prevTouchActive = false;
  stopRequested   = false;
}

// ---------------------------------
// IDLE MODE HANDLER (deep sleep cycle)
// ---------------------------------
void handleIdleMode() {
  if (!idleInitDone) {
    idleStartMs   = millis();
    idleInitDone  = true;
    if (!startLatched) {
      startReceived = false;
    }

    DBG("INDICATOR: Idle listen window started");
  }

  // 🔴 IMPORTANT: give WiFi/ESP-NOW time to run
  delay(10);
  yield();

  // If START arrived
  if (startReceived && !startLatched) {
    DBG("INDICATOR: START detected during idle");

    // 🔴 CRITICAL: commit to pump mode IMMEDIATELY
    rtcInPumpMode = true;
    startLatched  = true;
    idleInitDone  = false;

    pumpModeStartMs = millis();

    bool initialTank1Full = isTankFull(TANK1_PIN);
    bool initialTank2Full = isTankFull(TANK2_PIN);

    sendCode(51);               // READY
    blinkLed(3, 120, 120);

    prevTank1Full = initialTank1Full;
    prevTank2Full = initialTank2Full;

    if (initialTank1Full) {
      sendCode(1);
      tank1Sent = true;
    }
    if (initialTank2Full) {
      sendCode(2);
      tank2Sent = true;
    }

    return;   // 🔴 IMPORTANT: exit idle handler immediately
  }


  // End of listen window
  if (millis() - idleStartMs > IDLE_LISTEN_MS) {

    if (firstBootWindow) {
      DBG("INDICATOR: First boot window expired, staying awake");
      firstBootWindow = false;
      idleInitDone = false;   // restart idle listening
      return;
    }

    DBG("INDICATOR: Idle timeout → sleeping");
    esp_sleep_enable_timer_wakeup(IDLE_WAKE_INTERVAL_US);
    esp_deep_sleep_start();
  }
}

// ---------------------------------
// PUMP MODE HANDLER
// ---------------------------------
void handlePumpMode() {
  unsigned long now = millis();

  // ---- Tank sensing ----
  bool tank1Full = isTankFull(TANK1_PIN);
  bool tank2Full = isTankFull(TANK2_PIN);

  bool tank1JustFull = (tank1Full && !prevTank1Full) && !tank1Sent;
  bool tank2JustFull = (tank2Full && !prevTank2Full) && !tank2Sent;

  if (tank1JustFull) {
    sendCode(1);      // Tank1 full
    tank1Sent = true;
  }

  if (tank2JustFull) {
    sendCode(2);      // Tank2 full
    tank2Sent = true;
  }

  // ---- Safety timeouts ----
  if (!tank1Sent && (now - pumpModeStartMs > TANK1_TIMEOUT_MS)) {
    // Safety: treat as tank1 full
    sendCode(1);
    tank1Sent = true;
  }

  if (!tank2Sent && (now - pumpModeStartMs > TANK2_TIMEOUT_MS)) {
    // Safety: treat as tank2 full
    sendCode(2);
    tank2Sent = true;
  }

  prevTank1Full = tank1Full;
  prevTank2Full = tank2Full;

  // ---- Indicator's own capacitive range test (GPIO4) ----
//   bool touchNow = isTouchActive();
//   if (touchNow && !prevTouchActive) {
//     sendCode(3);      // indicator test
//     delay(200);
//   }
//   prevTouchActive = touchNow;

  // ---- STOP handling ----
  if (stopRequested) {
    stopRequested = false;

    DBG("INDICATOR: ENTERING IDLE MODE");

    blinkLed(4);
    sendCode(53);
    delay(200);

    // Reset latches
    startLatched = false;
    startReceived = false;

    rtcInPumpMode = false;
    idleInitDone  = false;

    esp_sleep_enable_timer_wakeup(IDLE_WAKE_INTERVAL_US);
    esp_deep_sleep_start();
  }

  bool bootPressed = (digitalRead(BOOT_PIN) == LOW);
  // unsigned long now = millis();

  if (bootPressed) {
    if (bootPressStart == 0 && !bootPressConsumed) {
      bootPressStart = now;
    }

    if (!bootPressConsumed && (now - bootPressStart > RANGE_LONG_PRESS_MS)) {
      DBG("INDICATOR: BOOT long press → RANGE TEST to SERVER");
      sendCode(3);   // indicator-originated range test
      bootPressConsumed = true;
    }
  } else {
    bootPressStart = 0;
    bootPressConsumed = false;
  }


  delay(300);
}

// ---------------------------------
// MAIN LOOP
// ---------------------------------
void loop() {
  if (!rtcInPumpMode) {
    // IDLE mode: wake briefly, listen for START, then sleep
    handleIdleMode();
  } else {
    // PUMP mode: full features
    handlePumpMode();
  }
}