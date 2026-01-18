#include <WiFi.h>
#include <esp_now.h>
#include <esp_sleep.h>
#include "esp_wifi.h"

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
const unsigned long IDLE_LISTEN_MS        = 1000;             // ~1s listen
const unsigned long IDLE_WAKE_INTERVAL_US = 5ULL * 1000000ULL; // wake every 5s

// Idle state
bool idleInitDone   = false;
bool startReceived  = false;
unsigned long idleStartMs = 0;

// Server MAC (room ESP32) – change if needed
uint8_t serverAddress[] = { 0x00, 0x4B, 0x12, 0x2F, 0xFA, 0x08 };

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
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(onMs);
    digitalWrite(LED_PIN, LOW);
    delay(offMs);
  }
  delay(250);
}

void sendCode(uint8_t code) {
  TankMessage msg;
  msg.tankId = code;
  esp_now_send(serverAddress, (uint8_t*)&msg, sizeof(msg));
}

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // not used
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
void onDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  if (len != sizeof(TankMessage)) return;

  TankMessage msg;
  memcpy(&msg, data, sizeof(msg));

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
      // Mark that we saw START during this wake window.
      startReceived = true;
      break;

    case 52:  // STOP from server
      // Pump session end requested
      stopRequested = true;
      break;

    default:
      break;
  }
}

// ---------------------------------
// SETUP
// ---------------------------------
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(TANK1_PIN, INPUT_PULLUP);
  pinMode(TANK2_PIN, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  esp_now_init();

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, serverAddress, 6);
  peerInfo.channel = 0;
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
    startReceived = false;
  }

  // If we received START during this wake window:
  if (startReceived) {
    // Immediately evaluate current tank states
    bool initialTank1Full = isTankFull(TANK1_PIN);
    bool initialTank2Full = isTankFull(TANK2_PIN);

    // Send READY (51) to server
    sendCode(51);
    // 3-blink handshake on indicator
    blinkLed(3, 120, 120);

    // Enter pump mode
    rtcInPumpMode   = true;
    pumpModeStartMs = millis();

    // Initialize flags
    tank1Sent = false;
    tank2Sent = false;

    // Initialize prev states
    prevTank1Full = initialTank1Full;
    prevTank2Full = initialTank2Full;

    // ---- NEW: handle tanks that are already full at pump start ----
    if (initialTank1Full) {
      sendCode(1);      // Tank1 already full → tell server to turn Pump1 off
      tank1Sent = true;
    }
    if (initialTank2Full) {
      sendCode(2);      // Tank2 already full → tell server to turn Pump2 off
      tank2Sent = true;
    }

    // From next loop() call, we'll be in pump mode
    return;
  }

  // Check if idle listen window expired
  if (millis() - idleStartMs > IDLE_LISTEN_MS) {
    // Go back to deep sleep for 5 seconds
    idleInitDone = false;
    esp_sleep_enable_timer_wakeup(IDLE_WAKE_INTERVAL_US);
    esp_deep_sleep_start();
  }

  // Small delay to avoid busy looping
  delay(50);
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
  bool touchNow = isTouchActive();
  if (touchNow && !prevTouchActive) {
    sendCode(3);      // indicator test
    delay(200);
  }
  prevTouchActive = touchNow;

  // ---- STOP handling ----
  if (stopRequested) {
    stopRequested = false;

    // 4 blinks on indicator entering idle
    blinkLed(4);

    // ACK_STOP back to server (53)
    sendCode(53);
    delay(200);

    // Back to idle mode (deep sleep cycles)
    rtcInPumpMode = false;
    idleInitDone  = false;

    esp_sleep_enable_timer_wakeup(IDLE_WAKE_INTERVAL_US);
    esp_deep_sleep_start();
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
