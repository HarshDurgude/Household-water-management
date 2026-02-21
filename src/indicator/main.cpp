#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_sleep.h>
#include <esp_wifi.h>

/**
 * ---------------------------------------------------------------------------------------
 * PROJECT: Household Water Management System (Indicator Unit)
 * DESCRIPTION:
 * This unit is installed at the water tank. It monitors the tank level using 
 * metallic probes (tank full detection) and an ultrasonic sensor (numeric level).
 * It communicates with the Server Unit via ESP-NOW and supports deep-sleep for 
 * power saving during idle periods.
 * ---------------------------------------------------------------------------------------
 */

// ---------------------------------------------------------------------------------------
// GLOBAL CONSTANTS & PINS
// ---------------------------------------------------------------------------------------
#define DEBUG 1             // Set to 0 to disable all serial logs
#define BOOT_PIN 0          // ESP32 BOOT button (GPIO0)
#define LED_PIN 2           // Onboard LED
#define TANK_PIN 23         // Metallic probe pin (Pulled LOW when water touches)
#define ULTRA_TRIG_PIN 18   // Ultrasonic trigger pin
#define ULTRA_ECHO_PIN 19   // Ultrasonic echo pin
#define TOUCH_PIN 4         // Capacitive touch for range testing

// Tank Geometry
#define TANK_HEIGHT_CM 32   // Physical height of the tank

// Timing & Sleep Constants
const unsigned long IDLE_LISTEN_MS = 2000;              // Time to listen for Server on wake
const unsigned long IDLE_WAKE_INTERVAL_US = 5ULL * 1000000ULL; // Sleep duration (5s)
const unsigned long LED_COOLDOWN_MS = 2000;
#define RANGE_LONG_PRESS_MS 2500

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
 */
typedef struct {
  uint8_t tankId;
} TankMessage;

/**
 * UltraData: Carries numeric sensor data back to Server
 */
typedef struct {
  uint8_t type;    // 61 = distance_cm, 63 = percentage
  int16_t value;   // Scaled by 100
} UltraData;

/**
 * ControlMessage: Received from server to set sync policy
 */
typedef struct {
  uint8_t type;    // 70 = policy
  uint8_t value;   // 0 = CONTINUOUS, 1 = NORMAL
} ControlMessage;

// ---------------------------------------------------------------------------------------
// GLOBAL STATE & SETTINGS
// ---------------------------------------------------------------------------------------
// Server MAC Address (Room Unit)
uint8_t serverAddress[] = { 0x00, 0x4B, 0x12, 0x2F, 0xFA, 0x08 };

// Operational Modes
enum SessionMode {
  MODE_CONTINUOUS = 0,  // Stays awake during session
  MODE_NORMAL     = 1   // Enters deep sleep after session/idle
};

volatile SessionMode sessionMode = MODE_NORMAL;
volatile bool allowIdleSleep = true;
volatile bool pumpCommit = false; // True if a session is currently active

// Session Persistence (stored in RTC memory to survive deep sleep)
RTC_DATA_ATTR bool rtcInPumpMode = false;

// Handshake & Internal Flags
bool startLatched = false;
bool idleInitDone = false;
unsigned long idleStartMs = 0;
bool tank1Sent = false;
bool prevTank1Full = false;
volatile bool stopRequested = false;

// Touch Calibration
uint16_t touchBaseline = 0;
const uint16_t TOUCH_MARGIN = 20;

// UI & Feedback
unsigned long ledCooldownUntil = 0;
unsigned long bootPressStart = 0;
bool bootPressConsumed = false;

// ---------------------------------------------------------------------------------------
// HELPER FUNCTIONS
// ---------------------------------------------------------------------------------------

/**
 * Utility to execute a block once based on a flag
 */
bool logOnce(bool &flag) {
  if (flag) return false;
  flag = true;
  return true;
}

/**
 * Visual feedback via onboard LED
 */
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

/**
 * Sends a command code to Server via ESP-NOW
 */
esp_err_t sendCode(uint8_t code) {
  DBG2("INDICATOR TX → code = ", code);
  TankMessage msg;
  msg.tankId = code;
  return esp_now_send(serverAddress, (uint8_t*)&msg, sizeof(msg));
}

/**
 * Validates tank full state using multiple checks to avoid noise
 */
bool isTankFull(int pin) {
  const int checks = 50;
  int lowCount = 0;
  for (int i = 0; i < checks; i++) {
    if (digitalRead(pin) == LOW) lowCount++;
    delay(10);
  }
  return (lowCount >= 1);
}

/**
 * Performs ultrasonic distance measurement
 */
float measureUltrasonicCm() {
  digitalWrite(ULTRA_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRA_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRA_TRIG_PIN, LOW);

  long duration = pulseIn(ULTRA_ECHO_PIN, HIGH, 30000);
  if (duration == 0) return -1.0;

  // Sound speed (0.0343 cm/us) / 2 (round trip)
  return (duration * 0.0343) / 2.0;
}

/**
 * Converts distance to water height and percentage
 */
void computeWaterLevel(float distanceCm, float &waterHeight, float &percent) {
  if (distanceCm <= 0) { waterHeight = -1; percent = -1; return; }

  // Adjusting for sensor offset (4.8cm calibration)
  waterHeight = TANK_HEIGHT_CM - (distanceCm - 4.8);
  if (waterHeight < 0) waterHeight = 0;
  if (waterHeight > TANK_HEIGHT_CM) waterHeight = TANK_HEIGHT_CM;

  percent = (waterHeight / TANK_HEIGHT_CM) * 100.0;
}

/**
 * Calibrates capacitive touch baseline
 */
void calibrateTouch() {
  delay(500);
  uint32_t sum = 0;
  for (int i = 0; i < 20; i++) { sum += touchRead(TOUCH_PIN); delay(50); }
  touchBaseline = sum / 20;
}

// ---------------------------------------------------------------------------------------
// ESP-NOW CALLBACKS
// ---------------------------------------------------------------------------------------

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) DBG("INDICATOR: ESP-NOW send FAILED");
}

void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  // 1. Control Policy Message
  if (len == sizeof(ControlMessage)) {
    ControlMessage ctrl;
    memcpy(&ctrl, data, sizeof(ctrl));
    if (ctrl.type == 70) {
      static bool policyLogged = false;
      if (logOnce(policyLogged)) DBG2("INDICATOR: Session mode = ", ctrl.value == 0 ? "CONTINUOUS" : "NORMAL");
      
      if (ctrl.value == 1) {
        sessionMode = MODE_NORMAL; allowIdleSleep = true;
        DBG("INDICATOR: Mode NORMAL (Auto-Sleep Enabled)");
      } else {
        sessionMode = MODE_CONTINUOUS; allowIdleSleep = false;
        DBG("INDICATOR: Mode CONTINUOUS (Stay Awake)");
      }
    }
    return;
  }

  // 2. Command Message
  if (len != sizeof(TankMessage)) return;
  TankMessage msg;
  memcpy(&msg, data, sizeof(msg));

  switch (msg.tankId) {
    case 1:  blinkLed(1); break; // ACK Tank Full
    case 3:  blinkLed(3, 120, 120); break; // ACK Test Ping
    case 4:  blinkLed(3, 120, 120); sendCode(4); break; // Server Range Test Pulse

    case 10: // Server Status Query (Tank Probes)
      if (isTankFull(TANK_PIN)) sendCode(11);
      else sendCode(12);
      break;

    case 50: // START Session Request
      DBG("INDICATOR: START packet received");
      if (!rtcInPumpMode && !startLatched) {
        DBG("INDICATOR: Session Handshake Accepted");
        rtcInPumpMode = true;
        startLatched = true;
        pumpCommit = true;
        idleInitDone = false;

        // Immediate Ready Signal (3x for reliability)
        for (int i = 0; i < 3; ++i) { sendCode(51); delay(40); }
        blinkLed(3, 120, 120);

        // Check if tank is already full upon starting
        bool initialFull = isTankFull(TANK_PIN);
        prevTank1Full = initialFull;
        tank1Sent = false;
        if (initialFull) { sendCode(1); tank1Sent = true; }
      }
      break;

    case 52: // STOP Session Request
      stopRequested = true;
      break;

    case 60: { // Ultrasonic Measurement Query
      float dist = measureUltrasonicCm();
      float waterH = 0, percent = 0;
      computeWaterLevel(dist, waterH, percent);

      if (dist < 0) { DBG("INDICATOR: Ultrasonic Error"); break; }

      Serial.printf("INDICATOR: Tank %.1f%% | Distance %.1f cm\n", percent, dist);

      // Send numeric data back to server
      UltraData out;
      out.type = 61; out.value = (int16_t)(dist * 100);
      esp_now_send(serverAddress, (uint8_t*)&out, sizeof(out));
      delay(40);
      out.type = 63; out.value = (int16_t)(percent * 100);
      esp_now_send(serverAddress, (uint8_t*)&out, sizeof(out));
      break;
    }
  }
}

// ---------------------------------------------------------------------------------------
// HANDLERS
// ---------------------------------------------------------------------------------------

/**
 * Manages indicator logic during non-pumping periods
 */
void handleIdleMode() {
  if (!allowIdleSleep) {
    static bool awakeLogged = false;
    if (logOnce(awakeLogged)) DBG("INDICATOR: Idle sleep disabled → Staying awake");
    delay(100); return;
  }

  if (pumpCommit) return;

  if (!idleInitDone) {
    idleStartMs = millis();
    idleInitDone = true;
    static bool idleLogged = false;
    if (logOnce(idleLogged)) DBG("INDICATOR: Idle mode active");
  }

  // Yield to allow background tasks (ESP-NOW)
  delay(10); yield();

  // Check if session started while in idle loop
  if (pumpCommit && !startLatched) {
    rtcInPumpMode = true; startLatched = true; idleInitDone = false;
    sendCode(51); // READY
    blinkLed(3, 120, 120);
    return;
  }

  // Go to deep sleep if timeout reached
  if (millis() - idleStartMs > IDLE_LISTEN_MS) {
    DBG("INDICATOR: Idle timeout → Sleeping");
    esp_sleep_enable_timer_wakeup(IDLE_WAKE_INTERVAL_US);
    esp_deep_sleep_start();
  }
}

/**
 * Manages indicator logic during active pumping sessions
 */
void handlePumpMode() {
  // 1. Session Termination
  if (stopRequested) {
    stopRequested = false;
    DBG("INDICATOR: Session STOP received");
    blinkLed(4);
    sendCode(53); // ACK_STOP
    delay(200);

    pumpCommit = false;
    startLatched = false;
    rtcInPumpMode = false;
    idleInitDone = false;

    if (sessionMode == MODE_NORMAL) {
      DBG("INDICATOR: Sleep mode → Deep Sleep");
      esp_sleep_enable_timer_wakeup(IDLE_WAKE_INTERVAL_US);
      esp_deep_sleep_start();
    } else {
      DBG("INDICATOR: Continuous mode → Staying awake");
    }
  }

  // 2. Tank Level Monitoring
  bool tankFull = isTankFull(TANK_PIN);
  if (tankFull && !prevTank1Full && !tank1Sent) {
    DBG("INDICATOR: Tank FULL (Probes)");
    sendCode(1); // Send Tank Full notification
    tank1Sent = true;
  }
  prevTank1Full = tankFull;

  // 3. User Interface (BOOT Button)
  if (digitalRead(BOOT_PIN) == LOW) {
    if (bootPressStart == 0 && !bootPressConsumed) bootPressStart = millis();
    if (!bootPressConsumed && (millis() - bootPressStart > RANGE_LONG_PRESS_MS)) {
      DBG("INDICATOR: Range Test Triggered");
      sendCode(3);
      bootPressConsumed = true;
    }
  } else { bootPressStart = 0; bootPressConsumed = false; }

  delay(300); // Loop throttle
}

// ---------------------------------------------------------------------------------------
// ARDUINO SETUP & LOOP
// ---------------------------------------------------------------------------------------

void setup() {
  allowIdleSleep = true;
  Serial.begin(115200);
  delay(300);
  DBG("SYSTEM BOOT");

  pinMode(BOOT_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(TANK_PIN, INPUT_PULLUP);
  pinMode(ULTRA_TRIG_PIN, OUTPUT);
  pinMode(ULTRA_ECHO_PIN, INPUT);
  digitalWrite(ULTRA_TRIG_PIN, LOW);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  esp_now_init();
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, serverAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  calibrateTouch();
  prevTank1Full = isTankFull(TANK_PIN);
  
  idleInitDone = false;
  stopRequested = false;
}

void loop() {
  if (!rtcInPumpMode) {
    handleIdleMode(); // Search for server/sleep cycle
  } else {
    handlePumpMode(); // Active session monitoring
  }
}
