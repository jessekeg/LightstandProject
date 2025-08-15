#include <WiFi.h>
#include "esp_wifi.h"
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include "AccelStepper.h"
#include "index.h"
#include <string.h>          // memcpy

// ------ ESP-NOW pairing helper ------
#include <ESPNowPair.h>      // v1.0.1+
using namespace ESPNOWPair;

// ====== MOTOR / APP SETTINGS ======
#define DEG_PER_REV        320
#define STEPS_PER_DEGREE   177.7778

// Motor pins
#define step_y 12
#define dir_y  14
#define step_x 27
#define dir_x  26
#define motorInterfaceType 1

// ====== CONTROL MODE ======
// #define WEB_CONTROL       // web text commands drive moveTo()
#define CONTROL             // joystick over ESP-NOW drives continuous velocity with ramp

// ====== PAIRING BUTTON ======
const int SYNC_BTN = 32; // GPIO32; wire button to GND (library enables INPUT_PULLUP)

// Optional AP settings if you host a page
const char* ap_ssid     = "ESP_WEB_SERVER";
const char* ap_password = nullptr;

AsyncWebServer   server(80);
WebSocketsServer webSocket(81);

AccelStepper xMotor(motorInterfaceType, step_x, dir_x);
AccelStepper yMotor(motorInterfaceType, step_y, dir_y);

// ---------- ESPNOW PAIR MANAGER ----------
PairManager pair;

// Payload from sender
struct Message { long x; long y; };
volatile Message messageRCV;   // updated in onReceive; read via memcpy

// ---------------- Web (optional) ----------------
float movements_to_complete[2] = {0, 0}; // [0]=X, [1]=Y

// ====== SPEED / ACCEL RAMPING PARAMS ======
// Your "configured max speed" (steps/second). Use the same intent as your previous setMaxSpeed calls.
static const float MAX_SPEED_X = (float)(DEG_PER_REV * 50); // example from your code
static const float MAX_SPEED_Y = 200000.0f;                 // example from your code

// Coefficient that scales your configured max. 1.0 = full max, 0.6 = 60% of max, etc.
static const float SPEED_COEFF = 1.0f;

// Acceleration limits (steps/second^2) for our *software* ramp
static const float ACCEL_X = 10000.0f;
static const float ACCEL_Y = 10000.0f;

// Change-only RX logs
static const int       RX_CHANGE_THRESHOLD   = 8;
static const uint32_t  RX_MIN_PRINT_INTERVAL = 60; // ms
long     last_rx_x = 0, last_rx_y = 0;
uint32_t last_rx_print_ms = 0;
bool     first_rx_printed = false;

// Pairing status logs
bool last_pairing = false;
int  last_peer_count = -1;

// --- Velocity ramp state ---
float curSpeedX = 0.0f, curSpeedY = 0.0f;   // current commanded speed (steps/s)
uint32_t lastUpdateMs = 0;

void parseText(uint8_t* text) {
  char str[50];
  String temp = String((char*)text);
  temp.toCharArray(str, 50);

  char* token = strtok(str, " ,");
  int i = 0;
  while (token != NULL && i < 2) {
    movements_to_complete[i] = atoi(token);
    token = strtok(NULL, " ,");
    i++;
  }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] WS: Disconnected\n", num);
      break;
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      Serial.printf("[%u] WS: Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
    } break;
    case WStype_TEXT:
      Serial.printf("[%u] WS RX: %s\n", num, payload);
      parseText(payload);
      webSocket.sendTXT(num, "Received:  " + String((char*)payload));
      break;
  }
}

// ====== JOYSTICK → VELOCITY with ACCEL RAMP ======
inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

inline void rampToward(float target, float accel, float dt, float &current) {
  float maxDelta = accel * dt;           // maximum change allowed this tick
  float diff = target - current;
  if (diff >  maxDelta) diff =  maxDelta;
  if (diff < -maxDelta) diff = -maxDelta;
  current += diff;
}

void controlMotorsVelocityRamp() {
  // Snapshot latest command safely
  Message m; memcpy(&m, (const void*)&messageRCV, sizeof(m));

  // Convert to coefficients [-1.0, +1.0]
  float cx = clampf((float)m.x / 1000.0f, -1.0f, 1.0f);
  float cy = clampf((float)m.y / 1000.0f, -1.0f, 1.0f);

  // Desired target speeds (steps/s)
  float targetX = cx * (SPEED_COEFF * MAX_SPEED_X);
  float targetY = cy * (SPEED_COEFF * MAX_SPEED_Y);

  // Time step
  uint32_t now = millis();
  float dt = (lastUpdateMs == 0) ? 0.0f : (now - lastUpdateMs) / 1000.0f;
  lastUpdateMs = now;

  // Ramp current speeds toward targets
  rampToward(targetX, ACCEL_X, dt, curSpeedX);
  rampToward(targetY, ACCEL_Y, dt, curSpeedY);

  // Apply speeds (runSpeed ignores AccelStepper's internal accel, which is fine—we're doing ramping)
  xMotor.setSpeed(curSpeedX);
  yMotor.setSpeed(curSpeedY);
  xMotor.runSpeed();
  yMotor.runSpeed();
}

// ====== WEB absolute move control (unchanged) ======
void moveMotorsAbsolute() {
  float stepsX = movements_to_complete[0] * STEPS_PER_DEGREE;
  float stepsY = movements_to_complete[1] * STEPS_PER_DEGREE;
  xMotor.moveTo(stepsX);
  yMotor.moveTo(stepsY);
  xMotor.run();
  yMotor.run();
}

void handleSerial() {
  static String inputString = "";
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      inputString.trim();
      if (inputString.length() > 0) {
        uint8_t tmp[50];
        inputString.toCharArray((char*)tmp, 50);
        parseText(tmp);
      }
      inputString = "";
    } else {
      inputString += c;
    }
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(200);

  // Network mode: AP+STA if you also host a page; or STA only if you don't need AP.
  WiFi.mode(WIFI_MODE_APSTA);
  // Example: enable your own AP on a fixed channel if needed:
  // WiFi.softAP(ap_ssid, ap_password, /*channel=*/1);

  // Pairing helper
  pair.setSyncButtonPin(SYNC_BTN);      // internal pull-up enabled
  pair.begin(/*skipWiFiSetup=*/true);   // we already set Wi-Fi mode

  // Receive joystick data into messageRCV (use memcpy for volatile target)
  pair.onReceive([](const uint8_t* mac, const uint8_t* data, int len){
    if (len == (int)sizeof(Message)) {
      Message tmp; memcpy(&tmp, data, sizeof(tmp));
      memcpy((void*)&messageRCV, &tmp, sizeof(tmp));

      // Change-only RX logs
      uint32_t now = millis();
      bool bigChange = (labs(tmp.x - last_rx_x) >= RX_CHANGE_THRESHOLD) ||
                       (labs(tmp.y - last_rx_y) >= RX_CHANGE_THRESHOLD);
      bool slowEnough = (now - last_rx_print_ms) >= RX_MIN_PRINT_INTERVAL;

      if (!first_rx_printed) {
        Serial.printf("[RX] first packet from %02X:%02X:%02X:%02X:%02X:%02X  x=%ld y=%ld\n",
          mac[0],mac[1],mac[2],mac[3],mac[4],mac[5], tmp.x, tmp.y);
        first_rx_printed = true;
        last_rx_x = tmp.x; last_rx_y = tmp.y; last_rx_print_ms = now;
      } else if (bigChange && slowEnough) {
        Serial.printf("[RX] cmd(%ld,%ld)\n", tmp.x, tmp.y);
        last_rx_x = tmp.x; last_rx_y = tmp.y; last_rx_print_ms = now;
      }
    } else {
      Serial.printf("[RX] %d bytes (unexpected size)\n", len);
    }
  });

  // ---- Web server / sockets (optional) ----
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    String html = HTML_CONTENT;  // from index.h
    request->send(200, "text/html", html);
  });
  server.begin();
  Serial.print("[NET] STA IP: "); Serial.println(WiFi.localIP());
  Serial.print("[NET] AP  IP: "); Serial.println(WiFi.softAPIP());

  // ---- AccelStepper base setup (max used as ceiling; we do the ramp in software) ----
  xMotor.setMaxSpeed(MAX_SPEED_X);
  yMotor.setMaxSpeed(MAX_SPEED_Y);
  // Acceleration is not used by runSpeed(), but keep values for reference
  xMotor.setAcceleration(ACCEL_X);
  yMotor.setAcceleration(ACCEL_Y);

  Serial.println("[BOOT] Receiver ready. Hold both sync buttons to pair.");
}

// ================= LOOP ==================
void loop() {
  pair.loop();
  webSocket.loop();
  handleSerial();

  // Pairing / peer status logs
  bool pairing_now = pair.isPairing();
  if (pairing_now != last_pairing) {
    last_pairing = pairing_now;
    Serial.println(pairing_now ? "[PAIRING] Window opened" : "[PAIRING] Window closed");
  }
  int pc = pair.peerCount();
  if (pc != last_peer_count) {
    last_peer_count = pc;
    Serial.print("[STATUS] Peers: "); Serial.println(pc);
    if (pc == 0) Serial.println("[STATUS] No peers. Press sync buttons on both devices.");
  }

  #ifdef CONTROL
    controlMotorsVelocityRamp();   // joystick → speed with acceleration ramp
  #endif

  #ifdef WEB_CONTROL
    moveMotorsAbsolute();          // old web moveTo() behavior
  #endif
}
