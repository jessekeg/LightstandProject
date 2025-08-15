#include <WiFi.h>
#include <ESPNowPair.h>
using namespace ESPNOWPair;

// -------- pins --------
#define SW_PIN 13          // pairing button (wire to GND). Library sets INPUT_PULLUP.
#define X_PIN  A0          // make sure these map to ADC-capable GPIOs on your board
#define Y_PIN  A3

// -------- smoothing / mapping --------
static const float ALPHA   = 0.25f;  // EMA smoothing
static const int   ADC_MAX = 4095;   // ESP32 12-bit ADC
static const int   OUT_MAX = 1000;   // normalized output

// Center & deadzone (raw ADC units) — adjust to your joystick
static const int X_CENTER = 1810;
static const int Y_CENTER = 1850;
static const int X_DEAD   = 25;
static const int Y_DEAD   = 25;

// -------- change-only print controls --------
static const int       CMD_CHANGE_THRESHOLD    = 8;    // min delta to print
static const uint32_t  MIN_PRINT_INTERVAL_MS   = 40;   // don’t print faster than this

// -------- payload --------
struct Message { int x; int y; };

PairManager pair;

float x_ema = X_CENTER;
float y_ema = Y_CENTER;

// change-detection state
int last_cmd_x = 0, last_cmd_y = 0;
uint32_t last_print_ms = 0;

// status tracking
bool last_pairing = false;
int  last_peer_count = -1;

// ------- helpers -------
inline int smoothEMA(int raw, float &ema) {
  ema = ALPHA * raw + (1.0f - ALPHA) * ema;
  return (int)(ema + 0.5f);
}

inline int applyDeadzoneAndMap(int value, int center, int dead) {
  int delta = value - center;
  int mag = abs(delta);
  if (mag <= dead) return 0;
  int sign = (delta < 0) ? -1 : 1;
  int effective = mag - dead;
  int maxSpan = (sign < 0) ? (center - dead) : (ADC_MAX - center - dead);
  if (maxSpan < 1) return 0;
  long out = (long)effective * OUT_MAX / maxSpan;
  if (out > OUT_MAX) out = OUT_MAX;
  return sign * (int)out;
}

// optional soft response curve (finer control near center)
inline int softCurve(int v) {
  long s = (v < 0) ? -1 : 1;
  long a = abs(v);
  long curved = (a * a * a) / (OUT_MAX * OUT_MAX);
  return (int)(s * curved);
}

inline bool changedEnough(int x_new, int y_new) {
  return (abs(x_new - last_cmd_x) >= CMD_CHANGE_THRESHOLD) ||
         (abs(y_new - last_cmd_y) >= CMD_CHANGE_THRESHOLD);
}

void setup() {
  Serial.begin(115200);

  // Optional: expand ADC range if joystick is 0–3.3V
  // analogSetPinAttenuation(X_PIN, ADC_11db);
  // analogSetPinAttenuation(Y_PIN, ADC_11db);

  // ESPNOW pairing helper
  WiFi.mode(WIFI_STA);                 // keep both peers on same channel
  pair.setSyncButtonPin(SW_PIN);       // internal pull-up enabled by library
  pair.begin();                        // loads saved peers too

  // Print the first successful send, and any failures
  pair.onSent([](const uint8_t* mac, esp_now_send_status_t s){
    static bool printedFirstOK = false;
    if (s == ESP_NOW_SEND_SUCCESS) {
      if (!printedFirstOK) {
        Serial.print("[SEND] OK -> ");
        for (int i=0;i<6;i++){ if(i) Serial.print(':'); Serial.printf("%02X", mac[i]); }
        Serial.println();
        printedFirstOK = true;
      }
    } else {
      Serial.print("[SEND] FAIL status=");
      Serial.print((int)s);
      Serial.print(" -> ");
      for (int i=0;i<6;i++){ if(i) Serial.print(':'); Serial.printf("%02X", mac[i]); }
      Serial.println();
    }
  });

  Serial.println("[BOOT] Sender ready.");
  Serial.println("[HINT] Hold the sync button (GPIO13→GND) on BOTH boards to pair.");
}

void loop() {
  pair.loop();  // handles pairing beacons & button

  // ----- status / mode prints -----
  // Pairing window transitions
  bool pairing_now = pair.isPairing();
  if (pairing_now != last_pairing) {
    last_pairing = pairing_now;
    Serial.println(pairing_now ? "[PAIRING] Window opened" : "[PAIRING] Window closed");
  }

  // Peer count changes
  int pc = pair.peerCount();
  if (pc != last_peer_count) {
    last_peer_count = pc;
    Serial.print("[STATUS] Peers: "); Serial.println(pc);
    if (pc == 0) {
      Serial.println("[STATUS] No peers. Press the sync buttons on both devices to pair.");
    }
  }

  // ----- joystick pipeline -----
  int x_raw = analogRead(X_PIN);
  int y_raw = analogRead(Y_PIN);

  int x_sm = smoothEMA(x_raw, x_ema);
  int y_sm = smoothEMA(y_raw, y_ema);

  int x_norm = applyDeadzoneAndMap(x_sm, X_CENTER, X_DEAD);
  int y_norm = applyDeadzoneAndMap(y_sm, Y_CENTER, Y_DEAD);

  int x_cmd = softCurve(x_norm);
  int y_cmd = softCurve(y_norm);

  // send
  Message msg{ x_cmd, y_cmd };
  esp_err_t e = pair.sendToAll((uint8_t*)&msg, sizeof(msg));
  // (If there are no peers yet, e == ESP_ERR_ESPNOW_NOT_FOUND — we handle by status prints above)

  // print only on meaningful changes (and not too often)
  uint32_t now = millis();
  if (changedEnough(x_cmd, y_cmd) && (now - last_print_ms >= MIN_PRINT_INTERVAL_MS)) {
    Serial.print("[JOY] cmd(");
    Serial.print(x_cmd); Serial.print(", ");
    Serial.print(y_cmd); Serial.println(")");
    last_cmd_x = x_cmd;
    last_cmd_y = y_cmd;
    last_print_ms = now;
  }

  delay(5); // fast loop; EMA provides smoothing
}
