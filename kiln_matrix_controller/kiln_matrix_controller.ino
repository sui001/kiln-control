/*
 * PROJECT: Kiln Controller, Waveshare ESP32-S3-Matrix build
 * VERSION: 2.1
 *
 * Grown from kiln_sensor_display_v1_5 (v1.6), with the control loop from
 * kiln_controller.ino (v2.0, SuperMini) brought across and reworked for a
 * kiln switched by MECHANICAL relays rather than SSRs:
 *
 *   - Relay window: one on/off cycle per RELAY_WINDOW_MS (30 s). The on-time
 *     is latched at the start of each window, so a relay switches at most
 *     once on and once off per window, and no pulse is shorter than
 *     RELAY_MIN_PULSE_MS. Deltrol 275s are rated in thousands of operations,
 *     not millions; a 5 s SSR window would chew through them.
 *   - Own small PID in % duty (0..100), conditional-integration anti-windup.
 *     v2.0's PID_v1 gains were scaled against a 5000 ms output, so they meant
 *     almost nothing; these are per-degree and need tuning on a test fire.
 *   - Step types: ramp / hold / full / cool / off.
 *       ramp: linear setpoint over the duration, then WAITS at target until
 *             the kiln actually arrives (within STEP_TOLERANCE_C), so a soak
 *             that follows never starts early. Faults if it can't get there.
 *       hold: holds the previous step's target for the duration.
 *       full: 100 % until within tolerance of target (fault on timeout).
 *       cool: relays off until temp falls to target. No timeout, it is safe.
 *       off:  relays off for the duration.
 *   - Safety: relays off on over-temp, on SUSTAINED thermocouple fault
 *     (TC_FAULT_READS in a row, not one glitch), on stop, on boot, and a
 *     task watchdog reboots into relays-off if the loop ever hangs.
 *     The lid safety switch in the coil loop is the real interlock; this
 *     code cannot see the lid.
 *
 * KNOWN GAP: run position is not persisted. A reboot or power cut mid-fire
 *   leaves the kiln idle with relays off (safe, but the piece cools
 *   uncontrolled).
 *
 * WIRING (Waveshare ESP32-S3-Matrix):
 *   MAX31855: SCK->GP36  CS->GP37  SO->GP38  (FSPI), 10nF across T+/T-
 *   ILI9341:  CS->GP1  RST->GP2  DC->GP3  MOSI->GP4  SCK->GP5  (HSPI)
 *   Relays:   GP39 -> ULN2003 IN1..IN3 (tied) -> OUT1..3 -> coil (-) loop
 *             ULN JST pin 5 (+V, 12 V) -> coil (+) loop, via lid switch
 *
 * LIBRARIES: Adafruit_MAX31855, Adafruit_GFX, Adafruit_ILI9341,
 *   ESPAsyncWebServer + AsyncTCP (ESP32Async), ArduinoJson v7.
 * BOARD: ESP32S3 Dev Module, USB CDC On Boot: Enabled.
 */

#include <SPI.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_MAX31855.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <AsyncJson.h>
#include <esp_task_wdt.h>

// ─── CONFIG ───────────────────────────────────────────────────────────────────

#define VERSION         "2.1"

// WiFi credentials live in secrets.h beside this file (gitignored), as
//   #define WIFI_SSID     "..."
//   #define WIFI_PASSWORD "..."
// Placeholders below are only used if secrets.h is missing.
#if __has_include("secrets.h")
  #include "secrets.h"
#else
  #define WIFI_SSID       "YOUR_SSID"
  #define WIFI_PASSWORD   "YOUR_PASSWORD"
#endif

#define MAX_SCK_PIN     36
#define MAX_SO_PIN      38
#define MAX_CS_PIN      37

#define TFT_CS          1
#define TFT_RST         2
#define TFT_DC          3
#define TFT_MOSI        4
#define TFT_SCK         5

#define RELAY_PIN        39   // -> ULN2003 IN1..IN3 -> relay coils
#define RELAY_TEST_TIMEOUT_MS  60000UL  // manual "relay on" auto-offs after this

// Relay window: mechanical relays, so slow. See header.
#define RELAY_WINDOW_MS      30000UL
#define RELAY_MIN_PULSE_MS    2000UL

// PID in % duty. Starting guesses, tune on an empty test fire.
#define PID_KP   4.0     // % per C of error
#define PID_KI   0.01    // % per C per second
#define PID_KD   0.0     // % per (C/s)

#define SAFETY_MAX_TEMP      950.0   // absolute cutoff, C. Glass work tops out ~850.
#define STEP_TOLERANCE_C       5.0   // "arrived" band for ramp/full/cool
#define STEP_WAIT_TIMEOUT_MS  (3UL * 3600UL * 1000UL)  // full, or ramp catch-up
#define TC_FAULT_READS          6    // consecutive bad reads (3 s) before faulting
#define OVERSHOOT_WARN_C       40.0  // ramp/hold: warn if this far above setpoint
#define WDT_TIMEOUT_MS      15000

#define GRAPH_SAMPLE_MS     60000UL  // TFT graph: one point a minute, 2 h window

#define READ_INTERVAL_MS    500
#define GRAPH_HISTORY_LEN   120   // TFT rolling graph, GRAPH_SAMPLE_MS apart
#define WIFI_RECONNECT_CHECK_MS  10000

#define USE_INTERNAL_SENSOR  0
#define CALIBRATION_OFFSET_C  0.0
#define PLACEHOLDER_SETPOINT  20.0

#define MAX_SCHEDULE_STEPS   30
#define SCHEDULE_FILE_PATH   "/schedule.json"

// ─── COLOURS (TFT — RGB565) ────────────────────────────────────────────────────

#define BG_COLOR      ILI9341_BLACK
#define TEXT_COLOR    ILI9341_WHITE
#define ACCENT_COLOR  0xFD20
#define MUTED_COLOR   0x8410
#define GRID_COLOR    0x2104
#define ERROR_COLOR   ILI9341_RED
#define OK_COLOR      0x07E0
#define GRAPH_COLOR   0xFD20

// ─── GLOBALS ──────────────────────────────────────────────────────────────────

SPIClass maxSPI(FSPI);
SPIClass tftSPI(HSPI);

Adafruit_MAX31855 thermocouple(MAX_CS_PIN, &maxSPI);
Adafruit_ILI9341 tft(&tftSPI, TFT_DC, TFT_CS, TFT_RST);

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

unsigned long lastReadMs       = 0;
unsigned long lastWifiCheckMs  = 0;
float currentTemp        = 0;
float currentSetpoint    = PLACEHOLDER_SETPOINT;
bool  hasFault            = false;
String faultDetail        = "";
unsigned long readCount   = 0;

// Median-of-5 spike filter: glitch reads (seen at 7-24 C against a steady
// 26 C, errReg clean, sometimes two close together) are dropped rather than
// fed onward. Costs ~1 s of lag at 500 ms reads, irrelevant for a kiln.
#define MED_N 5
float medBuf[MED_N] = {NAN, NAN, NAN, NAN, NAN};
int   medIdx    = 0;

// Relay output
bool relayOn = false;
bool manualTest = false;               // "relay on" from serial, idle only
unsigned long manualSinceMs = 0;
unsigned long windowStartMs = 0;
unsigned long windowOnMs    = 0;       // latched at window start
String serialLine = "";

// Control
enum CtrlState { CTRL_IDLE, CTRL_RUNNING, CTRL_COMPLETE, CTRL_FAULT };
CtrlState ctrlState = CTRL_IDLE;
float dutyPct       = 0;               // 0..100, what the relay window uses
float pidIntegral   = 0;
float pidLastPv     = NAN;
int   currentStep   = 0;
unsigned long stepStartMs = 0;
float stepStartTemp = 20;
bool  stepWaiting   = false;           // ramp done, waiting to arrive
int   badReads      = 0;
String warnDetail   = "";
unsigned long lastGraphMs = 0;

float graphHistory[GRAPH_HISTORY_LEN];
int graphIndex   = 0;
int graphCount   = 0;

#define GRAPH_X       10
#define GRAPH_Y       95
#define GRAPH_W       300
#define GRAPH_H       100
float graphMinC = 0, graphMaxC = 60;   // autoscaled in drawGraphLine

#define NETSTAT_X     200
#define NETSTAT_Y     11
#define NETSTAT_W     116
#define NETSTAT_H     12

// ─── SCHEDULE (RAM copy — source of truth is /schedule.json) ─────────────────

struct ScheduleStep {
  char type[6];        // "ramp", "hold", "full", "off"
  float temp;          // target temp, degC — used by ramp/full, ignored otherwise
  uint32_t durationMs; // used by ramp/hold/off — 0 for full (no fixed duration)
};

// TODO (deferred until PID exists, see v1.5 notes): ramp/soak deviation
// alarm — if actual lags planned target by more than X for more than
// Y, fault out rather than silently pushing on. Sui's call: default
// threshold should be deliberately LOOSE/generous, not tight, when
// this gets built. Not implemented yet — nothing to tune against
// without a PID loop driving the SSR.

ScheduleStep schedule_[MAX_SCHEDULE_STEPS];
int scheduleStepCount   = 0;
uint32_t scheduleTotalMs = 0;
float scheduleMaxTempC   = 0;

unsigned long scheduleStartMs = 0;

void recomputeScheduleStats() {
  scheduleTotalMs = 0;
  scheduleMaxTempC = 0;
  for (int i = 0; i < scheduleStepCount; i++) {
    if (strcmp(schedule_[i].type, "full") != 0 && strcmp(schedule_[i].type, "cool") != 0) {
      scheduleTotalMs += schedule_[i].durationMs;
    }
    if (strcmp(schedule_[i].type, "ramp") == 0 || strcmp(schedule_[i].type, "full") == 0) {
      if (schedule_[i].temp > scheduleMaxTempC) scheduleMaxTempC = schedule_[i].temp;
    }
  }
}

void loadScheduleFromFS() {
  scheduleStepCount = 0;
  if (!LittleFS.exists(SCHEDULE_FILE_PATH)) {
    recomputeScheduleStats();
    return;
  }
  File f = LittleFS.open(SCHEDULE_FILE_PATH, "r");
  if (!f) { recomputeScheduleStats(); return; }

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) {
    Serial.printf("Schedule parse failed: %s\n", err.c_str());
    recomputeScheduleStats();
    return;
  }

  for (JsonObject stepObj : doc.as<JsonArray>()) {
    if (scheduleStepCount >= MAX_SCHEDULE_STEPS) break;
    const char* t = stepObj["type"] | "ramp";
    strncpy(schedule_[scheduleStepCount].type, t, sizeof(schedule_[scheduleStepCount].type) - 1);
    schedule_[scheduleStepCount].type[sizeof(schedule_[scheduleStepCount].type) - 1] = '\0';
    schedule_[scheduleStepCount].temp = stepObj["temp"] | 0.0;
    long hrs  = stepObj["hrs"]  | 0;
    long mins = stepObj["mins"] | 0;
    schedule_[scheduleStepCount].durationMs = (uint32_t)((hrs * 3600L + mins * 60L) * 1000L);
    scheduleStepCount++;
  }
  recomputeScheduleStats();
  Serial.printf("Schedule loaded: %d steps, total %lus, peak %.0fC\n",
    scheduleStepCount, scheduleTotalMs / 1000, scheduleMaxTempC);
}

// ─── GRAPH HELPERS (TFT) ───────────────────────────────────────────────────────

void pushGraphPoint(float temp) {
  graphHistory[graphIndex] = temp;
  graphIndex = (graphIndex + 1) % GRAPH_HISTORY_LEN;
  if (graphCount < GRAPH_HISTORY_LEN) graphCount++;
}

int tempToY(float temp) {
  float clamped = constrain(temp, graphMinC, graphMaxC);
  float frac = (clamped - graphMinC) / (graphMaxC - graphMinC);
  return GRAPH_Y + GRAPH_H - (int)(frac * GRAPH_H);
}

int graphGridStep() {
  float span = graphMaxC - graphMinC;
  return span <= 60 ? 10 : span <= 300 ? 50 : 100;
}

void drawGraphFrame() {
  tft.drawRect(GRAPH_X, GRAPH_Y, GRAPH_W, GRAPH_H, GRID_COLOR);
}

void drawGraphLine() {
  // autoscale to the history, snapped to the grid step, min 20 C span
  if (graphCount > 0) {
    int startIdx = (graphIndex - graphCount + GRAPH_HISTORY_LEN) % GRAPH_HISTORY_LEN;
    float lo = 1e9, hi = -1e9;
    for (int i = 0; i < graphCount; i++) {
      float v = graphHistory[(startIdx + i) % GRAPH_HISTORY_LEN];
      lo = min(lo, v); hi = max(hi, v);
    }
    if (hi - lo < 20) { float mid = (hi + lo) / 2; lo = mid - 10; hi = mid + 10; }
    graphMinC = lo; graphMaxC = hi;
    int g = graphGridStep();
    graphMinC = floor(lo / g) * g;
    graphMaxC = ceil(hi / g) * g;
    if (graphMaxC <= graphMinC) graphMaxC = graphMinC + g;
  }
  tft.fillRect(GRAPH_X + 1, GRAPH_Y + 1, GRAPH_W - 2, GRAPH_H - 2, BG_COLOR);
  tft.fillRect(GRAPH_X + GRAPH_W + 1, GRAPH_Y - 4, tft.width() - GRAPH_X - GRAPH_W - 1, GRAPH_H + 8, BG_COLOR);
  int g = graphGridStep();
  for (int t = (int)graphMinC; t <= (int)graphMaxC; t += g) {
    int y = tempToY(t);
    tft.drawFastHLine(GRAPH_X, y, GRAPH_W, GRID_COLOR);
    tft.setTextColor(TEXT_COLOR);
    tft.setTextSize(1);
    tft.setCursor(GRAPH_X + GRAPH_W + 2, y - 3);
    tft.print(t);
  }
  if (graphCount < 2) return;
  int startIdx = (graphIndex - graphCount + GRAPH_HISTORY_LEN) % GRAPH_HISTORY_LEN;
  for (int i = 0; i < graphCount - 1; i++) {
    int idxA = (startIdx + i) % GRAPH_HISTORY_LEN;
    int idxB = (startIdx + i + 1) % GRAPH_HISTORY_LEN;
    int xA = GRAPH_X + (int)((float)i / (GRAPH_HISTORY_LEN - 1) * GRAPH_W);
    int xB = GRAPH_X + (int)((float)(i + 1) / (GRAPH_HISTORY_LEN - 1) * GRAPH_W);
    tft.drawLine(xA, tempToY(graphHistory[idxA]), xB, tempToY(graphHistory[idxB]), GRAPH_COLOR);
  }
}

// ─── DISPLAY LAYOUT (TFT) ──────────────────────────────────────────────────────

void drawStaticLayout() {
  tft.fillScreen(BG_COLOR);
  tft.setTextColor(ACCENT_COLOR);
  tft.setTextSize(2);
  tft.setCursor(8, 6);
  tft.println("KILN CONTROLLER");
  tft.drawFastHLine(0, 28, tft.width(), GRID_COLOR);
  drawGraphFrame();
}

void drawNetworkStatus() {
  tft.fillRect(NETSTAT_X, NETSTAT_Y, NETSTAT_W, NETSTAT_H, BG_COLOR);
  tft.setTextSize(1);
  tft.setCursor(NETSTAT_X, NETSTAT_Y);
  if (WiFi.status() == WL_CONNECTED) {
    tft.setTextColor(OK_COLOR);
    tft.print(WiFi.localIP());
  } else {
    tft.setTextColor(ERROR_COLOR);
    tft.print("NO WIFI");
  }
}

void drawStats(float temp, float setpoint, bool fault) {
  tft.fillRect(0, 35, tft.width(), 55, BG_COLOR);
  if (fault) {
    tft.setTextColor(ERROR_COLOR);
    tft.setTextSize(2);
    tft.setCursor(8, 40);
    tft.print("FAULT: ");
    tft.setTextSize(1);
    tft.setCursor(8, 60);
    tft.println(faultDetail);
    return;
  }
  tft.setTextColor(ACCENT_COLOR);
  tft.setTextSize(3);
  tft.setCursor(8, 35);
  tft.print(temp, 1);
  tft.setTextSize(2);
  tft.print(" C");
  tft.setTextColor(MUTED_COLOR);
  tft.setTextSize(1);
  tft.setCursor(180, 38);
  tft.println("SETPOINT");
  tft.setTextColor(TEXT_COLOR);
  tft.setTextSize(2);
  tft.setCursor(180, 50);
  tft.print(setpoint, 0);
  tft.print(" C");
  float delta = temp - setpoint;
  tft.setTextColor(MUTED_COLOR);
  tft.setTextSize(1);
  tft.setCursor(8, 65);
  tft.print("Delta: ");
  tft.setTextColor(delta >= 0 ? ACCENT_COLOR : OK_COLOR);
  tft.print(delta >= 0 ? "+" : "");
  tft.print(delta, 1);
  tft.print(" C");
}

float medianFilter(float v) {
  medBuf[medIdx] = v;
  medIdx = (medIdx + 1) % MED_N;
  float t[MED_N];
  int n = 0;
  for (int i = 0; i < MED_N; i++) if (!isnan(medBuf[i])) t[n++] = medBuf[i];
  for (int i = 1; i < n; i++) {            // insertion sort, n <= 5
    float x = t[i]; int j = i - 1;
    while (j >= 0 && t[j] > x) { t[j + 1] = t[j]; j--; }
    t[j + 1] = x;
  }
  return t[n / 2];
}

const char* stateName() {
  switch (ctrlState) {
    case CTRL_IDLE:     return "IDLE";
    case CTRL_RUNNING:  return "RUNNING";
    case CTRL_COMPLETE: return "COMPLETE";
    default:            return "FAULT";
  }
}

void drawRunStatus() {
  tft.fillRect(0, 203, tft.width(), 37, BG_COLOR);
  tft.setTextSize(2);
  tft.setCursor(8, 205);
  uint16_t c = ctrlState == CTRL_FAULT ? ERROR_COLOR
             : ctrlState == CTRL_RUNNING ? ACCENT_COLOR : OK_COLOR;
  tft.setTextColor(c);
  if (ctrlState == CTRL_RUNNING) {
    tft.printf("%s %d/%d %s", schedule_[currentStep].type, currentStep + 1,
               scheduleStepCount, stepWaiting ? "wait" : "");
  } else {
    tft.print(stateName());
  }
  tft.setCursor(200, 205);
  tft.setTextColor(relayOn ? ERROR_COLOR : OK_COLOR);
  tft.printf("%3.0f%% %s", dutyPct, relayOn ? "ON" : "off");
  if (warnDetail.length() || (ctrlState == CTRL_FAULT && faultDetail.length())) {
    tft.setTextSize(1);
    tft.setCursor(8, 226);
    tft.setTextColor(ERROR_COLOR);
    tft.print(ctrlState == CTRL_FAULT ? faultDetail : warnDetail);
  }
}

void writeRelay(bool on) {
  if (on == relayOn) return;
  relayOn = on;
  digitalWrite(RELAY_PIN, on ? HIGH : LOW);
  Serial.printf("[relay] %s\n", on ? "ON" : "OFF");
  drawRunStatus();
}

void relaysOffNow() {
  dutyPct = 0;
  windowOnMs = 0;
  manualTest = false;
  writeRelay(false);
}

// Slow time-proportioning window. The on-time is latched at the start of
// each window so the relay changes state at most twice per window.
void updateRelayWindow() {
  unsigned long now = millis();
  if (manualTest) {
    writeRelay(true);
    return;
  }
  if (ctrlState != CTRL_RUNNING) { writeRelay(false); return; }
  if (now - windowStartMs >= RELAY_WINDOW_MS) {
    windowStartMs = now;
    unsigned long on = (unsigned long)(dutyPct / 100.0f * RELAY_WINDOW_MS);
    if (on < RELAY_MIN_PULSE_MS) on = 0;
    if (on > RELAY_WINDOW_MS - RELAY_MIN_PULSE_MS) on = RELAY_WINDOW_MS;
    windowOnMs = on;
  }
  writeRelay(now - windowStartMs < windowOnMs);
}

void enterFault(const String& reason) {
  ctrlState = CTRL_FAULT;
  hasFault = true;
  faultDetail = reason;
  relaysOffNow();
  Serial.printf("FAULT: %s\n", reason.c_str());
  drawRunStatus();
}

float pidCompute(float sp, float pv, float dtS) {
  float e = sp - pv;
  float d = isnan(pidLastPv) ? 0 : -PID_KD * (pv - pidLastPv) / dtS;
  pidLastPv = pv;
  float out = PID_KP * e + pidIntegral + d;
  // conditional integration: don't wind further into a saturated output
  if (!((out >= 100 && e > 0) || (out <= 0 && e < 0))) {
    pidIntegral = constrain(pidIntegral + PID_KI * e * dtS, 0.0f, 100.0f);
  }
  return constrain(PID_KP * e + pidIntegral + d, 0.0f, 100.0f);
}

void advanceToStep(int idx, float fromTemp) {
  currentStep = idx;
  stepStartMs = millis();
  stepStartTemp = fromTemp;
  stepWaiting = false;
  if (idx < scheduleStepCount) {
    Serial.printf("[run] step %d/%d: %s from %.0f C\n", idx + 1, scheduleStepCount,
                  schedule_[idx].type, fromTemp);
  }
  drawRunStatus();
}

bool startFiring() {
  if (scheduleStepCount == 0 || ctrlState == CTRL_RUNNING) return false;
  if (hasFault && badReads > 0) return false;    // thermocouple not reading
  manualTest = false;
  pidIntegral = 0;
  pidLastPv = NAN;
  warnDetail = "";
  hasFault = false;
  faultDetail = "";
  scheduleStartMs = millis();
  windowStartMs = millis() - RELAY_WINDOW_MS;    // first window starts now
  ctrlState = CTRL_RUNNING;
  Serial.println("[run] firing STARTED");
  advanceToStep(0, currentTemp);
  return true;
}

void stopFiring() {
  ctrlState = CTRL_IDLE;
  relaysOffNow();
  Serial.println("[run] firing STOPPED");
  drawRunStatus();
}

// Called once per temperature read while RUNNING.
void updateControl(float dtS) {
  if (currentStep >= scheduleStepCount) {
    ctrlState = CTRL_COMPLETE;
    relaysOffNow();
    Serial.println("[run] schedule COMPLETE");
    drawRunStatus();
    return;
  }
  ScheduleStep& st = schedule_[currentStep];
  unsigned long elapsed = millis() - stepStartMs;
  float t = currentTemp;
  bool wasWaiting = stepWaiting;
  warnDetail = "";

  if (strcmp(st.type, "ramp") == 0) {
    bool heating = st.temp >= stepStartTemp;
    if (elapsed < st.durationMs) {
      float frac = (float)elapsed / st.durationMs;
      currentSetpoint = stepStartTemp + frac * (st.temp - stepStartTemp);
    } else {
      currentSetpoint = st.temp;
      bool arrived = heating ? (t >= st.temp - STEP_TOLERANCE_C)
                             : (t <= st.temp + STEP_TOLERANCE_C);
      if (arrived) { advanceToStep(currentStep + 1, st.temp); return; }
      stepWaiting = true;
      if (elapsed - st.durationMs >= STEP_WAIT_TIMEOUT_MS) {
        enterFault("Ramp never reached target");
        return;
      }
    }
    dutyPct = pidCompute(currentSetpoint, t, dtS);

  } else if (strcmp(st.type, "hold") == 0) {
    currentSetpoint = stepStartTemp;
    dutyPct = pidCompute(currentSetpoint, t, dtS);
    if (elapsed >= st.durationMs) { advanceToStep(currentStep + 1, stepStartTemp); return; }

  } else if (strcmp(st.type, "full") == 0) {
    currentSetpoint = st.temp;
    dutyPct = 100;
    pidIntegral = 50;         // bumpless-ish handover to the hold that follows
    if (t >= st.temp - STEP_TOLERANCE_C) { advanceToStep(currentStep + 1, st.temp); return; }
    if (elapsed >= STEP_WAIT_TIMEOUT_MS) { enterFault("Full step timed out"); return; }

  } else if (strcmp(st.type, "cool") == 0) {
    currentSetpoint = st.temp;
    dutyPct = 0;
    pidIntegral = 0;
    if (t <= st.temp + STEP_TOLERANCE_C) { advanceToStep(currentStep + 1, st.temp); return; }

  } else {  // "off"
    currentSetpoint = stepStartTemp;
    dutyPct = 0;
    pidIntegral = 0;
    if (elapsed >= st.durationMs) { advanceToStep(currentStep + 1, stepStartTemp); return; }
  }

  if ((strcmp(st.type, "ramp") == 0 || strcmp(st.type, "hold") == 0) &&
      t > currentSetpoint + OVERSHOOT_WARN_C) {
    warnDetail = "Over setpoint by " + String(t - currentSetpoint, 0) + " C";
  }
  if (stepWaiting != wasWaiting) drawRunStatus();
}

void handleSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r') continue;
    if (ch != '\n') { if (serialLine.length() < 32) serialLine += ch; continue; }
    serialLine.trim();
    serialLine.toLowerCase();
    if (serialLine == "relay on" || serialLine == "ssr on") {
      if (ctrlState == CTRL_RUNNING) Serial.println("[cmd] refused: firing in progress");
      else { manualTest = true; manualSinceMs = millis(); Serial.println("[cmd] manual relay test ON"); }
    } else if (serialLine == "relay off" || serialLine == "ssr off") {
      if (ctrlState == CTRL_RUNNING) Serial.println("[cmd] refused: use stop");
      else { relaysOffNow(); Serial.println("[cmd] manual relay test OFF"); }
    } else if (serialLine == "start") {
      Serial.println(startFiring() ? "[cmd] started" : "[cmd] start refused");
    } else if (serialLine == "stop") {
      stopFiring();
    } else if (serialLine == "status") {
      Serial.printf("[status] %s step %d/%d %s T=%.1f SP=%.1f duty=%.0f%% relay=%s I=%.1f\n",
        stateName(), currentStep + 1, scheduleStepCount,
        currentStep < scheduleStepCount ? schedule_[currentStep].type : "-",
        currentTemp, currentSetpoint, dutyPct, relayOn ? "ON" : "off", pidIntegral);
    } else if (serialLine.length()) {
      Serial.println("[cmd] unknown. Try: relay on | relay off | start | stop | status");
    }
    serialLine = "";
  }
}

String getFaultDetail() {
  uint8_t err = thermocouple.readError();
  if (err & MAX31855_FAULT_OPEN)      return "Open circuit";
  if (err & MAX31855_FAULT_SHORT_GND) return "Short to GND";
  if (err & MAX31855_FAULT_SHORT_VCC) return "Short to VCC";
  return "Unknown (err=0)";
}

// ─── DASHBOARD PAGE (stored in flash) ─────────────────────────────────────────

const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>Kiln Controller</title>
<style>
  :root {
    --bg: #0c0c0d;
    --panel: #17181a;
    --accent: #ffa600;
    --muted: #8a8d93;
    --ok: #3ddc84;
    --error: #ff4d4d;
    --hold: #4d9eff;
    --grid: #2a2c30;
  }
  * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; }
  body {
    margin: 0;
    background: var(--bg);
    color: #eee;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    padding: 12px;
    padding-bottom: 40px;
  }
  h1 {
    font-size: 1rem;
    letter-spacing: 0.06em;
    color: var(--accent);
    margin: 0 0 12px 0;
    text-transform: uppercase;
  }
  h2.section {
    font-size: 0.75rem;
    letter-spacing: 0.06em;
    color: var(--muted);
    text-transform: uppercase;
    margin: 24px 0 8px 0;
  }
  .stats {
    display: flex;
    gap: 10px;
    margin-bottom: 14px;
  }
  .card {
    background: var(--panel);
    border-radius: 12px;
    padding: 12px 16px;
    flex: 1;
  }
  .card .label {
    font-size: 0.7rem;
    color: var(--muted);
    text-transform: uppercase;
    letter-spacing: 0.05em;
    margin-bottom: 2px;
  }
  .card.temp .value {
    font-size: 2.6rem;
    font-weight: 700;
    color: var(--accent);
    line-height: 1.1;
  }
  .card.setpoint .value {
    font-size: 1.6rem;
    font-weight: 600;
    line-height: 1.2;
    margin-top: 8px;
  }
  .fault-banner {
    display: none;
    background: rgba(255,77,77,0.15);
    border: 1px solid var(--error);
    color: var(--error);
    padding: 10px 14px;
    border-radius: 10px;
    margin-bottom: 14px;
    font-weight: 600;
  }
  .graph-wrap {
    background: var(--panel);
    border-radius: 12px;
    padding: 10px;
  }
  canvas { width: 100%; height: 240px; display: block; touch-action: none; }
  .status {
    margin-top: 8px;
    font-size: 0.7rem;
    color: var(--muted);
  }
  .status .dot {
    display: inline-block;
    width: 8px; height: 8px;
    border-radius: 50%;
    background: var(--error);
    margin-right: 5px;
  }
  .status.connected .dot { background: var(--ok); }

  .runbar {
    display: flex;
    gap: 10px;
    margin: 14px 0;
  }
  button {
    -webkit-appearance: none;
    border: none;
    border-radius: 10px;
    padding: 14px 16px;
    font-size: 1rem;
    font-weight: 600;
    min-height: 48px;
    flex: 1;
  }
  button.primary { background: var(--accent); color: #1a1300; }
  button.danger  { background: var(--error); color: #2a0000; }
  button.ghost   { background: var(--panel); color: #eee; }

  .step {
    display: flex;
    align-items: center;
    flex-wrap: wrap;
    gap: 8px;
    background: var(--panel);
    border-radius: 10px;
    padding: 10px 12px;
    margin-bottom: 8px;
    border-left: 4px solid var(--accent);
  }
  .step.hold { border-left-color: var(--hold); }
  .step.full { border-left-color: var(--error); }
  .step.off  { border-left-color: var(--muted); }

  .step select, .step input {
    background: #0c0c0d;
    border: 1px solid #2a2c30;
    color: #eee;
    border-radius: 8px;
    padding: 8px;
    font-size: 0.95rem;
    min-height: 40px;
  }
  .step select { min-width: 110px; }
  .step input[type=number] { width: 64px; }
  .step .unit { font-size: 0.8rem; color: var(--muted); }
  .step .spacer { margin-left: auto; display: flex; gap: 4px; }
  .step button.icon {
    flex: none;
    min-height: 36px;
    min-width: 36px;
    padding: 0;
    background: #0c0c0d;
    color: #eee;
    font-size: 1rem;
  }

  .summary {
    display: flex;
    justify-content: space-between;
    padding: 10px 4px 4px;
    font-size: 0.85rem;
    color: var(--muted);
  }
  .summary b { color: #eee; font-size: 1rem; }
</style>
</head>
<body>
  <h1>Kiln Controller</h1>

  <div class="fault-banner" id="faultBanner"></div>

  <div class="stats">
    <div class="card temp">
      <div class="label">Temperature</div>
      <div class="value" id="tempVal">-- C</div>
    </div>
    <div class="card setpoint">
      <div class="label">Setpoint</div>
      <div class="value" id="setpointVal">-- C</div>
    </div>
  </div>
  <div class="card" style="margin-bottom:14px;">
    <div class="label">State</div>
    <div id="runVal" style="font-size:1.1rem;font-weight:600;color:#fff;">--</div>
  </div>

  <div class="graph-wrap">
    <canvas id="graph"></canvas>
  </div>
  <div class="status" id="connStatus"><span class="dot"></span><span id="connText">connecting...</span></div>

  <h2 class="section">Firing schedule</h2>
  <div id="steps"></div>
  <button class="ghost" id="addStep" style="width:100%;">+ Add step</button>

  <h2 class="section">Rate to time converter</h2>
  <div class="step" style="border-left-color:#5a5d63;">
    <input type="number" id="convFrom" placeholder="now"> <span class="unit">C at</span>
    <input type="number" id="convRate" placeholder="rate"> <span class="unit">C/hr to</span>
    <input type="number" id="convTo" placeholder="target"> <span class="unit">C =</span>
    <b id="convResult" style="margin-left:2px;">--</b>
    <span class="spacer">
      <button class="ghost icon" id="convAdd" style="flex:none;width:auto;min-height:36px;padding:0 10px;">Add step</button>
    </span>
  </div>

  <div class="summary">
    <span>Total: <b id="totalVal">0h 0m</b></span>
    <span>Peak: <b id="peakVal">--C</b></span>
  </div>
  <div class="runbar">
    <button class="ghost" id="saveBtn">Save schedule</button>
    <button class="primary" id="startBtn">Start firing</button>
    <button class="danger" id="stopBtn">Stop</button>
  </div>

<script>
let liveSteps = [];
let liveHistory = [];
let plannedProfile = [];

const typeColor = { ramp: '', hold: 'hold', full: 'full', cool: 'off', off: 'off' };

function fieldsFor(step, i) {
  let html = '';
  if (step.type === 'ramp' || step.type === 'full' || step.type === 'cool') {
    html += `<span class="unit">${step.type==='cool' ? 'down to' : ''}</span><input type="number" value="${step.temp ?? 600}" oninput="liveSteps[${i}].temp=+this.value;renderSteps()"> <span class="unit">C</span>`;
  }
  if (step.type !== 'full' && step.type !== 'cool') {
    html += `<span class="unit">${step.type==='ramp' ? 'over' : 'for'}</span>
      <input type="number" min="0" value="${step.hrs||0}" oninput="liveSteps[${i}].hrs=+this.value;renderSteps()"> <span class="unit">h</span>
      <input type="number" min="0" max="59" value="${step.mins||0}" oninput="liveSteps[${i}].mins=+this.value;renderSteps()"> <span class="unit">m</span>`;
  }
  return html;
}

function renderSteps() {
  document.getElementById('steps').innerHTML = liveSteps.map((s, i) => `
    <div class="step ${typeColor[s.type]}">
      <select onchange="liveSteps[${i}].type=this.value;renderSteps()">
        <option value="ramp" ${s.type==='ramp'?'selected':''}>Ramp</option>
        <option value="hold" ${s.type==='hold'?'selected':''}>Hold</option>
        <option value="full" ${s.type==='full'?'selected':''}>Full power</option>
        <option value="cool" ${s.type==='cool'?'selected':''}>Cool to</option>
        <option value="off"  ${s.type==='off'?'selected':''}>Off</option>
      </select>
      ${fieldsFor(s, i)}
      <span class="spacer">
        <button class="icon" onclick="moveStep(${i},-1)">^</button>
        <button class="icon" onclick="moveStep(${i},1)">v</button>
        <button class="icon" onclick="liveSteps.splice(${i},1);renderSteps()">x</button>
      </span>
    </div>`).join('');
  updateSummary();
  buildPlannedProfile();
  drawGraph();
}

function moveStep(i, dir) {
  const j = i + dir;
  if (j < 0 || j >= liveSteps.length) return;
  [liveSteps[i], liveSteps[j]] = [liveSteps[j], liveSteps[i]];
  renderSteps();
}

function updateSummary() {
  let mins = 0, peak = 0;
  liveSteps.forEach(s => {
    if (s.type !== 'full' && s.type !== 'cool') mins += (s.hrs||0)*60 + (s.mins||0);
    if (s.type === 'ramp' || s.type === 'full') peak = Math.max(peak, s.temp||0);
  });
  document.getElementById('totalVal').textContent = Math.floor(mins/60) + 'h ' + (mins%60) + 'm';
  document.getElementById('peakVal').textContent = peak + 'C';
}

function buildPlannedProfile() {
  plannedProfile = [];
  let tMs = 0, prevTemp = 20;
  plannedProfile.push({t:0, temp:prevTemp});
  liveSteps.forEach(s => {
    if (s.type === 'ramp') {
      tMs += ((s.hrs||0)*3600 + (s.mins||0)*60) * 1000;
      plannedProfile.push({t:tMs, temp:s.temp});
      prevTemp = s.temp;
    } else if (s.type === 'hold') {
      tMs += ((s.hrs||0)*3600 + (s.mins||0)*60) * 1000;
      plannedProfile.push({t:tMs, temp:prevTemp});
    } else if (s.type === 'full') {
      plannedProfile.push({t:tMs, temp:s.temp});
      prevTemp = s.temp;
    } else if (s.type === 'cool') {
      // no fixed duration: plot at a nominal 150 C/h natural cool
      tMs += Math.abs(prevTemp - s.temp) / 150 * 3600 * 1000;
      plannedProfile.push({t:tMs, temp:s.temp});
      prevTemp = s.temp;
    } else if (s.type === 'off') {
      tMs += ((s.hrs||0)*3600 + (s.mins||0)*60) * 1000;
      plannedProfile.push({t:tMs, temp:prevTemp});
    }
  });
}

document.getElementById('addStep').onclick = () => {
  liveSteps.push({type:'ramp', temp:600, hrs:1, mins:0});
  renderSteps();
};

let lastLiveTemp = 20;

function updateConverter() {
  const rate = parseFloat(document.getElementById('convRate').value);
  const to = parseFloat(document.getElementById('convTo').value);
  const fromInput = document.getElementById('convFrom').value;
  const from = fromInput === '' ? lastLiveTemp : parseFloat(fromInput);
  const result = document.getElementById('convResult');
  const btn = document.getElementById('convAdd');
  if (!rate || isNaN(to) || isNaN(from)) {
    result.textContent = '--';
    delete btn.dataset.hrs;
    return;
  }
  const hoursFloat = Math.abs(to - from) / rate;
  const hrs = Math.floor(hoursFloat);
  const mins = Math.round((hoursFloat - hrs) * 60);
  result.textContent = hrs + 'h ' + mins + 'm';
  btn.dataset.hrs = hrs;
  btn.dataset.mins = mins;
  btn.dataset.temp = to;
}
['convFrom','convRate','convTo'].forEach(id =>
  document.getElementById(id).addEventListener('input', updateConverter));

document.getElementById('convAdd').onclick = () => {
  const btn = document.getElementById('convAdd');
  if (btn.dataset.hrs === undefined) return;
  liveSteps.push({type:'ramp', temp:+btn.dataset.temp, hrs:+btn.dataset.hrs, mins:+btn.dataset.mins});
  renderSteps();
};

document.getElementById('saveBtn').onclick = () => {
  fetch('/api/schedule', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify(liveSteps)
  }).then(r => { if (!r.ok) r.text().then(t => alert('Not saved: ' + t)); });
};
document.getElementById('startBtn').onclick = () => {
  if (!confirm('Start firing? The relays will switch.')) return;
  liveHistory = [];
  fetch('/api/start', {method:'POST'}).then(r => r.json()).then(j => { if (!j.ok) alert('Start refused'); });
};
document.getElementById('stopBtn').onclick  = () => fetch('/api/stop', {method:'POST'});

fetch('/api/schedule').then(r => r.json()).then(data => {
  liveSteps = data.length ? data : [{type:'ramp', temp:600, hrs:1, mins:0}];
  renderSteps();
});

const canvas = document.getElementById('graph');
const ctx = canvas.getContext('2d');

function resizeCanvas() {
  const rect = canvas.getBoundingClientRect();
  canvas.width = rect.width * devicePixelRatio;
  canvas.height = rect.height * devicePixelRatio;
  ctx.setTransform(1,0,0,1,0,0);
  ctx.scale(devicePixelRatio, devicePixelRatio);
  drawGraph();
}
window.addEventListener('resize', resizeCanvas);

function drawGraph() {
  const rect = canvas.getBoundingClientRect();
  const w = rect.width, h = rect.height;
  ctx.clearRect(0, 0, w, h);

  const totalMs = plannedProfile.length ? plannedProfile[plannedProfile.length-1].t : 3600000;
  const maxTemp = Math.max(...plannedProfile.map(p => p.temp), 50);

  ctx.strokeStyle = '#2a2c30';
  ctx.fillStyle = '#8a8d93';
  ctx.font = '11px sans-serif';
  ctx.lineWidth = 1;
  const step = maxTemp <= 200 ? 50 : (maxTemp <= 600 ? 100 : 200);
  for (let t = 0; t <= maxTemp; t += step) {
    const y = h - (t/maxTemp)*h;
    ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(w,y); ctx.stroke();
    ctx.fillText(Math.round(t) + 'C', 4, y - 3);
  }

  const xOf = (tMs) => (tMs/totalMs) * w;
  const yOf = (temp) => h - (Math.max(0, Math.min(maxTemp, temp))/maxTemp)*h;

  if (plannedProfile.length > 1) {
    ctx.setLineDash([6,5]);
    ctx.strokeStyle = '#5a5d63';
    ctx.lineWidth = 2;
    ctx.beginPath();
    plannedProfile.forEach((p,i) => i===0 ? ctx.moveTo(xOf(p.t), yOf(p.temp)) : ctx.lineTo(xOf(p.t), yOf(p.temp)));
    ctx.stroke();
    ctx.setLineDash([]);
  }

  if (liveHistory.length > 1) {
    ctx.strokeStyle = '#ffa600';
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    liveHistory.forEach((p,i) => i===0 ? ctx.moveTo(xOf(p.t), yOf(p.temp)) : ctx.lineTo(xOf(p.t), yOf(p.temp)));
    ctx.stroke();
  }
}

function setConnected(connected) {
  document.getElementById('connStatus').classList.toggle('connected', connected);
  document.getElementById('connText').textContent = connected ? 'live' : 'disconnected — retrying...';
}

function connect() {
  const ws = new WebSocket('ws://' + location.host + '/ws');
  ws.onopen = () => setConnected(true);
  ws.onclose = () => { setConnected(false); setTimeout(connect, 2000); };
  ws.onerror = () => ws.close();
  ws.onmessage = (evt) => {
    let data;
    try { data = JSON.parse(evt.data); } catch(e) { return; }

    document.getElementById('setpointVal').textContent = data.setpoint.toFixed(0) + ' C';
    document.getElementById('runVal').textContent =
      data.state + (data.state === 'RUNNING' ? ' | step ' + data.step + '/' + data.steps + ' ' + data.stepType +
      (data.waiting ? ' (waiting)' : '') : '') + ' | ' + data.duty.toFixed(0) + '% | relays ' + (data.relay ? 'ON' : 'off') +
      (data.warn ? ' | ' + data.warn : '');
    lastLiveTemp = data.temp;
    document.getElementById('convFrom').placeholder = data.temp.toFixed(0);

    const faultBanner = document.getElementById('faultBanner');
    if (data.fault) {
      faultBanner.style.display = 'block';
      faultBanner.textContent = 'FAULT: ' + data.faultDetail;
      document.getElementById('tempVal').textContent = '-- C';
      drawGraph();
      return;
    }
    faultBanner.style.display = 'none';
    document.getElementById('tempVal').textContent = data.temp.toFixed(1) + ' C';

    if (data.running) {
      liveHistory.push({t: data.elapsedMs, temp: data.temp});
    }
    drawGraph();
  };
}

resizeCanvas();
connect();
</script>
</body>
</html>
)rawliteral";

// ─── WEBSOCKET / WIFI ───────────────────────────────────────────────────────────

void broadcastState() {
  if (ws.count() == 0) return;
  bool active = ctrlState == CTRL_RUNNING || ctrlState == CTRL_COMPLETE;
  unsigned long elapsedMs = active ? (millis() - scheduleStartMs) : 0;
  char buf[420];
  snprintf(buf, sizeof(buf),
    "{\"temp\":%.2f,\"setpoint\":%.1f,\"fault\":%s,\"faultDetail\":\"%s\","
    "\"running\":%s,\"elapsedMs\":%lu,\"totalMs\":%lu,\"maxTemp\":%.0f,"
    "\"state\":\"%s\",\"step\":%d,\"steps\":%d,\"stepType\":\"%s\",\"waiting\":%s,"
    "\"duty\":%.1f,\"relay\":%s,\"warn\":\"%s\"}",
    currentTemp, currentSetpoint,
    hasFault ? "true" : "false", hasFault ? faultDetail.c_str() : "",
    active ? "true" : "false", elapsedMs, scheduleTotalMs, scheduleMaxTempC,
    stateName(), currentStep + 1, scheduleStepCount,
    currentStep < scheduleStepCount ? schedule_[currentStep].type : "",
    stepWaiting ? "true" : "false",
    dutyPct, relayOn ? "true" : "false", warnDetail.c_str());
  ws.textAll(buf);
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("[ws] client #%u connected\n", client->id());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[ws] client #%u disconnected\n", client->id());
  }
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  unsigned long startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < 15000) {
    Serial.print(".");
    delay(300);
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWiFi connected — IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nWiFi NOT connected — dashboard unavailable, TFT/serial still work.");
  }
}

// ─── SETUP ────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(1500);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);   // relays off before anything else can happen
  Serial.println("=== gen3d kiln controller (S3-Matrix) v" VERSION " ===");
  Serial.println("Schedule-driven kiln control: PID, slow relay window, TFT + web dashboard");
  Serial.println("https://github.com/sui001/kiln-control/tree/main/kiln_matrix_controller");

  tftSPI.begin(TFT_SCK, -1, TFT_MOSI, TFT_CS);
  tft.begin();
  tft.setRotation(3);
  drawStaticLayout();

  maxSPI.begin(MAX_SCK_PIN, MAX_SO_PIN, -1, MAX_CS_PIN);
  if (!thermocouple.begin()) {
    Serial.println("ERROR: MAX31855 not found — check wiring");
  } else {
    Serial.println("MAX31855 OK");
  }

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
  } else {
    loadScheduleFromFS();
  }

  connectWiFi();
  drawNetworkStatus();

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", DASHBOARD_HTML);
  });

  server.on("/api/schedule", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (LittleFS.exists(SCHEDULE_FILE_PATH)) {
      request->send(LittleFS, SCHEDULE_FILE_PATH, "application/json");
    } else {
      request->send(200, "application/json", "[]");
    }
  });

  AsyncCallbackJsonWebHandler* scheduleHandler = new AsyncCallbackJsonWebHandler(
    "/api/schedule",
    [](AsyncWebServerRequest *request, JsonVariant &json) {
      if (ctrlState == CTRL_RUNNING) {
        request->send(409, "text/plain", "stop firing before editing the schedule");
        return;
      }
      if (!json.is<JsonArray>() || json.as<JsonArray>().size() > MAX_SCHEDULE_STEPS) {
        request->send(400, "text/plain", "invalid schedule");
        return;
      }
      File f = LittleFS.open(SCHEDULE_FILE_PATH, "w");
      if (f) {
        serializeJson(json, f);
        f.close();
      }
      loadScheduleFromFS();
      request->send(200, "application/json", "{\"ok\":true}");
    });
  server.addHandler(scheduleHandler);

  server.on("/api/start", HTTP_POST, [](AsyncWebServerRequest *request) {
    bool ok = startFiring();
    request->send(200, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false}");
  });

  server.on("/api/stop", HTTP_POST, [](AsyncWebServerRequest *request) {
    stopFiring();
    request->send(200, "application/json", "{\"ok\":true}");
  });

  server.begin();

  drawRunStatus();

  esp_task_wdt_config_t wdtCfg = { .timeout_ms = WDT_TIMEOUT_MS, .idle_core_mask = 0, .trigger_panic = true };
  esp_task_wdt_reconfigure(&wdtCfg);
  esp_task_wdt_add(NULL);

  Serial.println("Serial commands: relay on | relay off | start | stop | status");
  Serial.println("=== Setup complete ===\n");
}

// ─── LOOP ─────────────────────────────────────────────────────────────────────

void loop() {
  unsigned long now = millis();

  esp_task_wdt_reset();
  handleSerial();
  if (manualTest && millis() - manualSinceMs >= RELAY_TEST_TIMEOUT_MS) {
    Serial.println("[relay] manual test timeout");
    relaysOffNow();
  }

  if (now - lastWifiCheckMs >= WIFI_RECONNECT_CHECK_MS) {
    lastWifiCheckMs = now;
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi dropped — reconnecting...");
      WiFi.reconnect();
    }
    drawNetworkStatus();
    ws.cleanupClients();
  }

  if (now - lastReadMs >= READ_INTERVAL_MS) {
    lastReadMs = now;
    readCount++;

    double reading;
    #if USE_INTERNAL_SENSOR
      reading = thermocouple.readInternal();
    #else
      reading = thermocouple.readCelsius();
      if (!isnan(reading)) reading += CALIBRATION_OFFSET_C;
    #endif

    // A bad read (NaN, or the MAX31855's 0.00 glitch) keeps the last good
    // temperature. Only TC_FAULT_READS in a row counts as a real fault, so
    // one glitch can't abort a 30 h firing.
    bool bad = isnan(reading) || (reading == 0.00 && currentTemp > 1.0);
    if (bad) {
      badReads++;
      if (badReads >= TC_FAULT_READS) {
        String detail = isnan(reading) ? getFaultDetail() : String("Reading stuck at 0.00");
        if (ctrlState == CTRL_RUNNING) {
          enterFault("Thermocouple: " + detail);
        } else if (!hasFault) {
          hasFault = true;
          faultDetail = "Thermocouple: " + detail;
          Serial.printf("FAULT: %s\n", faultDetail.c_str());
        }
      }
    } else {
      badReads = 0;
      if (ctrlState != CTRL_FAULT) { hasFault = false; faultDetail = ""; }
      currentTemp = medianFilter(reading);
    }

    if (currentTemp >= SAFETY_MAX_TEMP && ctrlState != CTRL_FAULT) {
      enterFault("Over-temperature cutoff");
    }
    if (ctrlState == CTRL_RUNNING && badReads < TC_FAULT_READS) {
      updateControl(READ_INTERVAL_MS / 1000.0f);
    }

    Serial.printf("[%s] #%lu raw=%.2f T=%.2f SP=%.1f duty=%.0f%% relay=%s err=0b%s\n",
      stateName(), readCount, reading, currentTemp, currentSetpoint, dutyPct,
      relayOn ? "ON" : "off", String(thermocouple.readError(), BIN).c_str());

    drawStats(currentTemp, currentSetpoint, hasFault);
    if (lastGraphMs == 0 || now - lastGraphMs >= GRAPH_SAMPLE_MS) {
      lastGraphMs = now;
      pushGraphPoint(currentTemp);
      drawGraphLine();
    }

    broadcastState();
  }

  updateRelayWindow();
}

/*
 * FLASH INSTRUCTIONS:
 *   Board: ESP32S3 Dev Module, USB CDC On Boot: Enabled
 *   Libraries: Adafruit_MAX31855, Adafruit_GFX, Adafruit_ILI9341,
 *              ESPAsyncWebServer (ESP32Async), AsyncTCP (ESP32Async),
 *              ArduinoJson (v7+)
 *   LittleFS comes with the ESP32 Arduino core — no separate install.
 *
 *   Fill in WIFI_SSID / WIFI_PASSWORD above before flashing.
 *
 * KNOWN GAPS (deliberately deferred, not forgotten):
 *   - "Start firing" doesn't drive the SSR yet — PID/SSR is still
 *     Step 3 on the roadmap. Right now it only starts the elapsed-
 *     time tracking that feeds the dashboard graph.
 *   - Run-state (which step, elapsed-within-step) isn't persisted
 *     across a power cut — only the schedule definition is.
 *   - "Off" steps' planned-line assumes flat temp (no real cooldown
 *     model) — fine as a rough guide, not physically accurate.
 *
 * NEXT STEP: PID loop + SSR control on GP39, wired to actually chase
 * the schedule rather than just visualizing it.
 */
