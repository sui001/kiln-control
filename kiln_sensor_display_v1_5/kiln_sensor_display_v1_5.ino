/*
 * PROJECT: Kiln Controller
 * DEVICE:  Sensor + Display + WiFi dashboard + schedule (merged) — Waveshare ESP32-S3-Matrix
 * VERSION: 1.4
 *
 * v1.4: Firing schedule editor + full-program live graph.
 *
 *   - Schedule (array of steps: ramp / hold / full / off) saved to
 *     LittleFS as /schedule.json, survives reboot. Edited from the
 *     browser dashboard, not the TFT.
 *   - "Start firing" sets a start timestamp and begins tracking
 *     elapsed time against the schedule. NOTE: this does NOT yet
 *     drive the SSR — PID + SSR control is still a separate step
 *     (Step 3, per the project roadmap). Right now "running" only
 *     means "the dashboard is tracking elapsed time / drawing the
 *     planned-vs-actual graph" — it's scaffolding for when the PID
 *     loop lands, not actual kiln control yet. Don't mistake this
 *     for the kiln actually firing.
 *   - Dashboard graph now spans the WHOLE program (not a rolling
 *     60s window) and auto-scales its Y axis to the schedule's peak
 *     temperature, with a dashed "planned" line and a solid "actual"
 *     line layered on the same canvas.
 *   - Run-state (current step / elapsed-within-step) is NOT persisted
 *     across a power cut yet — only the schedule definition is. A
 *     power blip mid-firing currently loses run position. Flagged
 *     as a known gap, not solved in this version.
 *   - Dashboard CSS reworked for mobile: bigger tap targets, bigger
 *     temp readout, layout that stacks cleanly on a phone screen.
 *
 * WIRING — unchanged from v1.3a:
 *   MAX31855: SCK->GP36  CS->GP37  SO->GP38  (hardware SPI, FSPI)
 *     *** 10nF ceramic cap across T+/T- screw terminal — load-bearing ***
 *   ILI9341:  CS->GP1  RST->GP2  DC->GP3  MOSI->GP4  SCK->GP5  (hardware SPI, HSPI)
 *   SSR (reserved, not wired yet): GP39
 *
 * LIBRARIES REQUIRED:
 *   - Adafruit_MAX31855, Adafruit_GFX, Adafruit_ILI9341
 *   - ESPAsyncWebServer (by ESP32Async)
 *   - AsyncTCP (by ESP32Async)
 *   - ArduinoJson (v7+ — uses the newer JsonDocument API, not the old StaticJsonDocument<N>)
 *   - LittleFS (bundled with the ESP32 Arduino core, no separate install)
 *
 * BEFORE FLASHING: fill in WIFI_SSID / WIFI_PASSWORD below.
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

// ─── CONFIG ───────────────────────────────────────────────────────────────────

#define VERSION         "1.5"
/*
 * v1.5 changes (see top of file for full v1.4 history):
 *   - Bug fix: graph stopped reflecting edits until you hit Save.
 *     It was scaling axes off the server's last-broadcast totals,
 *     which only update after a save. Now computed straight from
 *     the in-browser schedule on every edit.
 *   - Reorder arrow icons were near-invisible (dark on dark) — now
 *     explicit white.
 *   - Added a rate-to-time converter on the dashboard (rate C/hr +
 *     target, optional "from" — defaults to live temp — outputs a
 *     ready-to-insert ramp step).
 *   - Ramp/soak deviation alarm still deferred (needs PID to mean
 *     anything) — when it lands, default threshold should be
 *     deliberately loose, not tight. Noted, not built.
 */

// >>> FILL THESE IN BEFORE FLASHING <<<
#define WIFI_SSID       "Telstra56C415"
#define WIFI_PASSWORD   "q2b9q3gvn9"

#define MAX_SCK_PIN     36
#define MAX_SO_PIN      38
#define MAX_CS_PIN      37

#define TFT_CS          1
#define TFT_RST         2
#define TFT_DC          3
#define TFT_MOSI        4
#define TFT_SCK         5

#define SSR_PIN_RESERVED 39   // not used yet — reserved for Step 3

#define READ_INTERVAL_MS    500
#define GRAPH_HISTORY_LEN   120   // TFT's own rolling graph — unchanged, still ~60s window
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

float graphHistory[GRAPH_HISTORY_LEN];
int graphIndex   = 0;
int graphCount   = 0;

#define GRAPH_X       10
#define GRAPH_Y       95
#define GRAPH_W       300
#define GRAPH_H       100
#define GRAPH_MIN_C   0
#define GRAPH_MAX_C   60

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

bool scheduleRunning      = false;
unsigned long scheduleStartMs = 0;

void recomputeScheduleStats() {
  scheduleTotalMs = 0;
  scheduleMaxTempC = 0;
  for (int i = 0; i < scheduleStepCount; i++) {
    if (strcmp(schedule_[i].type, "full") != 0) {
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
  float clamped = constrain(temp, GRAPH_MIN_C, GRAPH_MAX_C);
  float frac = (clamped - GRAPH_MIN_C) / (float)(GRAPH_MAX_C - GRAPH_MIN_C);
  return GRAPH_Y + GRAPH_H - (int)(frac * GRAPH_H);
}

void drawGraphFrame() {
  tft.drawRect(GRAPH_X, GRAPH_Y, GRAPH_W, GRAPH_H, GRID_COLOR);
  int step = (GRAPH_MAX_C <= 100) ? 20 : 100;
  for (int t = GRAPH_MIN_C; t <= GRAPH_MAX_C; t += step) {
    int y = tempToY(t);
    tft.drawFastHLine(GRAPH_X, y, GRAPH_W, GRID_COLOR);
    tft.setTextColor(MUTED_COLOR);
    tft.setTextSize(1);
    tft.setCursor(GRAPH_X + GRAPH_W + 4, y - 3);
    tft.print(t);
  }
}

void drawGraphLine() {
  tft.fillRect(GRAPH_X + 1, GRAPH_Y + 1, GRAPH_W - 2, GRAPH_H - 2, BG_COLOR);
  int step = (GRAPH_MAX_C <= 100) ? 20 : 100;
  for (int t = GRAPH_MIN_C; t <= GRAPH_MAX_C; t += step) {
    int y = tempToY(t);
    tft.drawFastHLine(GRAPH_X, y, GRAPH_W, GRID_COLOR);
  }
  if (graphCount < 2) return;
  int pointsToPlot = graphCount;
  int startIdx = (graphIndex - graphCount + GRAPH_HISTORY_LEN) % GRAPH_HISTORY_LEN;
  for (int i = 0; i < pointsToPlot - 1; i++) {
    int idxA = (startIdx + i) % GRAPH_HISTORY_LEN;
    int idxB = (startIdx + i + 1) % GRAPH_HISTORY_LEN;
    int xA = GRAPH_X + (int)((float)i / (GRAPH_HISTORY_LEN - 1) * GRAPH_W);
    int xB = GRAPH_X + (int)((float)(i + 1) / (GRAPH_HISTORY_LEN - 1) * GRAPH_W);
    int yA = tempToY(graphHistory[idxA]);
    int yB = tempToY(graphHistory[idxB]);
    tft.drawLine(xA, yA, xB, yB, GRAPH_COLOR);
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

const typeColor = { ramp: '', hold: 'hold', full: 'full', off: 'off' };

function fieldsFor(step, i) {
  let html = '';
  if (step.type === 'ramp' || step.type === 'full') {
    html += `<input type="number" value="${step.temp ?? 600}" oninput="liveSteps[${i}].temp=+this.value;renderSteps()"> <span class="unit">C</span>`;
  }
  if (step.type !== 'full') {
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
    if (s.type !== 'full') mins += (s.hrs||0)*60 + (s.mins||0);
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
  });
};
document.getElementById('startBtn').onclick = () => fetch('/api/start', {method:'POST'});
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

  unsigned long elapsedMs = scheduleRunning ? (millis() - scheduleStartMs) : 0;

  char buf[220];
  snprintf(buf, sizeof(buf),
    "{\"temp\":%.2f,\"internal\":%.2f,\"setpoint\":%.1f,\"fault\":%s,\"faultDetail\":\"%s\","
    "\"running\":%s,\"elapsedMs\":%lu,\"totalMs\":%lu,\"maxTemp\":%.0f}",
    currentTemp,
    thermocouple.readInternal(),
    currentSetpoint,
    hasFault ? "true" : "false",
    hasFault ? faultDetail.c_str() : "",
    scheduleRunning ? "true" : "false",
    elapsedMs,
    scheduleTotalMs,
    scheduleMaxTempC);

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
  Serial.println("\n=== Kiln sensor + display + dashboard + schedule (merged) ===");
  Serial.printf("Version: %s\n", VERSION);

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
    scheduleRunning = true;
    scheduleStartMs = millis();
    Serial.println("Schedule run STARTED (tracking only — no SSR/PID yet)");
    request->send(200, "application/json", "{\"ok\":true}");
  });

  server.on("/api/stop", HTTP_POST, [](AsyncWebServerRequest *request) {
    scheduleRunning = false;
    Serial.println("Schedule run STOPPED");
    request->send(200, "application/json", "{\"ok\":true}");
  });

  server.begin();

  Serial.println("=== Setup complete ===\n");
}

// ─── LOOP ─────────────────────────────────────────────────────────────────────

void loop() {
  unsigned long now = millis();

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

    Serial.printf("  [raw] internal=%.2f  external=%.2f  errReg=0b%s\n",
      thermocouple.readInternal(),
      thermocouple.readCelsius(),
      String(thermocouple.readError(), BIN).c_str());

    if (isnan(reading)) {
      hasFault = true;
      faultDetail = getFaultDetail();
      Serial.printf("FAULT: %s\n", faultDetail.c_str());
    } else if (reading == 0.00 && currentTemp > 1.0) {
      Serial.printf("  [glitch] rejected 0.00 read, keeping last good %.2f C\n", currentTemp);
      hasFault = false;
      pushGraphPoint(currentTemp);
    } else {
      hasFault = false;
      currentTemp = reading;
      pushGraphPoint(currentTemp);
      Serial.printf("Read #%lu: %.2f C\n", readCount, currentTemp);
    }

    drawStats(currentTemp, currentSetpoint, hasFault);
    if (!hasFault) drawGraphLine();

    broadcastState();
  }
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
