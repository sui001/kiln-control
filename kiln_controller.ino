/*
 * PROJECT: Kiln Controller
 * DEVICE:  ESP32-S3 SuperMini — primary controller
 * VERSION: 2.0 (merged)
 *
 * Merge of two earlier prototypes:
 *   - kiln_controller.ino (v1.x)         -> PID loop + SSR PWM windowing,
 *                                            safety cutoffs, live dashboard.
 *   - kiln_sensor_display_v1_5.ino       -> step-based schedule (ramp/hold/
 *     (built for a Waveshare ESP32-S3-      full/off), LittleFS persistence,
 *      Matrix + TFT, now retired)           mobile dashboard UI, rate-to-time
 *                                            converter. TFT/display code
 *                                            dropped — SuperMini has no
 *                                            screen, dashboard is the UI.
 *
 * v2.0 adds actual PID/SSR control to the step-based schedule (the v1.5
 * sketch tracked elapsed time for the dashboard graph but never drove the
 * SSR). "full" steps run the SSR at 100% duty (bypassing PID) until the
 * reading gets within FULL_STEP_TOLERANCE_C of target, with a timeout
 * safety fault if that never happens. "ramp"/"hold"/"off" are elapsed-time
 * driven, same as before.
 *
 * WIRING (ESP32-S3 SuperMini):
 *   MAX31855 (software SPI, bit-banged — any GPIO works):
 *     VCC  -> 3.3V
 *     GND  -> GND
 *     SCK  -> GPIO12
 *     CS   -> GPIO10
 *     SO   -> GPIO13
 *   SSR control input -> GPIO9 (via 220ohm resistor recommended)
 *   SSR load side -> 240V kiln element (respect mains safety!)
 *
 * LIBRARIES REQUIRED (Arduino Library Manager):
 *   - Adafruit MAX31855 library
 *   - ArduinoJson (v7+ — uses the JsonDocument API)
 *   - PID_v1 (Brett Beauregard)
 *   - LittleFS (bundled with the ESP32 Arduino core)
 *   - ESPAsyncWebServer (by ESP32Async) — install from GitHub
 *   - AsyncTCP (by ESP32Async) — install from GitHub
 *
 * BEFORE FLASHING: fill in WIFI_SSID / WIFI_PASSWORD below.
 *   Do NOT commit real credentials if this repo is public — use
 *   placeholders in anything pushed, keep real values local only.
 *
 * BOARD: ESP32S3 Dev Module (or SuperMini variant), USB CDC On Boot: Enabled
 */

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncJson.h>
#include <Adafruit_MAX31855.h>
#include <ArduinoJson.h>
#include <PID_v1.h>
#include <LittleFS.h>
#include <SPI.h>

// ─── CONFIGURATION ────────────────────────────────────────────────────────────

#define VERSION "2.0"

// >>> FILL THESE IN BEFORE FLASHING — do not commit real values <<<
const char* WIFI_SSID     = "YOUR_SSID";
const char* WIFI_PASSWORD = "YOUR_PASSWORD";

// Pins
#define SSR_PIN         9     // GPIO driving SSR control input
#define MAX_SCK_PIN     12    // SPI clock
#define MAX_CS_PIN      10    // MAX31855 chip select
#define MAX_SO_PIN      13    // SPI MISO (data out from MAX31855)

// PID tuning — you will likely need to tune these for your kiln mass
#define PID_KP          2.0
#define PID_KI          0.005
#define PID_KD          1.0

// SSR PWM window (ms) — longer = smoother power, shorter = more responsive
#define WINDOW_SIZE_MS  5000

// How often to sample temp and update control (ms)
#define SAMPLE_INTERVAL_MS  1000

// How often to check/retry WiFi if it drops (ms)
#define WIFI_RECONNECT_CHECK_MS  10000

// Safety cutoff — absolute max temp before emergency stop (°C)
#define SAFETY_MAX_TEMP 1300.0

// "full" steps run the SSR at 100% until within this many °C of target
#define FULL_STEP_TOLERANCE_C   3.0
// Safety: if a "full" step never gets there, fault out rather than hang
#define FULL_STEP_TIMEOUT_MS    (4UL * 3600UL * 1000UL)  // 4 hours

#define MAX_SCHEDULE_STEPS   30
#define SCHEDULE_FILE_PATH   "/schedule.json"

// ─── GLOBALS ──────────────────────────────────────────────────────────────────

Adafruit_MAX31855 thermocouple(MAX_SCK_PIN, MAX_CS_PIN, MAX_SO_PIN);
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// PID
double pidInput    = 0;   // current temp
double pidOutput   = 0;   // 0..WINDOW_SIZE_MS on-time
double pidSetpoint = 0;   // target temp at this moment
PID myPID(&pidInput, &pidOutput, &pidSetpoint, PID_KP, PID_KI, PID_KD, DIRECT);

// SSR PWM window
unsigned long windowStartMs = 0;
bool ssrOn = false;

// Sensor state
float currentTemp     = 0;
float currentSetpoint = 0;
bool hasFault          = false;
String faultDetail     = "";
unsigned long lastReadMs      = 0;
unsigned long lastWifiCheckMs = 0;

// ─── SCHEDULE (RAM copy — source of truth is /schedule.json) ─────────────────

struct ScheduleStep {
  char type[6];        // "ramp", "hold", "full", "off"
  float temp;          // target temp, degC — used by ramp/full, ignored otherwise
  uint32_t durationMs; // used by ramp/hold/off — ignored for full
};

ScheduleStep schedule_[MAX_SCHEDULE_STEPS];
int scheduleStepCount    = 0;
uint32_t scheduleTotalMs = 0;
float scheduleMaxTempC   = 0;

enum ControllerState { CTRL_IDLE, CTRL_RUNNING, CTRL_COMPLETE, CTRL_FAULT };
ControllerState controllerState = CTRL_IDLE;

unsigned long scheduleStartMs = 0;  // whole-program start, for dashboard graph
int currentStepIndex          = 0;
unsigned long stepStartMs     = 0;
float stepStartTemp           = 20.0;

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

// ─── FAULT / SAFETY ───────────────────────────────────────────────────────────

String getFaultDetail() {
  uint8_t err = thermocouple.readError();
  if (err & MAX31855_FAULT_OPEN)      return "Open circuit";
  if (err & MAX31855_FAULT_SHORT_GND) return "Short to GND";
  if (err & MAX31855_FAULT_SHORT_VCC) return "Short to VCC";
  return "Unknown (err=0)";
}

void ssrOff() {
  ssrOn = false;
  digitalWrite(SSR_PIN, LOW);
}

void enterFault(const String& reason) {
  controllerState = CTRL_FAULT;
  hasFault    = true;
  faultDetail = reason;
  ssrOff();
  myPID.SetMode(MANUAL);
  pidOutput = 0;
  Serial.printf("FAULT: %s\n", reason.c_str());
}

// ─── SSR PWM WINDOW ───────────────────────────────────────────────────────────

void updateSSR() {
  unsigned long now = millis();
  if (now - windowStartMs >= WINDOW_SIZE_MS) {
    windowStartMs = now;
  }
  bool shouldBeOn = (pidOutput > (now - windowStartMs));
  if (shouldBeOn != ssrOn) {
    ssrOn = shouldBeOn;
    digitalWrite(SSR_PIN, ssrOn ? HIGH : LOW);
  }
}

// ─── SCHEDULE CONTROL STATE MACHINE ───────────────────────────────────────────

void advanceToStep(int idx, float atTemp) {
  currentStepIndex = idx;
  stepStartMs = millis();
  stepStartTemp = atTemp;
}

bool startFiring() {
  if (scheduleStepCount == 0 || controllerState == CTRL_RUNNING) return false;
  advanceToStep(0, currentTemp);
  scheduleStartMs  = millis();
  controllerState  = CTRL_RUNNING;
  hasFault         = false;
  myPID.SetMode(AUTOMATIC);
  Serial.println("Firing started");
  return true;
}

void stopFiring() {
  controllerState = CTRL_IDLE;
  ssrOff();
  myPID.SetMode(MANUAL);
  pidOutput = 0;
  Serial.println("Firing stopped");
}

void updateControlLoop() {
  if (controllerState != CTRL_RUNNING) return;

  if (currentStepIndex >= scheduleStepCount) {
    controllerState = CTRL_COMPLETE;
    ssrOff();
    myPID.SetMode(MANUAL);
    Serial.println("Firing schedule complete");
    return;
  }

  ScheduleStep& step = schedule_[currentStepIndex];
  unsigned long elapsed = millis() - stepStartMs;

  if (strcmp(step.type, "ramp") == 0) {
    float frac = step.durationMs > 0 ? constrain((float)elapsed / step.durationMs, 0.0f, 1.0f) : 1.0f;
    currentSetpoint = stepStartTemp + frac * (step.temp - stepStartTemp);
    pidSetpoint = currentSetpoint;
    myPID.Compute();
    if (elapsed >= step.durationMs) advanceToStep(currentStepIndex + 1, step.temp);

  } else if (strcmp(step.type, "hold") == 0) {
    currentSetpoint = stepStartTemp;
    pidSetpoint = currentSetpoint;
    myPID.Compute();
    if (elapsed >= step.durationMs) advanceToStep(currentStepIndex + 1, stepStartTemp);

  } else if (strcmp(step.type, "full") == 0) {
    currentSetpoint = step.temp;
    pidOutput = WINDOW_SIZE_MS;  // bypass PID — SSR full on
    if (currentTemp >= step.temp - FULL_STEP_TOLERANCE_C) {
      advanceToStep(currentStepIndex + 1, step.temp);
    } else if (elapsed >= FULL_STEP_TIMEOUT_MS) {
      enterFault("Full-power step timed out before reaching target");
    }

  } else {  // "off"
    currentSetpoint = stepStartTemp;
    pidOutput = 0;
    ssrOff();
    if (elapsed >= step.durationMs) advanceToStep(currentStepIndex + 1, stepStartTemp);
  }
}

// ─── WEBSOCKET BROADCAST ──────────────────────────────────────────────────────

const char* stateName() {
  switch (controllerState) {
    case CTRL_IDLE:     return "IDLE";
    case CTRL_RUNNING:  return "RUNNING";
    case CTRL_COMPLETE: return "COMPLETE";
    default:            return "FAULT";
  }
}

void broadcastState() {
  if (ws.count() == 0) return;

  unsigned long elapsedMs = (controllerState == CTRL_RUNNING || controllerState == CTRL_COMPLETE)
                             ? (millis() - scheduleStartMs) : 0;

  char buf[300];
  snprintf(buf, sizeof(buf),
    "{\"temp\":%.2f,\"setpoint\":%.1f,\"fault\":%s,\"faultDetail\":\"%s\","
    "\"state\":\"%s\",\"ssr\":%s,\"elapsedMs\":%lu,\"totalMs\":%lu,\"maxTemp\":%.0f}",
    currentTemp, currentSetpoint,
    hasFault ? "true" : "false", hasFault ? faultDetail.c_str() : "",
    stateName(), ssrOn ? "true" : "false",
    elapsedMs, scheduleTotalMs, scheduleMaxTempC);

  ws.textAll(buf);
}

void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("[ws] client #%u connected\n", client->id());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[ws] client #%u disconnected\n", client->id());
  }
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
    display: flex;
    align-items: center;
    gap: 10px;
  }
  .badge {
    font-size: 0.65rem;
    padding: 3px 8px;
    border-radius: 4px;
    letter-spacing: 0.04em;
    background: #333;
    color: var(--muted);
  }
  .badge.RUNNING  { background: var(--accent); color: #1a1300; }
  .badge.COMPLETE { background: var(--ok); color: #06210f; }
  .badge.FAULT    { background: var(--error); color: #2a0000; }
  .ssr-dot {
    width: 9px; height: 9px; border-radius: 50%;
    background: #333; margin-left: auto;
  }
  .ssr-dot.on { background: var(--error); box-shadow: 0 0 6px var(--error); }
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
  button:disabled { opacity: 0.4; }

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
  <h1>Kiln Controller <span class="badge" id="stateBadge">IDLE</span> <span class="ssr-dot" id="ssrDot" title="SSR"></span></h1>

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
let running = false;

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

    const badge = document.getElementById('stateBadge');
    badge.textContent = data.state;
    badge.className = 'badge ' + data.state;
    document.getElementById('ssrDot').className = 'ssr-dot' + (data.ssr ? ' on' : '');
    running = (data.state === 'RUNNING');
    document.getElementById('startBtn').disabled = running;
    document.getElementById('saveBtn').disabled = running;

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

    if (running) {
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

// ─── HTTP / WIFI ──────────────────────────────────────────────────────────────

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
    Serial.println("\nWiFi NOT connected — dashboard unavailable, retrying in background.");
  }
}

// ─── SETUP ────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Kiln Controller starting ===");
  Serial.printf("Version: %s\n", VERSION);

  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, LOW);

  if (!thermocouple.begin()) {
    Serial.println("ERROR: MAX31855 not found. Check wiring.");
  } else {
    Serial.println("MAX31855 OK");
  }

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
  } else {
    loadScheduleFromFS();
  }

  connectWiFi();

  // PID setup
  myPID.SetOutputLimits(0, WINDOW_SIZE_MS);
  myPID.SetSampleTime(SAMPLE_INTERVAL_MS);
  myPID.SetMode(MANUAL);
  pidOutput = 0;

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", DASHBOARD_HTML);
  });

  server.on("/api/schedule", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (LittleFS.exists(SCHEDULE_FILE_PATH)) {
      request->send(LittleFS, SCHEDULE_FILE_PATH, "application/json");
    } else {
      request->send(200, "application/json", "[]");
    }
  });

  AsyncCallbackJsonWebHandler* scheduleHandler = new AsyncCallbackJsonWebHandler(
    "/api/schedule",
    [](AsyncWebServerRequest* request, JsonVariant& json) {
      if (controllerState == CTRL_RUNNING) {
        request->send(409, "text/plain", "stop firing before editing schedule");
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

  server.on("/api/start", HTTP_POST, [](AsyncWebServerRequest* request) {
    bool ok = startFiring();
    request->send(200, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false}");
  });

  server.on("/api/stop", HTTP_POST, [](AsyncWebServerRequest* request) {
    stopFiring();
    request->send(200, "application/json", "{\"ok\":true}");
  });

  server.begin();
  Serial.println("=== Setup complete ===\n");
}

// ─── LOOP ─────────────────────────────────────────────────────────────────────

void loop() {
  ws.cleanupClients();

  unsigned long now = millis();

  if (now - lastWifiCheckMs >= WIFI_RECONNECT_CHECK_MS) {
    lastWifiCheckMs = now;
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi dropped — reconnecting...");
      WiFi.reconnect();
    }
  }

  if (now - lastReadMs >= SAMPLE_INTERVAL_MS) {
    lastReadMs = now;

    double reading = thermocouple.readCelsius();

    if (isnan(reading)) {
      String detail = getFaultDetail();
      if (controllerState == CTRL_RUNNING) {
        enterFault("Thermocouple fault: " + detail);
      } else {
        hasFault = true;
        faultDetail = detail;
      }
    } else {
      hasFault = false;
      currentTemp = reading;
      pidInput = currentTemp;

      if (currentTemp >= SAFETY_MAX_TEMP && controllerState == CTRL_RUNNING) {
        enterFault("Over-temperature safety cutoff");
      } else {
        updateControlLoop();
      }
    }

    broadcastState();

    Serial.printf("[%s] Temp: %.1fC  SP: %.1fC  SSR: %s\n",
      stateName(), currentTemp, currentSetpoint, ssrOn ? "ON" : "off");
  }

  if (controllerState == CTRL_RUNNING) {
    updateSSR();
  }
}

/*
 * FLASH INSTRUCTIONS:
 *   Board: "ESP32S3 Dev Module" (or search for your SuperMini variant)
 *   USB CDC On Boot: Enabled
 *   Upload Speed: 921600
 *   Flash Mode: QIO 80MHz
 *
 *   Install libraries via Library Manager:
 *     - Adafruit MAX31855 library
 *     - ArduinoJson (v7+)
 *     - PID (Brett Beauregard)
 *     - ESPAsyncWebServer (ESP32Async) — search or install from GitHub
 *     - AsyncTCP (ESP32Async) — install from GitHub
 *
 *   After flashing: open Serial Monitor at 115200 baud.
 *   Note the IP address printed, open in browser.
 *
 * SAFETY NOTE:
 *   240V mains wiring must be done properly.
 *   SSR load terminals to kiln element only.
 *   SSR control side is 3-32VDC input — GPIO via 220 ohm resistor is fine.
 *   Never touch SSR load side when powered.
 */
