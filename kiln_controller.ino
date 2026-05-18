/*
 * PROJECT: Kiln Controller
 * DEVICE:  ESP32-S3 SuperMini — primary controller
 *
 * PID-based kiln controller with ramp/hold firing schedules.
 * Reads temperature via MAX31855 (K-type thermocouple, SPI).
 * Controls a 40A SSR on 240V. Serves a browser dashboard over
 * WiFi with live temperature graphing and editable firing schedule.
 *
 * CONFIGURATION:
 *   Set WIFI_SSID and WIFI_PASSWORD below.
 *   Set SSR_PIN to whichever GPIO drives your SSR control input.
 *   SPI pins default to ESP32-S3 SuperMini hardware SPI.
 *
 * WIRING:
 *   MAX31855:
 *     VCC  -> 3.3V
 *     GND  -> GND
 *     SCK  -> GPIO12
 *     CS   -> GPIO10
 *     SO   -> GPIO13
 *   SSR control input -> GPIO9 (via 220ohm resistor recommended)
 *   SSR load side -> 240V kiln element (respect mains safety!)
 *
 * LIBRARIES REQUIRED (install via Arduino Library Manager):
 *   - Adafruit MAX31855 library
 *   - ESP32 AsyncWebServer (me-no-dev/ESPAsyncWebServer)
 *   - AsyncTCP (me-no-dev/AsyncTCP)
 *   - ArduinoJson
 *   - PID_v1 (Brett Beauregard)
 *
 * BOARD: ESP32S3 Dev Module (or SuperMini variant)
 *   USB Mode: CDC (for Serial monitor)
 */

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_MAX31855.h>
#include <ArduinoJson.h>
#include <PID_v1.h>
#include <SPI.h>

// ─── CONFIGURATION ────────────────────────────────────────────────────────────

const char* WIFI_SSID     = "YOUR_SSID";
const char* WIFI_PASSWORD = "YOUR_PASSWORD";

// Pins
#define SSR_PIN         9     // GPIO driving SSR control input
#define MAX_CS_PIN      10    // MAX31855 chip select
#define MAX_SCK_PIN     12    // SPI clock
#define MAX_SO_PIN      13    // SPI MISO (data out from MAX31855)

// PID tuning — you will likely need to tune these for your kiln mass
#define PID_KP          2.0
#define PID_KI          0.005
#define PID_KD          1.0

// SSR PWM window (ms) — longer = smoother power, shorter = more responsive
#define WINDOW_SIZE_MS  5000

// How often to sample temp and update PID (ms)
#define SAMPLE_INTERVAL_MS  1000

// Max schedule waypoints
#define MAX_WAYPOINTS   20

// Safety cutoff — absolute max temp before emergency stop (°C)
#define SAFETY_MAX_TEMP 1300.0

// ─── GLOBALS ──────────────────────────────────────────────────────────────────

Adafruit_MAX31855 thermocouple(MAX_SCK_PIN, MAX_CS_PIN, MAX_SO_PIN);
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// PID
double pidInput    = 0;   // current temp
double pidOutput   = 0;   // 0-WINDOW_SIZE_MS on-time
double pidSetpoint = 0;   // target temp at this moment
PID myPID(&pidInput, &pidOutput, &pidSetpoint, PID_KP, PID_KI, PID_KD, DIRECT);

// Schedule: waypoints as {elapsed minutes, target °C}
struct Waypoint {
  float minutes;
  float tempC;
};

Waypoint schedule[MAX_WAYPOINTS];
int waypointCount = 0;

// Controller state
enum State { IDLE, RUNNING, COMPLETE, ERROR_STATE };
State controllerState = IDLE;

unsigned long scheduleStartMs  = 0;
unsigned long windowStartMs    = 0;
unsigned long lastSampleMs     = 0;
float currentTemp              = 0;
float currentSetpoint          = 0;
bool ssrOn                     = false;
String errorMessage            = "";

// ─── SCHEDULE INTERPOLATION ───────────────────────────────────────────────────

float getSetpointAtMinute(float elapsedMinutes) {
  if (waypointCount == 0) return 0;
  if (elapsedMinutes <= schedule[0].minutes) return schedule[0].tempC;

  for (int i = 1; i < waypointCount; i++) {
    if (elapsedMinutes <= schedule[i].minutes) {
      float t = (elapsedMinutes - schedule[i-1].minutes) /
                (schedule[i].minutes - schedule[i-1].minutes);
      return schedule[i-1].tempC + t * (schedule[i].tempC - schedule[i-1].tempC);
    }
  }
  return schedule[waypointCount - 1].tempC;
}

float scheduleTotalMinutes() {
  if (waypointCount == 0) return 0;
  return schedule[waypointCount - 1].minutes;
}

// ─── SSR CONTROL ──────────────────────────────────────────────────────────────

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

void ssrOff() {
  ssrOn = false;
  digitalWrite(SSR_PIN, LOW);
}

// ─── WEBSOCKET BROADCAST ──────────────────────────────────────────────────────

void broadcastStatus() {
  StaticJsonDocument<512> doc;
  float elapsed = 0;
  if (controllerState == RUNNING) {
    elapsed = (millis() - scheduleStartMs) / 60000.0;
  }
  doc["state"]      = (controllerState == IDLE)     ? "IDLE" :
                      (controllerState == RUNNING)   ? "RUNNING" :
                      (controllerState == COMPLETE)  ? "COMPLETE" : "ERROR";
  doc["temp"]       = currentTemp;
  doc["setpoint"]   = currentSetpoint;
  doc["ssr"]        = ssrOn;
  doc["elapsed"]    = elapsed;
  doc["totalMins"]  = scheduleTotalMinutes();
  doc["error"]      = errorMessage;

  String json;
  serializeJson(doc, json);
  ws.textAll(json);
}

// ─── WEBSOCKET HANDLER ────────────────────────────────────────────────────────

void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type != WS_EVT_DATA) return;

  AwsFrameInfo* info = (AwsFrameInfo*)arg;
  if (info->opcode != WS_TEXT) return;

  data[len] = 0;
  StaticJsonDocument<1024> doc;
  if (deserializeJson(doc, (char*)data)) return;

  String cmd = doc["cmd"] | "";

  if (cmd == "start") {
    if (waypointCount >= 2 && controllerState != RUNNING) {
      scheduleStartMs   = millis();
      windowStartMs     = millis();
      controllerState   = RUNNING;
      myPID.SetMode(AUTOMATIC);
      Serial.println("Schedule started");
    }

  } else if (cmd == "stop") {
    controllerState = IDLE;
    ssrOff();
    myPID.SetMode(MANUAL);
    pidOutput = 0;
    Serial.println("Stopped");

  } else if (cmd == "set_schedule") {
    if (controllerState == RUNNING) return; // don't change mid-fire
    JsonArray pts = doc["waypoints"].as<JsonArray>();
    waypointCount = 0;
    for (JsonObject pt : pts) {
      if (waypointCount >= MAX_WAYPOINTS) break;
      schedule[waypointCount].minutes = pt["m"];
      schedule[waypointCount].tempC   = pt["t"];
      waypointCount++;
    }
    Serial.printf("Schedule loaded: %d waypoints\n", waypointCount);

  } else if (cmd == "get_schedule") {
    // Send schedule back to requester
    StaticJsonDocument<1024> resp;
    resp["cmd"] = "schedule";
    JsonArray arr = resp.createNestedArray("waypoints");
    for (int i = 0; i < waypointCount; i++) {
      JsonObject pt = arr.createNestedObject();
      pt["m"] = schedule[i].minutes;
      pt["t"] = schedule[i].tempC;
    }
    String out;
    serializeJson(resp, out);
    client->text(out);
  }
}

// ─── EMBEDDED DASHBOARD HTML ──────────────────────────────────────────────────
// Served from PROGMEM to save RAM

const char DASHBOARD_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Kiln Controller</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
<style>
  :root {
    --bg: #1a1a1a; --surface: #242424; --border: #333;
    --accent: #e06c1a; --green: #4caf50; --red: #e53935;
    --text: #e0e0e0; --muted: #888;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { background: var(--bg); color: var(--text); font-family: 'Courier New', monospace; }
  header { background: var(--surface); border-bottom: 1px solid var(--border);
           padding: 12px 20px; display: flex; align-items: center; gap: 16px; }
  header h1 { font-size: 1.1rem; color: var(--accent); letter-spacing: 2px; text-transform: uppercase; }
  .badge { padding: 3px 10px; border-radius: 3px; font-size: 0.75rem; font-weight: bold; }
  .badge.IDLE     { background: #333; color: var(--muted); }
  .badge.RUNNING  { background: var(--accent); color: #fff; }
  .badge.COMPLETE { background: var(--green); color: #fff; }
  .badge.ERROR    { background: var(--red); color: #fff; }
  .ssr-dot { width: 10px; height: 10px; border-radius: 50%; background: #333; margin-left: auto; }
  .ssr-dot.on { background: var(--red); box-shadow: 0 0 8px var(--red); }
  main { padding: 16px; display: grid; gap: 16px; max-width: 1100px; margin: 0 auto; }
  .stats { display: grid; grid-template-columns: repeat(3, 1fr); gap: 12px; }
  .stat { background: var(--surface); border: 1px solid var(--border); border-radius: 6px;
          padding: 14px; text-align: center; }
  .stat .val { font-size: 2rem; font-weight: bold; color: var(--accent); }
  .stat .lbl { font-size: 0.7rem; color: var(--muted); text-transform: uppercase; margin-top: 4px; }
  .card { background: var(--surface); border: 1px solid var(--border); border-radius: 6px; padding: 16px; }
  .card h2 { font-size: 0.8rem; color: var(--muted); text-transform: uppercase;
             letter-spacing: 1px; margin-bottom: 12px; }
  canvas { width: 100% !important; }
  table { width: 100%; border-collapse: collapse; font-size: 0.85rem; }
  th { text-align: left; color: var(--muted); font-size: 0.7rem; text-transform: uppercase;
       padding: 6px 8px; border-bottom: 1px solid var(--border); }
  td { padding: 5px 8px; }
  td input { background: #111; color: var(--text); border: 1px solid var(--border);
             border-radius: 3px; padding: 3px 6px; width: 80px; font-family: inherit; font-size: 0.85rem; }
  td input:focus { outline: none; border-color: var(--accent); }
  .btn { padding: 8px 18px; border: none; border-radius: 4px; cursor: pointer;
         font-family: inherit; font-size: 0.85rem; font-weight: bold; letter-spacing: 1px; }
  .btn-accent  { background: var(--accent); color: #fff; }
  .btn-muted   { background: #333; color: var(--text); }
  .btn-red     { background: var(--red); color: #fff; }
  .btn-small   { padding: 3px 10px; font-size: 0.75rem; }
  .controls { display: flex; gap: 10px; flex-wrap: wrap; align-items: center; margin-top: 12px; }
  .elapsed-bar { height: 6px; background: #333; border-radius: 3px; margin-top: 12px; }
  .elapsed-fill { height: 100%; background: var(--accent); border-radius: 3px; transition: width 1s; }
  #errorMsg { color: var(--red); font-size: 0.8rem; display: none; margin-top: 8px; }
</style>
</head>
<body>
<header>
  <h1>&#x1F525; Kiln</h1>
  <span class="badge IDLE" id="stateBadge">IDLE</span>
  <span style="color:var(--muted);font-size:0.75rem;" id="elapsedLabel">--:--</span>
  <div class="ssr-dot" id="ssrDot" title="SSR"></div>
</header>

<main>
  <div class="stats">
    <div class="stat">
      <div class="val" id="tempVal">--</div>
      <div class="lbl">Actual °C</div>
    </div>
    <div class="stat">
      <div class="val" id="spVal">--</div>
      <div class="lbl">Setpoint °C</div>
    </div>
    <div class="stat">
      <div class="val" id="diffVal">--</div>
      <div class="lbl">Delta °C</div>
    </div>
  </div>

  <div class="card">
    <h2>Temperature Profile</h2>
    <canvas id="chart" height="280"></canvas>
    <div class="elapsed-bar"><div class="elapsed-fill" id="elapsedFill" style="width:0%"></div></div>
  </div>

  <div class="card">
    <h2>Firing Schedule</h2>
    <table id="scheduleTable">
      <thead><tr><th>#</th><th>Time (min)</th><th>Temp (°C)</th><th></th></tr></thead>
      <tbody id="scheduleBody"></tbody>
    </table>
    <div class="controls">
      <button class="btn btn-muted" onclick="addRow()">+ Add Point</button>
      <button class="btn btn-accent" onclick="saveSchedule()">Save Schedule</button>
      <button class="btn btn-accent" onclick="startFiring()" id="startBtn">&#x25B6; Start</button>
      <button class="btn btn-red" onclick="stopFiring()">&#x25A0; Stop</button>
    </div>
    <div id="errorMsg"></div>
  </div>
</main>

<script>
const ctx = document.getElementById('chart').getContext('2d');
const chart = new Chart(ctx, {
  type: 'line',
  data: {
    datasets: [
      {
        label: 'Projected',
        data: [],
        borderColor: 'rgba(224,108,26,0.4)',
        borderDash: [6,3],
        borderWidth: 2,
        pointRadius: 0,
        fill: false,
        tension: 0.1
      },
      {
        label: 'Actual',
        data: [],
        borderColor: '#e06c1a',
        borderWidth: 2,
        pointRadius: 0,
        fill: false,
        tension: 0.1
      }
    ]
  },
  options: {
    animation: false,
    responsive: true,
    interaction: { mode: 'index', intersect: false },
    scales: {
      x: {
        type: 'linear',
        title: { display: true, text: 'Minutes', color: '#888' },
        ticks: { color: '#888' },
        grid: { color: '#2a2a2a' }
      },
      y: {
        title: { display: true, text: '°C', color: '#888' },
        ticks: { color: '#888' },
        grid: { color: '#2a2a2a' },
        min: 0
      }
    },
    plugins: {
      legend: { labels: { color: '#aaa', font: { family: 'Courier New' } } }
    }
  }
});

let ws;
let actualData = [];
let state = 'IDLE';
let totalMins = 0;

function connect() {
  ws = new WebSocket('ws://' + location.host + '/ws');
  ws.onopen    = () => { ws.send(JSON.stringify({cmd:'get_schedule'})); };
  ws.onclose   = () => setTimeout(connect, 2000);
  ws.onmessage = (e) => {
    const msg = JSON.parse(e.data);
    if (msg.cmd === 'schedule') { loadScheduleFromMsg(msg); return; }
    handleStatus(msg);
  };
}

function handleStatus(d) {
  state    = d.state;
  totalMins = d.totalMins;

  document.getElementById('tempVal').textContent  = d.temp.toFixed(1);
  document.getElementById('spVal').textContent    = d.setpoint.toFixed(1);
  const delta = d.temp - d.setpoint;
  document.getElementById('diffVal').textContent  = (delta >= 0 ? '+' : '') + delta.toFixed(1);

  const badge = document.getElementById('stateBadge');
  badge.textContent  = d.state;
  badge.className    = 'badge ' + d.state;

  const dot = document.getElementById('ssrDot');
  dot.className = 'ssr-dot' + (d.ssr ? ' on' : '');

  if (d.state === 'RUNNING') {
    actualData.push({ x: parseFloat(d.elapsed.toFixed(3)), y: parseFloat(d.temp.toFixed(1)) });
    chart.data.datasets[1].data = actualData;
    chart.update('none');

    const pct = totalMins > 0 ? Math.min(100, (d.elapsed / totalMins) * 100) : 0;
    document.getElementById('elapsedFill').style.width = pct + '%';

    const m = Math.floor(d.elapsed), s = Math.round((d.elapsed - m) * 60);
    document.getElementById('elapsedLabel').textContent =
      String(m).padStart(2,'0') + ':' + String(s).padStart(2,'0');
  }

  if (d.state === 'ERROR') {
    showError(d.error || 'Unknown error');
  }
}

// ─── Schedule table ───────────────────────────────────────────────────────────

function getScheduleRows() {
  const rows = document.querySelectorAll('#scheduleBody tr');
  return Array.from(rows).map(r => {
    const inputs = r.querySelectorAll('input');
    return { m: parseFloat(inputs[0].value), t: parseFloat(inputs[1].value) };
  }).filter(p => !isNaN(p.m) && !isNaN(p.t));
}

function addRow(m, t) {
  const body = document.getElementById('scheduleBody');
  const idx  = body.rows.length + 1;
  const tr   = document.createElement('tr');
  tr.innerHTML = `<td style="color:var(--muted)">${idx}</td>
    <td><input type="number" value="${m !== undefined ? m : ''}" min="0" placeholder="0"></td>
    <td><input type="number" value="${t !== undefined ? t : ''}" min="0" max="1300" placeholder="20"></td>
    <td><button class="btn btn-muted btn-small" onclick="this.closest('tr').remove();renumber()">✕</button></td>`;
  body.appendChild(tr);
}

function renumber() {
  document.querySelectorAll('#scheduleBody tr').forEach((r, i) => {
    r.cells[0].textContent = i + 1;
  });
}

function loadScheduleFromMsg(msg) {
  document.getElementById('scheduleBody').innerHTML = '';
  msg.waypoints.forEach(p => addRow(p.m, p.t));
  rebuildProjected(msg.waypoints);
}

function rebuildProjected(pts) {
  if (!pts || pts.length < 2) return;
  const projected = [];
  const total = pts[pts.length - 1].m;
  const steps = Math.ceil(total * 2); // point every 30s
  for (let i = 0; i <= steps; i++) {
    const minute = (i / steps) * total;
    projected.push({ x: parseFloat(minute.toFixed(2)), y: interpolate(pts, minute) });
  }
  chart.data.datasets[0].data = projected;
  chart.options.scales.x.max = total;
  chart.update('none');
}

function interpolate(pts, m) {
  if (m <= pts[0].m) return pts[0].t;
  for (let i = 1; i < pts.length; i++) {
    if (m <= pts[i].m) {
      const frac = (m - pts[i-1].m) / (pts[i].m - pts[i-1].m);
      return pts[i-1].t + frac * (pts[i].t - pts[i-1].t);
    }
  }
  return pts[pts.length - 1].t;
}

function saveSchedule() {
  if (state === 'RUNNING') { showError("Stop firing before editing schedule."); return; }
  const pts = getScheduleRows();
  if (pts.length < 2) { showError("Need at least 2 waypoints."); return; }
  pts.sort((a, b) => a.m - b.m);
  ws.send(JSON.stringify({ cmd: 'set_schedule', waypoints: pts }));
  rebuildProjected(pts);
  actualData = [];
  chart.data.datasets[1].data = [];
  chart.update('none');
  hideError();
}

function startFiring() {
  if (ws.readyState !== 1) return;
  actualData = [];
  chart.data.datasets[1].data = [];
  chart.update('none');
  ws.send(JSON.stringify({ cmd: 'start' }));
}

function stopFiring() {
  ws.send(JSON.stringify({ cmd: 'stop' }));
}

function showError(msg) {
  const el = document.getElementById('errorMsg');
  el.textContent = '⚠ ' + msg;
  el.style.display = 'block';
}
function hideError() {
  document.getElementById('errorMsg').style.display = 'none';
}

// Default schedule (example: simple bisque fire)
window.onload = () => {
  connect();
  // Will be overwritten by get_schedule response if schedule exists on device
  // Shown only if device has no schedule loaded yet
};
</script>
</body>
</html>
)rawhtml";

// ─── SETUP ────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nKiln Controller starting...");

  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, LOW);

  // Init thermocouple
  if (!thermocouple.begin()) {
    Serial.println("ERROR: MAX31855 not found. Check wiring.");
  } else {
    Serial.println("MAX31855 OK");
  }

  // WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 30) {
    delay(500); Serial.print("."); retries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nConnected. IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nWiFi failed — running offline (no dashboard)");
  }

  // PID setup
  myPID.SetOutputLimits(0, WINDOW_SIZE_MS);
  myPID.SetSampleTime(SAMPLE_INTERVAL_MS);
  myPID.SetMode(MANUAL);
  pidOutput = 0;

  // WebSocket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // Serve dashboard
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send_P(200, "text/html", DASHBOARD_HTML);
  });

  server.begin();
  Serial.println("Web server started.");

  // Default example schedule (can be overridden from browser)
  // Bisque fire: room temp -> 600°C over 2h, hold 30min, cool to 100°C over 3h
  schedule[0] = {0,   20};
  schedule[1] = {120, 600};
  schedule[2] = {150, 600};
  schedule[3] = {330, 100};
  waypointCount = 4;
}

// ─── LOOP ─────────────────────────────────────────────────────────────────────

void loop() {
  ws.cleanupClients();

  unsigned long now = millis();

  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = now;

    // Read temp
    double reading = thermocouple.readCelsius();

    if (isnan(reading)) {
      // Thermocouple fault
      if (controllerState == RUNNING) {
        controllerState = ERROR_STATE;
        errorMessage    = "Thermocouple fault";
        ssrOff();
        myPID.SetMode(MANUAL);
        Serial.println("ERROR: thermocouple fault");
      }
    } else {
      currentTemp = reading;
      pidInput    = currentTemp;

      // Safety cutoff
      if (currentTemp >= SAFETY_MAX_TEMP && controllerState == RUNNING) {
        controllerState = ERROR_STATE;
        errorMessage    = "Over-temperature safety cutoff";
        ssrOff();
        myPID.SetMode(MANUAL);
        Serial.println("ERROR: over-temperature cutoff");
      }

      if (controllerState == RUNNING) {
        float elapsedMin = (now - scheduleStartMs) / 60000.0;

        // Check if schedule complete
        if (elapsedMin >= scheduleTotalMinutes()) {
          controllerState = COMPLETE;
          ssrOff();
          myPID.SetMode(MANUAL);
          Serial.println("Firing schedule complete.");
        } else {
          currentSetpoint = getSetpointAtMinute(elapsedMin);
          pidSetpoint     = currentSetpoint;
          myPID.Compute();
        }
      }
    }

    // Broadcast to all connected clients
    broadcastStatus();

    Serial.printf("[%.1f min] Temp: %.1f°C  SP: %.1f°C  SSR: %s  State: %d\n",
      controllerState == RUNNING ? (now - scheduleStartMs) / 60000.0 : 0.0,
      currentTemp, currentSetpoint, ssrOn ? "ON" : "off", controllerState);
  }

  // SSR PWM window (runs every loop iteration for timing accuracy)
  if (controllerState == RUNNING) {
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
 *     - ESPAsyncWebServer (search for 'ESP Async WebServer')
 *     - AsyncTCP
 *     - ArduinoJson (Benoit Blanchon)
 *     - PID (Brett Beauregard)
 *
 *   After flashing: open Serial Monitor at 115200 baud.
 *   Note the IP address printed, open in browser.
 *
 * SAFETY NOTE:
 *   240V mains wiring must be done properly.
 *   SSR load terminals to kiln element only.
 *   SSR control side is 3-32VDC input — GPIO via 220Ω resistor is fine.
 *   Never touch SSR load side when powered.
 */
