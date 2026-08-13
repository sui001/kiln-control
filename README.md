# Kiln Controller

ESP32-S3 based PID kiln controller with browser dashboard. Set firing schedules, monitor live temperature, and watch actual vs projected curves in real time — all from a browser on your local network.

![Status](https://img.shields.io/badge/status-in%20development-orange)
![Platform](https://img.shields.io/badge/platform-ESP32--S3-blue)
![License](https://img.shields.io/badge/license-MIT-green)

---

## Features

- PID temperature control via solid state relay (SSR)
- Step-based firing schedule: ramp / hold / full power / off
- Schedule persists to LittleFS — survives reboot, no reflash needed to change a fire
- Browser dashboard served directly from the ESP32 — no app, no cloud
- Live graph: projected curve vs actual temperature
- Editable schedule in the browser, plus a rate-to-time (°C/hr) converter
- Thermocouple fault detection and over-temperature safety cutoff
- "Full power" steps run the SSR at 100% until within tolerance of target, with a timeout fault if they never get there
- Serial monitor logging

---

## Hardware

| Component | Notes |
|---|---|
| ESP32-S3 SuperMini | Or any ESP32-S3 dev board |
| MAX31855 breakout | Adafruit or clone, SPI |
| K-type thermocouple | Rated for your max firing temp |
| Solid State Relay (SSR) | 40A recommended for kiln duty |
| 240V kiln | Element wired through SSR load side |

### Wiring

```
MAX31855
  VCC  →  3.3V
  GND  →  GND
  SCK  →  GPIO12
  CS   →  GPIO10
  SO   →  GPIO13

SSR
  Control +  →  GPIO9 (via 220Ω resistor)
  Control -  →  GND
  Load side  →  240V kiln element (respect mains safety)
```

> ⚠️ **240V mains wiring must be done safely.** The SSR load side carries lethal voltage. If you're not confident, get a sparky to do that part.

---

## Software

### Arduino IDE setup

1. Install ESP32 board support via Board Manager  
   URL: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`

2. Board settings:
   - Board: **ESP32S3 Dev Module**
   - USB CDC On Boot: **Enabled**
   - Upload Speed: 921600
   - Flash Mode: QIO 80MHz

### Libraries (install via Library Manager)

| Library | Author |
|---|---|
| Adafruit MAX31855 | Adafruit |
| ArduinoJson | Benoit Blanchon |
| PID | Brett Beauregard |
| ESPAsyncWebServer | me-no-dev (install from GitHub) |
| AsyncTCP | me-no-dev (install from GitHub) |

**ESPAsyncWebServer and AsyncTCP** are not in the standard Library Manager. Install manually:
- https://github.com/me-no-dev/ESPAsyncWebServer
- https://github.com/me-no-dev/AsyncTCP

Download as ZIP and install via **Sketch → Include Library → Add .ZIP Library**

---

## Configuration

Open `kiln_controller.ino` and edit the config block near the top:

```cpp
const char* WIFI_SSID     = "YOUR_SSID";
const char* WIFI_PASSWORD = "YOUR_PASSWORD";

#define SSR_PIN         9      // GPIO driving SSR control input
#define MAX_CS_PIN      10     // MAX31855 chip select

#define PID_KP          2.0    // Tune for your kiln
#define PID_KI          0.005
#define PID_KD          1.0

#define SAFETY_MAX_TEMP 1300.0 // Emergency cutoff (°C)
```

> **Never commit real WiFi credentials.** This repo is public — leave `YOUR_SSID` / `YOUR_PASSWORD` as placeholders in anything you push, and only fill in real values on the copy that stays on your machine.

---

## Usage

1. Flash the firmware
2. Open Serial Monitor at **115200 baud**
3. Note the IP address printed on connect
4. Open that IP in a browser on the same network
5. Edit the firing schedule in the browser, hit **Save Schedule**
6. Hit **Start** when ready

The dashboard shows:
- Live temperature, setpoint, and delta
- Projected firing curve (dashed) vs actual (solid)
- SSR on/off indicator
- Progress bar through the schedule

---

## Firing Schedule

Schedules are a list of steps, each one of:

| Step type | Fields | Behaviour |
|---|---|---|
| `ramp` | target °C, duration (h/m) | Linear ramp from current setpoint to target over the duration |
| `hold` | duration (h/m) | Holds the setpoint reached by the previous step |
| `full` | target °C | SSR at 100% duty (PID bypassed) until within 3°C of target, or faults out after 4h if it never gets there |
| `off` | duration (h/m) | SSR off, setpoint holds flat (for a controlled cool-down window) |

Example — simple bisque fire: ramp to 600°C over 2h, hold 30min, off for 3h to cool.

Edit steps in the browser, hit **Save Schedule** — it's written to `/schedule.json` on the device and survives a reboot. No reflash needed to change a fire.

---

## PID Tuning

The default values (`KP=2.0, KI=0.005, KD=1.0`) are a starting point. Every kiln has different thermal mass.

Signs to look for:
- **Overshooting badly** → reduce KP, increase KD
- **Sluggish / can't keep up** → increase KP
- **Oscillating around setpoint** → reduce KI

Run a few test fires at a fixed setpoint and watch the Serial output before doing a real fire.

---

## Planned

- [ ] AP fallback mode (ESP32 creates own hotspot if WiFi unavailable)
- [ ] Multiple saved schedule slots
- [ ] Remote access (Tailscale / ngrok notes)
- [ ] Firing log export (CSV)
- [ ] Run-state (current step / elapsed-within-step) persistence across a power cut — only the schedule definition survives right now, not firing progress

---

## History

`kiln_sensor_display_v1_5/` is an earlier prototype built for a Waveshare ESP32-S3-Matrix + ILI9341 TFT, back when SuperMinis weren't on hand. It has the step-schedule editor and LittleFS persistence but never actually drove the SSR (tracking-only). It's kept for reference but superseded by `kiln_controller.ino`, which merges its schedule/persistence work with the earlier PID+SSR control loop, retargeted to the SuperMini.

---

## License

MIT — do whatever you like with it. A mention is appreciated but not required.
