# Kiln Controller

ESP32-S3 based PID kiln controller with browser dashboard. Set firing schedules, monitor live temperature, and watch actual vs projected curves in real time — all from a browser on your local network.

![Status](https://img.shields.io/badge/status-in%20development-orange)
![Platform](https://img.shields.io/badge/platform-ESP32--S3-blue)
![License](https://img.shields.io/badge/license-MIT-green)

---

## Features

- PID temperature control via solid state relay (SSR)
- Ramp / hold / ramp firing schedules with unlimited waypoints
- Browser dashboard served directly from the ESP32 — no app, no cloud
- Live graph: projected curve vs actual temperature
- Editable schedule in the browser (saved to device)
- Thermocouple fault detection and over-temperature safety cutoff
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

Schedules are defined as waypoints: `(time in minutes, target °C)`. The controller linearly interpolates between them, so you get ramps for free.

Example — simple bisque fire:

| Time (min) | Temp (°C) | Notes |
|---|---|---|
| 0 | 20 | Room temp start |
| 120 | 600 | 2hr ramp up |
| 1 | 980 | fast as possible to 980 |
| 330 | 100 | 3hr ramp down |

A default schedule is pre-loaded. You can edit it in the browser and save it to the device without reflashing.

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
- [ ] Schedule persistence (save to SPIFFS, survives power cycle)
- [ ] Multiple saved schedule slots
- [ ] Remote access (Tailscale / ngrok notes)
- [ ] Firing log export (CSV)

---

## License

MIT — do whatever you like with it. A mention is appreciated but not required.
