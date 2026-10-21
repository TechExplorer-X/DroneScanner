# 🛸 DroneScanner — ESP32 Drone Remote ID Scanner

**English** | [**简体中文**](README_CN.md)

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)
[![ESP32](https://img.shields.io/badge/Hardware-ESP32-blue)](https://www.espressif.com/)
[![Open Drone ID](https://img.shields.io/badge/Protocol-ASTM_F3411--22a-green)](https://www.astm.org/f3411-22a.html)

> An ESP32-based drone detection and identification system that listens to Remote ID broadcasts from drones using WiFi promiscuous mode and displays the detection results in real time through a web control panel.

---

## 📸 Preview

| Web Control Panel | Detailed Output |
|:---:|:---:|
| ![Web UI Screenshot](/img/侦测1.jpg) | ![Detailed Info](/img/侦测2.jpg) |

---

## ✨ Features

- **📡 Real-time drone detection** — Captures drone broadcast packets in WiFi promiscuous mode
- **🔍 Open Drone ID decoding** — Full ASTM F3411-22a Remote ID protocol parsing
- **🏷️ Multi-brand model identification** — Built-in database of 100 SN-prefix mappings covering 12 brands (DJI, Autel, Parrot, Skydio, Yuneec, etc.) with automatic brand, model & category matching
- **🖥️ Web control panel** — ESP32 acts as an AP; view results directly from any phone/PC browser
  - Real-time list: MAC, RSSI, model, serial number, GPS coordinates
  - Real-time statistics: detection count, total packets, uptime
  - One-click clear
- **🔌 Dual serial output** — Serial (USB debug) + Serial1 (external display / logging)

---

## 📊 Model Database

Built-in **100** SN-prefix → brand → model → category mappings covering **12 brands**:

| Category | Entries | Examples |
|:----:|:--------:|------|
| Consumer (消费级) | 33 | Mavic 4 Pro, Mini 5 Pro, Air 3S, EVO II, Anafi |
| FPV (穿越机) | 8 | Avata 2, Avata 360, Flip, Neo 2 |
| Enterprise (行业级) | 35 | Matrice 4 Series, Matrice 350 RTK, Dragonfish, Skydio X2 |
| Agriculture (植保级) | 11 | Agras T50, Agras T100, XAG P150 Max |
| Logistics (物流级) | 2 | FlyCart 30, FlyCart 100 |
| Cinema (影视级) | 3 | Alta X, Alta X Gen2, Alta 8 |
| Surveying (测绘级) | 8 | eBee X, WingtraOne, md4-1000 |

Brands: DJI (55), Autel (13), Parrot (5), Skydio (4), Yuneec (4), XAG (4), senseFly (4), Freefly (3), Percepto (3), Microdrones (2), Wingtra (2), SwellPro (1).

> Matching is based on the SN prefix (e.g., `1581F8`); extend the `droneModels` array in `scanner/scanner.ino` to add more mappings.

---

## 🧱 Hardware Requirements

| Component | Spec |
|------|------|
| **ESP32 dev board** | Any WiFi-enabled ESP32 (ESP32-C3 / S3 / WROOM, etc.) |
| **USB cable** | Power + serial debug |
| **External antenna (optional)** | 2.4GHz antenna to extend WiFi detection range |

> ✅ No additional RF hardware required — the ESP32's built-in WiFi controller captures drone broadcasts.

---

## 🚀 Quick Start

### 1️⃣ Install Arduino IDE + ESP32 support

1. Download and install [Arduino IDE](https://www.arduino.cc/en/software)
2. Add ESP32 board support: **File → Preferences → Additional boards manager URLs**
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. **Tools → Board Manager → search "ESP32" → Install**

### 2️⃣ Clone / download this project

```bash
git clone https://github.com/TechExplorer-X/DroneScanner.git
cd DroneScanner
```

### 3️⃣ Compile & flash

1. Open `scanner/scanner.ino` in Arduino IDE
2. Select board: **ESP32 Dev Module** (or the specific model matching your hardware)
3. Select the correct COM port
4. Click **Upload** (→)

### 4️⃣ Connect & use

1. Search for the WiFi hotspot: **`DroneScanner`** (password: `12345678`)
2. Open [http://192.168.4.1](http://192.168.4.1) in a browser
3. The web page auto-refreshes; drones appear as soon as they are detected

---

## ⚙️ How It Works

1. **Promiscuous mode listening** — ESP32 enables WiFi promiscuous mode (`wifi.c`) without connecting to any router
2. **ODID frame filtering** — Listens to all 2.4GHz WiFi packets and filters Beacon/Probe frames containing the ODID protocol header
3. **Protocol decoding** — `odid_wifi` extracts the ODID payload, `opendroneid` decodes it into structured data
4. **Data caching** — Results are stored in a ring buffer (up to `MAX_DRONES` = 20 entries), newest data overwrites oldest
5. **Visualization** — The built-in WebServer provides a REST API and an HTML control panel
6. **Debug output** — Serial (115200 baud) prints real-time logs for debugging or external displays

---

## 🛠️ Configuration

### WiFi hotspot

Edit the top of `scanner/scanner.ino`:

```cpp
const char* AP_SSID = "DroneScanner";     // hotspot name
const char* AP_PASSWORD = "***";          // hotspot password
const int AP_CHANNEL = 6;                 // WiFi channel
```

### Serial pins

```cpp
const int SERIAL1_RX_PIN = 7;
const int SERIAL1_TX_PIN = 6;
```

Adjust freely according to your wiring.

---

## 🌐 REST API

> The ESP32 acts as an AP and provides the following REST API:

| Path | Method | Description |
|------|:----:|------|
| `/` | GET | Web control panel (HTML + JS) |
| `/api/drones` | GET | JSON list of detected drones |
| `/api/clear` | GET | Clear all detection records |

### `/api/drones` response example

```json
{
  "drones": [
    {
      "mac": "AA:BB:CC:DD:EE:FF",
      "rssi": -65,
      "uav_id": "1581F8LQ1234",
      "model_name": "Mavic 4 Pro (64GB)",
      "model_cat": "消费级",
      "brand": "DJI",
      "lat": 39.9042,
      "lon": 116.4074,
      "altitude_msl": 120,
      "height_agl": 80,
      "speed": 5.2,
      "heading": 180,
      "op_id": "OPERATOR_ID",
      "last_seen": 3
    }
  ],
  "count": 1,
  "packets": 42,
  "version": "2.2",
  "uptime": 3600
}
```

---

## 🗂️ Project Structure

```
DroneScanner\
├── LICENSE                 # GPL-3.0 license
├── scanner/
│   ├── scanner.ino         # Main program (Web UI + API + core logic)
│   ├── opendroneid.h       # Open Drone ID protocol decoder header
│   ├── opendroneid.c       # Open Drone ID protocol decoding implementation
│   ├── odid_wifi.h         # ODID over WiFi wrapper (with extern "C" guards)
│   └── wifi.c              # WiFi promiscuous mode configuration
```

---

## 🗺️ Roadmap

- [ ] Expand the DJI model identification database (more models & firmware versions)
- [ ] Add automated tests for Remote ID packet decoding and CI builds
- [ ] Improve WiFi promiscuous mode reception stability and detection range
- [ ] Enhance the web dashboard (map view, history, etc.)
- [ ] Add more field test data and bilingual documentation

---

## ❓ FAQ

- **Cannot connect to the WiFi hotspot?** Make sure the ESP32 is powered on and search for `DroneScanner` in your WiFi list. If the AP is not visible, check the antenna connection.
- **No drones detected?** Ensure the drone is airborne (some models do not broadcast Remote ID on the ground), move closer to the drone (about 100m without an antenna), check the ESP32 antenna, and watch the serial monitor for logs.
- **Web page always shows 0 drones?** Open the Arduino IDE serial monitor (115200 baud) and check for `[state]` log output. If there is no output at all, check whether WiFi promiscuous mode initialized successfully.

---

## 📄 License

This project is developed based on the open-source ODID library and the ESP32 Arduino framework, **for educational and research purposes only**.

- [Open Drone ID](https://github.com/opendroneid) — open-source ODID protocol implementation
- [ASTM F3411-22a](https://www.astm.org/f3411-22a.html) — Remote ID standard
- [ESP32 Arduino WiFi](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/wifi.html) — official docs

Licensed under **GPL-3.0**. See [LICENSE](LICENSE).

---

## 🙌 Contributing

Issues and pull requests are welcome! If you add new drone model mappings or improve ODID decoding, please include the corresponding hardware identifiers and test data.

---

## 🛸 About DroneScanner

DroneScanner was created to make drone identification technology accessible to developers, researchers, and hobbyists using affordable hardware. The project combines embedded firmware, wireless protocol decoding, and a browser-based interface.

The maintainer is committed to sustained open-source maintenance and welcomes community collaboration. The goal is to build a sustainable, accessible, and community-driven open-source project that supports open research and education in UAV identification.
