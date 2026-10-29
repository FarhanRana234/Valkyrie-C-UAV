# Valkyrie C-UAV Turret

A real-time dual-sensor automated counter-UAV (C-UAV) tracking and defense turret platform. Powered by YOLO/ONNX inference, Kalman filter motion prediction, ESP32 hardware motion control, and a responsive web command dashboard.

## Quick Start
```bash
pip install -r requirements-export.txt     # export YOLO PyTorch models to ONNX
python export_models.py                    # generates .onnx model binaries
pip install -r requirements.txt            # install web app dependencies
python main.py                             # launches server on http://<your-ip>:5000
```

Access the interface remotely from any phone, tablet, or PC connected to the same local network by opening `http://<your-ip>:5000` in a browser.

---

## System Architecture
* **Tracking Pipeline:** Threaded dual-stream processing (`ir` and `rgb` sensor channels) using ONNX Runtime.
* **Pan/Tilt Motion:** ESP32-controlled motor drivers (TMC2209 for Pitch/Tilt, L298N for Pan/X-axis rotation) with a 3:1 GT2 belt drive reduction.
* **Fire Control & Chamber Logic:** Hall-effect magnetic sensor chamber validation, automatic or manual reload sequencing, safety arming, and active lead tracking.
* **Web Dashboard:** Built with Flask & Socket.IO for low-latency live MJPEG video streaming, remote manual targeting, and telemetric controls.

---

## File Overview
| File | Description |
|---|---|
| `main.py` | Core Flask web server, ONNX inference pipeline, and ESP32 serial coordinator |
| `templates/index.html` | Web browser command center interface |
| `WIRING.md` | Hardware pinout specifications, voltage rails, and component wiring guide |
| `config.json` | Global settings, camera FOVs, model paths, and tracking thresholds |
| `export_models.py` | PyTorch (`.pt`) to ONNX conversion tool |
| `esp32_sim.py` | Hardware serial simulator for testing desktop UI logic without physical hardware |

---

## Auto-Fire Safety Rules
The automated firing sequence is conditionally locked behind multi-stage safety checks:
1. System status is set to **ARMED**.
2. Active target tracking lock is established (not coasting).
3. Target detection confidence exceeds `auto_shoot_confidence`.
4. Target lock age exceeds minimum frame stability threshold (`auto_shoot_min_age`).
5. Angular error between crosshair boresight and target lead position is within `auto_shoot_max_error_deg`.
6. Chamber sensor confirms a dart is properly loaded.
7. Active fire cooldown timer has elapsed.

---

## ESP32 Serial Protocol
* **PC to ESP32 (10 bytes):** `0xA5 | 0x01 | seq(u16) | pan_mrad(i16) | tilt_mrad(i16) | flags(u8) | checksum`
  * *Flags:* Bit 0 = Track Valid | Bit 1 = Fire Request | Bit 2 = Reload Request
* **ESP32 to PC (4 bytes, ~10 Hz):** `0x5A | 0x02 | flags(u8) | checksum`
  * *Flags:* Bit 0 = Dart Present | Bit 1 = Hall Sensor OK | Bit 2 = Reload Active
