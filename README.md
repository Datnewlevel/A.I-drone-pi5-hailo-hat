# AI-Powered Autonomous Landing Drone (Hailo + ArduPilot)

Autonomous drone system combining edge AI object detection, MAVLink flight control, and 4G-networked telemetry to achieve **fiducial-marker tracking and precision landing** — end to end, from bench test to outdoor flight.

> **What it does:** the drone flies a mission to a designated landing waypoint, holds position in `GUIDED` mode, detects a fiducial marker on the ground using an on-board AI accelerator, yaws to center the marker, then executes a controlled descent and precision landing on it.

![Demo Flight](docs/images/demo_flight.gif)
*Replace this with your own flight test footage.*

![Hardware Setup](docs/images/hardware_setup.jpg)
*Replace this with a photo of the assembled drone / companion computer stack.*

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Hardware](#hardware)
- [Software Stack](#software-stack)
- [Repository Structure](#repository-structure)
- [1. Flight Controller Setup (ArduCopter)](#1-flight-controller-setup-arducopter)
- [2. Companion Computer OS Setup](#2-companion-computer-os-setup)
- [3. Hailo AI Accelerator — Install & Model Training](#3-hailo-ai-accelerator--install--model-training)
- [4. MAVLink Routing](#4-mavlink-routing)
- [5. 4G Connectivity (Quectel EC20)](#5-4g-connectivity-quectel-ec20)
- [6. Video Streaming](#6-video-streaming)
- [7. Object Detection + Yaw Tracking (Hailo → MAVLink)](#7-object-detection--yaw-tracking-hailo--mavlink)
- [8. Precision Landing (Fiducial Marker) — Work in Progress](#8-precision-landing-fiducial-marker--work-in-progress)
- [9. Testing Progression](#9-testing-progression)
- [Troubleshooting / Lessons Learned](#troubleshooting--lessons-learned)
- [Roadmap](#roadmap)
- [License](#license)

---

## Overview

This project builds a companion-computer stack for an ArduCopter-based drone that:

1. Flies a normal GPS mission to a landing approach waypoint.
2. Switches to `GUIDED` and holds position over the target area.
3. Runs a YOLO model on a Hailo-8L AI accelerator to detect a marked target (currently a printed fiducial tag).
4. Sends yaw-rate commands over MAVLink (`SET_POSITION_TARGET_LOCAL_NED`) to keep the target centered in frame, using a PID controller on pixel offset.
5. Streams live annotated video back over a 4G link for remote monitoring.
6. (In progress) Switches to fiducial-marker pose estimation for a final precision, centered landing.

The pilot is always in the loop for actual flight — this system does not replace a licensed/experienced pilot for test flights; it augments GUIDED-mode behavior under the pilot's supervision, with manual override always available.

## System Architecture

```
                         ┌─────────────────────────────┐
                         │      Ground Station          │
                         │  Mission Planner / GCS        │
                         └──────────────▲────────────────┘
                                        │ MAVLink UDP 14550
                                        │ (over 4G / Netbird VPN)
┌───────────────────────────────────────┼────────────────────────────────────┐
│  Raspberry Pi 5 (companion computer)  │                                    │
│                                        │                                    │
│   ┌────────────────┐   UDP 14551      │        ┌───────────────────────┐   │
│   │  AI / Control   │◄─────────────────┴───────►│    mavlink-router      │   │
│   │  (Hailo + PID)  │                            │  (owns /dev/ttyAMA0)   │   │
│   └───────▲────────┘                            └───────────▲───────────┘   │
│           │ frames                                            │ serial      │
│   ┌───────┴────────┐                                          │             │
│   │  USB Webcam     │                                          │             │
│   └────────────────┘                                          │             │
│                                                                 │             │
│   ┌────────────────┐                                          │             │
│   │  GStreamer tee   │──► TCP MJPEG stream (to GCS, over 4G)   │             │
│   │  + .mkv recorder │                                          │             │
│   └────────────────┘                                          │             │
│                                                                 ▼             │
│   ┌────────────────┐                              ┌───────────────────────┐ │
│   │ Quectel EC20 4G │◄─────────────────────────────┤   Goku H743 Pro FC     │ │
│   │ + Netbird VPN   │                              │   (ArduCopter)         │ │
│   └────────────────┘                              └───────────────────────┘ │
└───────────────────────────────────────────────────────────────────────────────┘
```

Key design principle: **detection (Hailo/GStreamer), control (pymavlink threads), and streaming are kept architecturally separate.** The AI/control script never touches the flight controller's serial port directly — `mavlink-router` owns it exclusively.

## Hardware

| Component | Part |
|---|---|
| Frame | Drone foldable ZD680 |
| Flight controller | Goku H743 Pro (ArduCopter) |
| ESC | Goku G55M AM32 4-in-1 |
| Companion computer | Raspberry Pi 5 |
| AI accelerator | Hailo AI HAT (13 TOPS) |
| Camera | USB webcam |
| GPS | HGLRC M100-5883 + Holybro M9N (dual GPS) |
| Optical flow | MicoAir MTF-01 (MSP protocol, `SERIAL8`, `FLOW_TYPE=7`) |
| Connectivity | Quectel EC20 4G USB modem + Netbird VPN overlay |
| Battery | 4S LiPo (~16.31V nominal) |

## Software Stack

- **Flight software:** ArduCopter / ArduPilot, Mission Planner, ArduPilot SITL (for logic validation before touching hardware)
- **MAVLink:** `pymavlink`, `mavlink-router`
- **AI / CV:** Hailo SDK + `hailo_apps` GStreamer pipeline, OpenCV, `simple-pid`
- **Model pipeline:** Ultralytics YOLOv8, `onnx-to-hef` Docker conversion, Roboflow for dataset management
- **Networking:** Netbird VPN, Quectel EC20 (ModemManager), NetworkManager, systemd
- **Remote access:** mosh, tmux

## Repository Structure

```
.
├── apps/
│   └── hailo_yaw_follow.py       # Main detection + yaw-tracking + streaming app
├── landing/                      # AprilTag precision-landing prototypes (WIP)
├── tags/                         # Printable fiducial marker files
├── scripts/
│   ├── ec20-4g.service           # systemd unit for 4G modem
│   └── mavlink-router/main.conf  # mavlink-router config
├── docs/
│   └── images/                   # demo media (photos / gifs)
└── README.md
```

> Adjust this tree to match your actual folder layout before publishing.

---

## 1. Flight Controller Setup (ArduCopter)

Parameters worth double-checking before any flight — these were misconfigured at one point in this project and are worth calling out explicitly:

| Parameter | Value | Notes |
|---|---|---|
| `BATT_FS_LOW_VOLT` | `14.0` | 4S LiPo — **do not** leave at 8S defaults |
| `BATT_FS_CRT_VOLT` | `13.2` | |
| `BATT_FS_LOW_ACT` | non-zero (e.g. `2` = Land) | `0` = "no action", which silently disables the failsafe |
| `BATT_FS_CRT_ACT` | non-zero (e.g. `1` = RTL or `2` = Land) | same caveat |
| `GPS_AUTO_SWITCH` | `1` (Best) or `2` (Blend) | dual-GPS setup |

Edit these via **Full Parameter List** or the **Failsafe** sidebar in Mission Planner — the **Battery Monitor calibration page** does not reliably persist the failsafe voltage fields.

Also review:
- `MOT_PWM_TYPE` vs `SERVO_DSHOT_ESC` — confirm these don't conflict for your ESC protocol.
- GPS2 compass offsets after calibration (a large Z-offset is not necessarily a fault, but should be within ArduPilot's acceptable range).

## 2. Companion Computer OS Setup

- Raspberry Pi 5, Raspberry Pi OS (Bookworm or Trixie).
- Disable Wi-Fi before any flight test — it is a confirmed EMI source for the onboard GPS/EKF:
  ```bash
  sudo rfkill block wifi
  ```
- The Pi 5's **USB 3.0 ports are also a confirmed EMI source** for GPS. If you must use a USB 3.0 port, use a USB 2.0-only cable to force the link down to USB 2 speeds and reduce interference. Shielding (e.g. copper tape) around the port helps but doesn't fully close the leakage at the port opening.

## 3. Hailo AI Accelerator — Install & Model Training

Full Hailo-8L installation steps, DKMS driver setup, and the custom YOLOv8 training / HEF conversion pipeline used for this project are documented separately here:

**➡️ [ai-hat-hailo-pi5 setup & training guide](https://github.com/Datnewlevel/ai-hat-hailo-pi5)**

A couple of driver notes relevant to this repo specifically:

- After any kernel update, rebuild the Hailo DKMS module:
  ```bash
  sudo dkms autoinstall
  ```
- Add the driver to persistent module loading:
  ```bash
  echo hailo_pci | sudo tee /etc/modules-load.d/hailo.conf
  ```
- If your custom-trained model has fewer than 4 classes, the Hailo Model Zoo v2.14 HEF conversion will crash. Workaround: train with 1 real class + 3 dummy classes, and keep `data.yaml`, `entrypoint.sh` (`--classes`), and `yolov8s_nms_config.json` in sync on class count.

## 4. MAVLink Routing

`mavlink-router` is the **only** process allowed to open the flight controller's serial port. Everything else — Mission Planner, the AI/control app — talks over UDP.

`scripts/mavlink-router/main.conf`:
```ini
[General]
TcpServerPort = 0

[UartEndpoint fc]
Device = /dev/ttyAMA0
Baud = 921600

[UdpEndpoint gcs]
Mode = Normal
Address = <YOUR_GCS_IP>
Port = 14550

[UdpEndpoint ai]
Mode = Server
Address = 127.0.0.1
Port = 14551
```

Run as a systemd service so it starts on boot and restarts on failure:
```bash
sudo systemctl enable --now mavlink-router
```

- Mission Planner connects on UDP **14550**.
- The AI/control app connects on UDP **14551** (localhost only — never exposed off-device).

### MAVLink control notes (learned the hard way)

- A **1Hz heartbeat** (`MAV_TYPE_GCS`) from the companion computer is mandatory — ArduCopter ignores all external commands from a component that hasn't sent one.
- Filter incoming heartbeats to `MAV_COMP_ID_AUTOPILOT1` only. Mission Planner's own heartbeats (sysID 255) will otherwise cause visible mode flickering in your control loop.
- Set a distinct `source_system` / `source_component` (e.g. `1` / `191`) to avoid ID collisions with Mission Planner.
- Yaw-rate commands require `type_mask = 0b0000010111000111` (`0x05C7`) in `SET_POSITION_TARGET_LOCAL_NED` — bit 11 must be `0` to enable the yaw-rate field.
- `GUIDED` requires a valid GPS position estimate. For indoor/no-GPS bench testing use `GUIDED_NOGPS` with `SET_ATTITUDE_TARGET` instead.
- Yaw direction can come out reversed in hardware — if so, simply negate the PID output (`yaw_rate = -pid(x_error)`).

## 5. 4G Connectivity (Quectel EC20)

APN used in this project: `v-internet` (Viettel, Vietnam) — change to match your carrier.

1. Bring the modem up with ModemManager:
   ```bash
   mmcli -m 0 --simple-connect="apn=v-internet"
   ```
2. Confirm registration:
   ```bash
   mmcli -m 0
   ```
3. Route with a **higher metric (500)** so 4G is used only as intended (e.g. as backup/priority path) rather than fighting with other interfaces:
   ```bash
   sudo nmcli connection modify <ec20-connection-name> ipv4.route-metric 500
   ```
4. Install as a systemd service (`scripts/ec20-4g.service`) that waits for registration on boot before continuing:
   ```ini
   [Unit]
   Description=EC20 4G modem bring-up
   After=network.target

   [Service]
   Type=oneshot
   RemainAfterExit=true
   ExecStart=/usr/local/bin/ec20-connect.sh

   [Install]
   WantedBy=multi-user.target
   ```
   ```bash
   sudo systemctl enable --now ec20-4g.service
   ```
5. Remote access to the Pi is via **Netbird VPN** overlay + `mosh`:
   ```bash
   mosh pi@<netbird-ip>
   ```

## 6. Video Streaming

Working pipeline: GStreamer MJPEG → `matroskamux` → `tcpserversink`, viewed with `ffplay` over TCP.

- **UDP failed** through the Netbird VPN overlay due to WireGuard fragmentation — TCP is used instead.
- The pipeline `tee`s into two branches: a low-quality downscaled branch for the 4G stream, and a full-resolution `.mkv` recording branch for later dataset collection / review — see `VideoStreamer` in `apps/hailo_yaw_follow.py`.
- Camera capability ceiling: 30fps MJPG / 25fps YUYV.
- `v4l2h264enc` is unavailable on this Pi, and `openh264enc` causes a severe FPS drop — MJPEG is the reliable choice here.
- If viewing locally over SSH/mosh, export the display before running anything with `cv2.imshow`, or FPS degrades noticeably:
  ```bash
  export DISPLAY=:0
  ```

## 7. Object Detection + Yaw Tracking (Hailo → MAVLink)

Main script: [`apps/hailo_yaw_follow.py`](apps/hailo_yaw_follow.py).

Three concurrent responsibilities, cleanly separated:

- **Heartbeat thread** — sends the mandatory 1Hz `MAV_TYPE_GCS` heartbeat.
- **Receiver thread** — listens for FC heartbeats/mode changes, filtered to `MAV_COMP_ID_AUTOPILOT1`.
- **Control thread** — runs a PID loop on horizontal pixel offset at 10Hz and sends yaw-rate commands, only while in `GUIDED` mode and only while a detection is fresh (`DETECTION_TIMEOUT`).

The Hailo detection callback runs on every frame (full speed) for the AI inference + MAVLink command path; OpenCV drawing and video streaming are throttled separately to `DRAW_FPS` to avoid wasting CPU on frames nobody needs to see.

```python
PID_KP = 0.04
PID_KI = 0.0
PID_KD = 0.005
MAX_YAW_RATE_DEG = 20.0
DEADZONE_PX = 30
TARGET_LABEL = "tag"
```

`TARGET_LABEL` must match exactly the label string produced by your trained model (check against `tag_labels_v2.json` before deploying a new model — a silent mismatch means the detector runs but never locks onto anything).

## 8. Precision Landing (Fiducial Marker) — Work in Progress

The next stage replaces the current bounding-box/pixel-offset tracking with full 6-DOF pose estimation for centered, accurate landing, using AprilTag detection (`pupil_apriltags`) instead of (or alongside) the Hailo detector for the final approach phase.

**Design used in this project:**

- A **multi-scale nested marker set** — four different tag IDs printed at decreasing sizes on the same landing target, so the vision system always has a marker sized appropriately for the current altitude instead of one marker that's either too small to see from far away or too large to fit in frame up close.
- Each tag ID maps to a known physical `tag_size`, used for `solvePnP`/pose estimation once that specific ID is the closest/largest one currently detected.
- Camera calibration (intrinsics `fx, fy, cx, cy`) is a hard prerequisite for pose estimation — without it, only pixel-offset tracking (not real pose/yaw alignment) is possible.
- Field-testing methodology: log detections (tag ID, pixel size, decision margin) at a series of known altitudes to empirically determine the handoff altitude between marker tiers, rather than relying purely on theoretical FOV calculations.

This part of the system is still being validated and is **not** yet flight-ready — see [Roadmap](#roadmap).

## 9. Testing Progression

This project follows a strict test progression before any new capability goes near a real flight:

1. **Bench test** — code / detection logic sanity check, no flight controller involved.
2. **SITL** (ArduPilot Software-In-The-Loop, run on WSL2 Ubuntu) — validates MAVLink logic, mode handling, and control-loop behavior against a simulated FC.
3. **Armed, no props** — validates real MAVLink communication with the actual flight controller hardware, with props removed for safety.
4. **Tethered flight** — low-altitude hover test with the drone physically restrained.
5. **Outdoor flight** — full test, always flown by an experienced pilot, with manual override available at all times.

## Troubleshooting / Lessons Learned

- **GPS/EKF interference:** confirmed sources are the Pi 5's USB 3.0 ports and 2.4GHz Wi-Fi. Mitigate with `rfkill block wifi`, a USB 2.0-only cable in USB 3.0 ports, and copper-tape shielding (note: shielding alone doesn't fully close leakage at port openings).
- **Battery failsafe silently disabled:** `BATT_FS_LOW_ACT` / `BATT_FS_CRT_ACT` set to `0` means "no action" — always confirm these are non-zero before flight.
- **False positives are a training-data problem, not a calibration problem** — if your detector fires on the wrong things, add hard negative samples to the dataset rather than tightening thresholds.
- **Recording format:** use `XVID`/`.avi` rather than `mp4v`/`.mp4` for on-device recording — an interrupted shutdown during `.mp4` recording can leave a file with a missing moov atom that's unreadable afterward.
- **Camera backend:** always open with `cv2.CAP_V4L2` — without it, setting camera properties can hang the terminal.

## Roadmap

- [ ] Camera intrinsic calibration (chessboard/ChArUco) for the USB webcam
- [ ] Finalize multi-tier fiducial marker sizes based on field distance-test data (5m / 8m / 12m / 15m)
- [ ] Integrate `LANDING_TARGET` MAVLink message + ArduCopter's built-in `PLND` controller (`PLND_ENABLED=1`)
- [ ] Tethered low-altitude hover test with the landing pipeline active
- [ ] Full outdoor precision-landing flight test

## License

MIT, Apache-2.0