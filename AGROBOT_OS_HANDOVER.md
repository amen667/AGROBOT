# AGROBOT — Engineering handover for custom OS integration

**Baseline:** 26 November 2026 · single-motor bench prototype · Raspberry Pi 4 / ESP32 · ROS 2 Jazzy · Dashboard V2

**Repository:** https://github.com/amen667/AGROBOT · branch `main` · confirmed integration commit `5669cad` (nine added files, 896 insertions). Dashboard and live-camera code were subsequently copied to the Pi and operated successfully; their Git upload is pending at the time of this handover.

This document describes the new prototype, its interfaces, configuration, reproducibility requirements and known limitations so the OS developer can integrate it independently. The original team navigation, dual-motor bridge, firmware, AI and repository README are preserved. For normal operation and all physical wiring, see [README_SINGLE_MOTOR.md](README_SINGLE_MOTOR.md).

## 1. Work completed and architecture

- Recovered the AI workflow on the ASUS Ubuntu development laptop and deployed it to a Raspberry Pi 4 running Ubuntu Server 24.04 and ROS 2 Jazzy.
- Converted the pretrained MobileNetV2 38-class PlantVillage crop disease model to float32 TFLite. Compared converted logits with the PyTorch model during conversion. Selected tomato-only predictions for the prototype.
- Integrated USB camera capture, TFLite inference and ROS 2 inspection messages; implemented one-shot, periodic and on-demand inspection variants.
- Flashed a new single-motor ESP32 firmware, configured Pi GPIO UART, tested UART command/feedback, encoder counts and physical motor movement; kept the original firmware and dual-motor implementation separate.
- Implemented a single-cycle bench controller and a three-cycle automatic stop–inspect–move–stop controller, both on separate bench topics.
- Confirmed physical movement following an AI result in the single-cycle test. Verified all three automatic cycles with motor power disconnected.
- Built Mission Control V1 to start/stop processes and display logs, ROS connectivity, prediction and motion command. Upgraded to V2 with live camera, snapshot, capture/inference events and inspection IDs. User confirmed V2 operational on the Pi after resolving an old dashboard's port conflict.
- Established SSH-based access and committed/pushed the nine core files. Local model assets remain outside that commit.

```text
Laptop browser http://localhost:8765
       | HTTP through SSH tunnel (local port 8765)
       v
Pi dashboard V2 (HTTP loopback + ROS monitor + child process supervisor)
       |                                  |
       | starts AI service                | starts navigation + motor bridge
       v                                  v
USB camera -> live camera AI service <--- /plant_health/inspect --- navigation V1 auto
       |                                  |                         |
       | preview / snapshot / events      | Trigger response        | Twist
       | and classification result        +-------------------------+
       v                                                            v
Dashboard monitor <--- encoder topic ------------------------ single motor bridge
                                                                    |
                                                       /dev/ttyAMA0, 115200 8N1
                                                                    |
                                                               ESP32 UART2
                                                                    |
                                                      BTS7960 -> DC motor + encoder
```

The navigation controller calls the service and uses its response to decide when to move; dashboard and other observers consume published results. The dashboard does not run inference itself. The original LiDAR navigation (`ros2_ws/src/agrobot_navigation/`) is not the controller used in this prototype.

## 2. File inventory and ownership

| Repository-relative path | Purpose | Status at handover |
|---|---|---|
| `ai_vision/plant_health_tflite.py` | `PlantClassifier`: preprocessing, inference, tomato filtering | Commit `5669cad` |
| `ai_vision/camera_ai.py` | One-shot camera and classification test | Commit `5669cad` |
| `ai_vision/camera_ai_node.py` | Periodic camera AI result publisher | Commit `5669cad` |
| `ai_vision/camera_ai_service.py` | Original on-demand inspection service | Commit `5669cad` |
| `ai_vision/camera_ai_live_service.py` | Single-owner camera preview, capture and inspection service for V2 | On Pi; pending Git push |
| `raspberry_pi/inspection_controller.py` | Simulated inspection/movement controller | Commit `5669cad` |
| `raspberry_pi/single_motor_bridge.py` | ROS 2 Twist to UART + encoder telemetry | Commit `5669cad` |
| `raspberry_pi/navigation_v1_bench.py` | One result triggers one timed movement | Commit `5669cad` |
| `raspberry_pi/navigation_v1_auto.py` | Three-cycle synchronized state machine | Commit `5669cad` |
| `firmware_esp32/single_motor_controller/single_motor_controller.ino` | New single-motor firmware, UART2 and watchdog | Commit `5669cad`; flashed version matches UART2 / PWM settings |
| `agrobot_dashboard.py` | Mission Control V1 | On Pi; pending Git push |
| `agrobot_dashboard_v2.py` | Mission Control V2 (recommended entry point) | On Pi; pending Git push |
| `README_SINGLE_MOTOR.md` | Operator quick start + complete wiring | Updated for V2 |
| `AGROBOT_OS_HANDOVER.md` | This integration handover | New |

A standalone `start_agrobot.sh` was proposed as an intermediate alternative; its creation on the Pi has not been confirmed and it is not needed for the V2 dashboard. Do not run it concurrently with the dashboard. Do not automatically replace or activate the original team nodes.

## 3. Hardware and host configuration

| Item | Tested configuration |
|---|---|
| Host | Raspberry Pi 4, Ubuntu Server 24.04, ROS 2 Jazzy |
| Development machine | ASUS TUF Gaming A15, Ubuntu |
| User / network | `amen`, Pi address `192.168.1.134` at test time |
| Repository directory | `/home/amen/AGROBOT` |
| AI virtual environment | `/home/amen/agrobot_ai_env` |
| Microcontroller | ESP32-WROOM-32 |
| Driver | BTS7960 / IBT-2, tested board uses 74HC244 buffer |
| Motor | Single JGA25-370 12 V 280 RPM DC geared motor, quadrature encoder |
| Motor supply | 3S LiPo 1300 mAh 25C (11.1 V nominal, 12.6 V fully charged) |
| Camera | USB UVC, OpenCV V4L2 index 0 (`/dev/video0` in tests) |

See the quick-start README for the full pin tables. Summary: Pi physical pins 6 (GND), 8 (GPIO14 TX), 10 (GPIO15 RX) connect to ESP32 GND, GPIO16 RX2 and GPIO17 TX2. ESP32 GPIO23/18 drive RPWM/LPWM; GPIO21/22 receive encoder A/B. Driver logic and encoder were tested at 3.3 V; verify exact driver and encoder voltage requirements before replication. Use a separate fused motor-current path. The ESP32 was USB-powered from the Pi; optional external regulated supply has not yet been tested. Never feed raw LiPo voltage into the logic boards.

### Host UART boot configuration (already applied on tested Pi)

- `/boot/firmware/config.txt`: `enable_uart=1`; `[all]` contains `dtoverlay=disable-bt` to make the primary UART available.
- `/boot/firmware/cmdline.txt`: remove the `console=serial0,115200` entry, preserving all other boot options on the same line.
- `serial-getty@ttyAMA0.service`: disabled so the UART is not occupied by a login console. On this installation `hciuart` was absent; check platform-specific Bluetooth services before changing them on another OS.
- `amen` belongs to `dialout`. Device `/dev/ttyAMA0` is the configured serial endpoint. On a different kernel/board confirm the actual device mapping, permissions and pin mux; do not assume this pathname is universal.
- Reboot after boot configuration changes. Only a single bridge may own the UART. Use `sudo fuser -v /dev/ttyAMA0` for diagnostics.

USB camera enumeration was intermittent during earlier tests; unplugging/reconnecting resolved it. The live service currently hard-codes index 0. For custom OS integration, prefer a configurable stable device path after testing camera enumeration. Previously observed alias: `/dev/v4l/by-id/usb-S5K6A1-201014_Integrated_Camera-video-index0`.

## 4. Runtime dependencies and AI assets

The dashboard uses standard-library HTTP, JSON, threading and subprocess modules plus ROS Python bindings. Its child processes use these interpreter paths:

| Process | Interpreter | Entry point |
|---|---|---|
| Dashboard | system Python 3, ROS environment sourced | `agrobot_dashboard_v2.py` |
| Motor bridge | `/usr/bin/python3` | `raspberry_pi/single_motor_bridge.py` |
| Navigation | `/usr/bin/python3` | `raspberry_pi/navigation_v1_auto.py` |
| Live AI service | `~/agrobot_ai_env/bin/python` | `ai_vision/camera_ai_live_service.py` |

Dependencies: ROS 2 Jazzy `rclpy`, `geometry_msgs`, `std_msgs`, `std_srvs`, `sensor_msgs`; system Python `pyserial`; AI venv `ai-edge-litert` (tested 2.1.2), NumPy (tested 2.5.5), Pillow and `opencv-python-headless`. AI venv is configured with `include-system-site-packages = true` in `pyvenv.cfg` to expose ROS system packages. ROS Jazzy Python bindings and AI venv must be ABI-compatible (Ubuntu 24.04 system Python 3.12). Camera access requires compatible V4L2/UVC drivers and permissions.

The test installation sources `/opt/ros/jazzy/setup.bash`; dashboard-managed child processes inherit ROS environment variables. All three processes and dashboard monitor must share the same ROS domain and compatible discovery configuration.

### AI model

Source model: `linkanjarad/mobilenet_v2_1.0_224-plant-disease-identification` (Hugging Face). Float32 conversion performed on laptop using ONNX/ONNX2TF; approximately 8.7 MB TFLite output. The existing local installation includes:

```text
ai_vision/models/
  plant_health_model_float32.tflite
  labels.json
  preprocessing/preprocessor_config.json
```

Verify exact asset paths against `plant_health_tflite.py` and the Pi before building the OS image. These assets were not included in the confirmed Git commit; a clone alone cannot run inference. Obtain authorized copies or recreate the conversion after checking upstream model and data licenses. The conversion notebook/recipe and full dependency lock are not yet packaged as a reproducible build.

Preprocessing: PIL RGB, resize shortest side to 256, center crop 224×224, scale to [-1,1], float32 NHWC model input. `PlantClassifier.classify(image)` returns ranked tomato classes, inference elapsed time in ms, and total probability mass of tomato classes. Observed inference around 38–42 ms on the Pi in earlier tests; capture and warmup make end-to-end inspection longer. This is a measured example, not a guarantee.

**Important AI limitation:** tomato probabilities are renormalized within the tomato subset. A desk, wall, hand or other non-leaf object can therefore still produce a high tomato-class confidence. Total tomato probability is not a validated plant-presence score either. The current implementation has no leaf detector, background class or calibrated reject mechanism. A successful service response means inference completed, not that a tomato leaf or valid diagnosis was confirmed. AI V2 rejection/detection and accuracy testing are pending; the dashboard upgrade did not change the model.

## 5. Interface contracts for OS integration

### ROS 2 interfaces

| Name | Type | Producer / consumer |
|---|---|---|
| `/plant_health/inspect` | `std_srvs/srv/Trigger` | AI service server; nav and dashboard clients |
| `/plant_health/result` | `std_msgs/msg/String` | AI service publishes JSON; dashboard observes |
| `/plant_health/camera/preview` | `sensor_msgs/msg/CompressedImage` | Live AI service -> dashboard; sensor-data QoS |
| `/plant_health/camera/snapshot` | `sensor_msgs/msg/CompressedImage` | Live AI service -> dashboard; depth 10 |
| `/plant_health/camera/event` | `std_msgs/msg/String` | Live AI service -> dashboard; depth 20 |
| `/agrobot/bench/cmd_vel` | `geometry_msgs/msg/Twist` | Navigation publishes; bridge and dashboard subscribe |
| `/agrobot/bench/encoder_ticks` | `std_msgs/msg/Int64` | Bridge publishes; navigation and dashboard subscribe |

Service request is empty. Successful response has `success=true` and `message` containing the same JSON as the result topic. Failure has `success=false` and an error message. V2 example (illustrative values):

```json
{"status":"inspection_complete","prediction":"Tomato Yellow Leaf Curl Virus","confidence":0.82,"inference_ms":39.5,"tomato_probability":0.63,"inspection_id":"a1b2c3d4e5f6-1"}
```

Snapshot `header.frame_id` holds the matching inspection ID. Both JPEG snapshot and RGB inference input originate from the same captured frame; JPEG compression and classifier preprocessing mean the displayed image is not the literal model tensor. The preview `frame_id` is `camera`. Preview timer runs every 0.15 s (up to ~6.7 FPS, actual frame rate depends on capture); width is downscaled to at most 480 px, JPEG quality 72. Snapshot keeps original frame size, quality 95. Single-threaded AI executor ensures camera preview and inspection do not concurrently access the same device. Preview may pause during inference. Camera warms up for 10 frames; retries opening after ~2 seconds on error.

Camera event JSON fields: `phase`, `inspection_id` (nullable), `detail`, `timestamp` (Unix wall clock seconds). Phases: `camera_warming`, `camera_ready`, `camera_error`, `capturing`, `analyzing`, `complete`, `inspection_error`. IDs combine a per-process random session prefix and incrementing count. Snapshot and event history are in memory, not a persistent inspection database.

### ESP32 serial contract

- `/dev/ttyAMA0` at 115200, 8N1; 3.3 V UART logic. ESP32 uses `Serial2.begin(115200, SERIAL_8N1, 16, 17)`.
- Single ASCII bytes `f` / `b` / `s` mean forward / backward / stop. ROS bridge sends current command every 50 ms. Do not substitute the original dual-motor serial protocol.
- Firmware motor PWM constant 130 out of 255; not a calibrated speed. On startup motor is stopped. Firmware watchdog stops motor after 300 ms without a refreshed movement command.
- Telemetry approximately every 100 ms: `ENC:<signed count> ACTIVE:<0 or 1>` followed by newline. Bridge parses telemetry and publishes signed count only; ACTIVE is not exported as a ROS topic. Direction sign depends on encoder wiring and rotation.
- Bridge maps `linear.x > 0.02` to forward, `< -0.02` to backward, otherwise stop. Angular command is not used. Bridge commands stop after 250 ms without ROS command or 500 ms without serial feedback. No measured speed, odometry or distance control is implemented in this new bridge.

### Automatic navigation state machine

```text
SETTLING (stop; await fresh feedback, stable encoder and service)
   -> INSPECTING (stop; async Trigger request)
   -> MOVING (only after successful response and inspection_complete)
   -> SETTLING (stop; increment cycle)
   -> DONE after 3 movement cycles, or FAULT on timeout/error
```

| Parameter | Value |
|---|---|
| Loop | 20 Hz (50 ms) |
| Max cycles | 3 |
| Forward command | `Twist.linear.x = 0.045` (direction threshold, not actual measured m/s) |
| Move duration | 2 s |
| Settle duration / encoder stability | 0.7 s |
| Inspection timeout | 10 s |
| Encoder feedback freshness | 0.5 s |
| Settling/service readiness timeout | 5 s |

A final movement occurs after the third inspection, then the controller stops. DONE and FAULT continue publishing stop until the process is stopped. To repeat cycles from the dashboard, stop completed navigation then start again. Do not run `navigation_v1_bench.py` or other Twist publishers concurrently. No plant localization, row-end detection, wheel-distance calibration, verified motor movement, stall detection or obstacle avoidance yet.

## 6. Dashboard runtime and integration points

Use the dashboard V2 as the sole process supervisor for the current prototype. It starts no robot nodes at initial launch. **Normal operator startup:**

```bash
ssh -t -o ExitOnForwardFailure=yes -L 8765:127.0.0.1:8765 amen@192.168.1.134 'source /opt/ros/jazzy/setup.bash && python3 ~/AGROBOT/agrobot_dashboard_v2.py'
```

Browser: `http://localhost:8765`. The SSH tunnel remains open while operating. Normal maintenance SSH: `ssh amen@192.168.1.134`. The current IP is not guaranteed to persist after network changes. The server binds to Pi loopback only, and restricts Host headers to localhost/127.0.0.1:8765. Do not expose port 8765 directly to LAN or internet without redesigning authentication, authorization and transport security. POST actions require a random per-process token embedded in the page; this is not a user-login system. SSH authenticates access to the tunnel.

### HTTP interface (current private implementation, not a stable public API)

| Method / path | Purpose |
|---|---|
| GET `/` | Mission Control HTML, CSS and JS |
| GET `/api/state` | Managed node states, encoder count/age, latest command, result, phase, cycle, service/graph state, logs |
| GET `/api/camera` | Live flag, preview sequence, snapshot ID, recent camera events |
| GET `/camera/live.jpg?...` | Most recent fresh JPEG preview |
| GET `/camera/snapshot.jpg?id=<inspection_id>` | Current matching JPEG snapshot |
| POST `/api/action` | `start`, `stop`, `stop_all`, `prepare`, `inspect`; node keys `bridge`, `ai`, `nav` |
| POST `/api/heartbeat` | Refresh operator dashboard heartbeat |

POST requires `X-Dashboard-Token`. The dashboard starts a separate process group for each child and captures stdout/stderr. It shows navigation phase by parsing controller log strings (not a dedicated state topic). Log buffer holds up to 500 entries; camera event history up to 12. Inspect once invokes the Trigger service without publishing motion. The UI separates the most recent AI result from live robot state.

Interlocks: navigation start requires managed bridge and AI, fresh encoder and live preview, available service, no pending manual inspection, one encoder publisher, no competing command publisher and expected command subscribers (bridge + dashboard monitor). During navigation the monitor checks for heartbeat expiry (10 s), stale feedback, child exit, stale graph and unexpected ROS endpoints. Stop tries SIGINT, then SIGTERM and SIGKILL if required. Stopping either required node first stops navigation. Dashboard closure handles SIGINT/SIGTERM/SIGHUP and stops managed children. A process crash or power interruption may prevent cleanup, so independent ESP32 watchdog and a physical motor-power disconnect remain essential. The ROS/HTTP checks are software interlocks, not certified safety controls.

## 7. Custom OS integration checklist

The custom OS has not been provided or tested; this is an integration specification, not a claim that the current code runs on it. The exact OS architecture determines whether these components can run directly or require a Linux/ROS companion.

1. **Platform:** retain a ROS 2 Jazzy-compatible Linux userland and Python ABI, V4L2 USB support, UART drivers and device permissions. If the homemade OS is not Linux/ROS-compatible, keep the Pi Linux stack as a companion and bridge the documented ROS/serial interfaces, or port them explicitly.
2. **Provision:** clone the repository; install ROS messages and Python dependencies; obtain the licensed AI assets and create the venv. Record versions, model hash and label ordering in the OS build manifest. Verify model paths rather than relying on a clone alone.
3. **Configure hardware:** reproduce the UART configuration and wiring, verify camera path, ensure a common signal ground and correct voltage levels, and use the flashed single-motor firmware. Maintain one UART owner and one camera owner.
4. **Configure process supervision:** either retain V2 as supervisor (recommended for initial integration) or replace its process management deliberately. Its `SPECS` uses `/usr/bin/python3`, repository-relative paths and the home-directory AI venv; adjust these if the OS changes layout. Do not start the same nodes independently via systemd and dashboard buttons.
5. **Boot behavior:** if adding a service manager, start only the dashboard at boot; motion must remain disabled until explicit operator confirmation. Keep loopback/SSH access or implement a reviewed authenticated alternative. Auto-start and passwordless SSH are not configured yet.
6. **Interface compatibility:** preserve ROS topic names, message types, Trigger JSON keys, snapshot IDs, QoS, UART bytes and watchdog timing. The dashboard's publisher/subscriber-count interlock assumes the current graph; extra observers or a different supervisor may require a deliberate redesign rather than disabling safety checks.
7. **State reporting:** future improvement is a dedicated typed navigation-state/actual-motor-feedback topic instead of parsing logs. Preserve a distinction between command, measured encoder movement, AI validity and faults.
8. **Acceptance tests:** first run with battery disconnected. Check dashboard/SSH, encoder telemetry, live camera, Inspect once, snapshot/result ID match, automatic three-cycle log sequence and stop behavior. Test camera unplug, encoder feedback loss, browser closure/heartbeat expiry, competing publishers and process shutdown under controlled conditions. Then use a secured elevated motor and fused supply for powered testing. Only after bench validation integrate into a mobile platform with appropriate independent safety controls.

## 8. Validation record and limitations

| Item | Evidence / status |
|---|---|
| UART and encoder | Tested; example ROS count 447 after node restart |
| Single motor physical spin and stop | Confirmed in bounded manual and single AI-triggered tests |
| Three-cycle synchronized automatic logic | Logs show three successful inspections, three two-second motion command periods, final `ALL CYCLES COMPLETED`; battery disconnected |
| Dashboard V2 | Pi server started after resolving port conflict; user confirmed operation. No exhaustive hardware/failure-mode test recorded |
| AI model execution | TFLite runs on Pi and publishes tomato-class results |
| Disease diagnosis / leaf rejection | Not validated; arbitrary scenes can be classified as tomato diseases |
| Full powered automatic three-cycle run | Pending confirmation |
| Model redistribution / clean OS install | Pending; weights and metadata not included in confirmed commit |
| External ESP32 supply | Discussed, not validated |
| Encoder-distance travel / stall detection | Not implemented |
| Automatic plant detection / obstacle avoidance / row completion | Not implemented in bench controller |
| Persistent inspection logging | Not implemented; dashboard history is in memory |
| Boot service, direct LAN authentication, custom OS deployment | Not configured or tested |

The prototype is a supervised single-motor bench integration, not yet a field-ready autonomous robot. Earlier USB/driver power intermittency was resolved for testing but root cause was not conclusively established. Preserve the original code and the working baseline while making incremental changes. Keep the motor battery disconnected during software changes and never rely on software Stop as the sole emergency stop.

## 9. Git and handoff policy

The confirmed first push advanced `main` from `e93f358` to `5669cad`. Explicitly stage the two dashboard scripts, new live camera service and the two handover documents in the next commit. Do not use `git add .` because the Pi contains untracked test images and local model assets. Check `git diff --cached --name-status` before committing and push normally, never force-push. Updating `README_SINGLE_MOTOR.md` is intentional; leave the original top-level README and original implementation unchanged. Verify remote commit after pushing. Keep credentials, tokens, local captures and model files out of Git until their intended handling is agreed. The next commit hash is not yet known.
