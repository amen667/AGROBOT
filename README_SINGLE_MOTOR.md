# AGROBOT — Dashboard quick start

Single-motor bench prototype · Raspberry Pi 4 · ROS 2 Jazzy · ESP32 · Mission Control V2

For architecture, dependencies, ROS interfaces, OS integration and validation status, see [AGROBOT_OS_HANDOVER.md](AGROBOT_OS_HANDOVER.md). The original team README and implementation are unchanged.

## 1. Open the dashboard

With motor power disconnected, connect the hardware as shown below and power the Pi and ESP32. The laptop and Pi must be able to reach each other over the network. The Pi already has ROS, the AI environment, model files and firmware configured.

**On the laptop (not inside an existing SSH session):**

```bash
ssh -t -o ExitOnForwardFailure=yes -L 8765:127.0.0.1:8765 amen@192.168.1.134 'source /opt/ros/jazzy/setup.bash && python3 ~/AGROBOT/agrobot_dashboard_v2.py'
```

Enter the Pi SSH password. After `AGROBOT dashboard ready`, open **http://localhost:8765** in the laptop browser. Keep the SSH terminal open. If the Pi's IP changes, substitute the new address. No separate node-start commands are needed.

| Button | Operation |
|---|---|
| Start bridge + AI | Start motor communication and live camera/AI service |
| Inspect once (no movement) | Capture a photo and classify it; no navigation command |
| Start 3 cycles | After live camera and encoder feedback are ready, confirm automatic navigation |
| Stop navigation | Stop the navigation process; verify actual motor stop |
| Stop all nodes | Stop all dashboard-managed processes |

The display shows live preview, inspection snapshot, capture/analysis events, prediction, confidence, inference time, encoder count, navigation phase and logs. The motor display represents a **command**, not verified motion. Current AI can classify unrelated objects as tomato disease: it is not a leaf detector.

Keep the battery disconnected for the first test. Only use powered motion with the wheel secured/elevated, supervision, an appropriately fused supply and a physical disconnect within reach. No obstacle avoidance is implemented. Do not run another bridge, camera service or movement publisher simultaneously.

## 2. Wiring

All GPIO numbers below are **GPIO numbers**, except the Raspberry Pi physical header pins explicitly identified in the first table. Verify the labels printed on your own hardware; wire colors and module revisions can differ.

### 2.1 Raspberry Pi to ESP32 — hardware UART

| Raspberry Pi physical pin | Raspberry Pi signal | ESP32 pin | Direction / purpose |
| --- | --- | --- | --- |
| 6 | GND | GND | Common signal ground |
| 8 | GPIO14 / TXD | GPIO16 / RX2 | Pi transmits to ESP32 |
| 10 | GPIO15 / RXD | GPIO17 / TX2 | ESP32 transmits to Pi |

- UART device on the configured Pi: `/dev/ttyAMA0`.
- Baud rate: `115200`, 8 data bits, no parity, 1 stop bit (8N1).
- UART is 3.3 V logic; TX and RX are crossed.
- In the tested setup the ESP32 is powered through USB from the Pi. The separate GPIO UART carries motor commands and feedback, not the USB serial port.
- Do not connect the Pi's 5 V or 3.3 V power rail to the ESP32 as an additional supply when it is already USB-powered. Turn off both boards before changing wiring, and avoid driving UART signals into an unpowered board.

### 2.2 ESP32 to BTS7960 / IBT-2 motor driver

| BTS7960 logic connector | ESP32 connection | Function |
| --- | --- | --- |
| RPWM | GPIO23 | Forward PWM |
| LPWM | GPIO18 | Reverse PWM |
| R_EN | 3.3 V | Right channel enable |
| L_EN | 3.3 V | Left channel enable |
| VCC | 3.3 V | Logic supply in the tested setup |
| GND | GND | Logic ground |
| R_IS | Not connected | Current sense (not implemented) |
| L_IS | Not connected | Current sense (not implemented) |

**Hardware-specific caution:** The table records the 3.3 V logic wiring actually used on this bench module. It is not a universal BTS7960 module specification. Confirm the logic supply and input thresholds for your exact module before replicating this circuit. A different IBT-2 revision may require a different logic supply or level shifting. Never apply 5 V to an ESP32 or Pi GPIO.

### 2.3 BTS7960 motor power and motor output

| BTS7960 terminal | Connection |
| --- | --- |
| B+ | LiPo positive, through an appropriately rated fuse / disconnect |
| B− | LiPo negative |
| M+ | Motor red power wire |
| M− | Motor white power wire |

The motor power supply is separate from the Pi and ESP32 logic supply. The LiPo must never be connected directly to either board. Keep the driver's ground reference common with the ESP32 and Pi; do not route motor current through a GPIO ground wire.

### 2.4 Motor encoder to ESP32

| Encoder wire on tested motor | ESP32 connection | Purpose |
| --- | --- | --- |
| Black | GND | Encoder ground |
| Blue | 3.3 V | Encoder supply |
| Yellow | GPIO21 | Encoder channel A (interrupt input) |
| Green | GPIO22 | Encoder channel B (direction input) |

The encoder was successfully tested at 3.3 V. Verify that the encoder supply and output signal levels are appropriate for the ESP32 before using a replacement motor. The sign of the encoder count depends on rotation direction and wiring; a negative count is not automatically a fault.

### 2.5 USB connections

| Device | Connection |
| --- | --- |
| USB camera | Raspberry Pi USB port |
| ESP32 | Raspberry Pi USB port for power in this setup |
| Raspberry Pi | Suitable dedicated Pi power supply |

### 2.6 ESP32 firmware settings

New firmware file: `firmware_esp32/single_motor_controller/single_motor_controller.ino`.

| Setting | Value |
| --- | --- |
| Forward PWM pin | GPIO23 |
| Reverse PWM pin | GPIO18 |
| Encoder A / B | GPIO21 / GPIO22 |
| UART2 RX / TX | GPIO16 / GPIO17 |
| Serial baud rate | 115200 |
| Motor PWM value | 130 (about 51% of an 8-bit PWM range) |
| Movement commands | `f` (forward), `b` (backward), `s` (stop) |
| Firmware command watchdog | 300 ms |
| Encoder telemetry | Approximately every 100 ms |
| Telemetry format | `ENC:<signed count> ACTIVE:<0 or 1>` |

The current firmware is already flashed on the tested ESP32. Reflashing is not required for normal startup. The PWM setting is not a calibrated vehicle speed.


### 2.7 Optional external ESP32 power (not yet validated)

A regulated 5 V USB power supply can power the ESP32 instead of the Pi USB port. Alternatively use a regulated 5 V buck converter into the development board's designated 5V/VIN input **only if the exact board supports that input**. Disconnect the original USB power first; verify the converter output with a multimeter. Retain common ground and the existing 3.3 V UART wiring. Do not connect raw 3S LiPo voltage to the ESP32; do not backfeed either board's GPIO when it is unpowered. External power does not change the UART software configuration. Keep motor-current paths separate from signal grounds.

## 3. SSH and troubleshooting commands

For maintenance only, open a separate laptop terminal:

```bash
ssh amen@192.168.1.134
```

The Pi prompt is `amen@amen:~$`. These commands run **on the Pi** (not required for normal dashboard startup):

```bash
source /opt/ros/jazzy/setup.bash
cd ~/AGROBOT
ros2 node list
sudo ss -ltnp 'sport = :8765'
ls -l /dev/ttyAMA0 /dev/video*
```

If port 8765 is occupied, stop all nodes in the old dashboard and close its SSH session. On the Pi, use the `ss` command above, check the process identity with `ps -fp PID` (replace PID with the actual number), and terminate only the confirmed old dashboard using `kill -TERM PID`. Check that the port is free and that no stale robot processes remain before restarting. Keep the motor supply disconnected during troubleshooting. If the browser still shows the old UI, refresh with Ctrl+F5.

## 4. Shutdown

1. Press **Stop all nodes** in the dashboard and verify motor stop.
2. Disconnect the motor battery safely.
3. Press Ctrl+C in the dashboard SSH terminal. Close the browser.
4. To power off the Pi, connect with normal SSH and run `sudo shutdown -h now`. Wait for shutdown before disconnecting power; avoid driving UART pins into an unpowered board.

Dashboard startup is currently manual via the single SSH command. Boot-time launch and direct LAN access are not configured.
