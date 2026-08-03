import glob
import re
import time
import serial
import sys

def find_port():
    ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    ports = sorted(ports)
    print("PORTS:", ports)

    if not ports:
        raise SystemExit("NO ESP32 PORT FOUND")

    if len(ports) > 1:
        print("ERROR: More than one USB serial device.")
        print("For this motor test, unplug LiDAR and keep ONLY ESP32 plugged.")
        raise SystemExit

    return ports[0]

def open_esp32(port):
    print("OPENING:", port)

    ser = serial.Serial()
    ser.port = port
    ser.baudrate = 115200
    ser.timeout = 0.2

    # try to avoid extra ESP32 reset from serial control lines
    ser.dtr = False
    ser.rts = False

    ser.open()
    time.sleep(0.5)
    return ser

def read_line(ser):
    try:
        return ser.readline().decode(errors="ignore").strip()
    except Exception:
        return ""

def wait_ready(ser):
    print("WAITING FOR ESP32 READY...")

    start = time.time()
    last_print = 0

    while time.time() - start < 25:
        line = read_line(ser)
        if line:
            print(line)

        if line.startswith("L:") or "AGROBOT" in line or "Commands:" in line:
            print("ESP32 READY ✅")
            return True

        if time.time() - last_print > 5:
            last_print = time.time()
            ser.write(b"s")
            ser.flush()

    return False

def parse_ticks(line):
    m = re.search(r"L:\s*(-?\d+)\s+R:\s*(-?\d+)", line)
    if not m:
        return None
    return int(m.group(1)), int(m.group(2))

def collect_ticks(ser, seconds):
    end = time.time() + seconds
    last = None
    lines = []

    while time.time() < end:
        line = read_line(ser)
        if line:
            lines.append(line)
            ticks = parse_ticks(line)
            if ticks is not None:
                last = ticks

    return last, lines

def send_many(ser, ch, n, delay=0.04):
    for _ in range(n):
        ser.write(ch.encode())
        ser.flush()
        time.sleep(delay)

def stop(ser):
    send_many(ser, "s", 10, 0.04)

def boost_full(ser):
    send_many(ser, "+", 10, 0.04)

def set_speed(ser, target):
    # reset speed to 0
    send_many(ser, "-", 20, 0.01)

    # ESP32 speed increases by 20 each "+"
    steps = int(target / 20)
    send_many(ser, "+", steps, 0.01)

def move(ser, cmd, duration, speed):
    print("READ BEFORE...")
    before, _ = collect_ticks(ser, 0.5)

    print("SET SPEED:", speed)
    set_speed(ser, speed)

    print("MOVE:", cmd, "duration:", duration)
    ser.write(cmd.encode())
    ser.flush()

    time.sleep(duration)

    print("STOP")
    stop(ser)

    after, lines = collect_ticks(ser, 1.0)

    for line in lines[-12:]:
        print(line)

    print("TICKS BEFORE:", before)
    print("TICKS AFTER :", after)

    if before is not None and after is not None and before != after:
        print("RESULT: MOTOR MOVED ✅")
    elif after is not None:
        print("RESULT: COMMAND SENT BUT NO MOVEMENT ❌")
    else:
        print("RESULT: NO SERIAL TICKS READ ❌")

port = find_port()
ser = open_esp32(port)

while not wait_ready(ser):
    print("")
    print("ESP32 did not become ready.")
    print("Press EN/RST button on ESP32 once, wait 3 seconds, then press ENTER here.")
    input()
    ser.close()
    time.sleep(1)
    ser = open_esp32(port)

print("")
print("STABLE MOTOR CONSOLE")
print("Commands:")
print("  f      forward 1s speed 120")
print("  f 1    forward 1s")
print("  b      backward 1s speed 120")
print("  l      left 0.45s speed 120")
print("  r      right 0.45s speed 120")
print("  s      stop")
print("  q      quit")
print("")
print("IMPORTANT: do not close this console between movements.")
print("")

while True:
    user = input("drive> ").strip().lower()

    if not user:
        continue

    parts = user.split()
    cmd = parts[0]

    if cmd == "q":
        stop(ser)
        break

    if cmd == "s":
        stop(ser)
        print("STOP sent")
        continue

    if cmd not in ["f", "b", "l", "r"]:
        print("Use only: f b l r s q")
        continue

    if cmd == "f":
        default_duration = 1.0
        speed = 120
    elif cmd == "b":
        default_duration = 1.0
        speed = 120
    else:
        default_duration = 0.45
        speed = 120

    if len(parts) >= 2:
        duration = float(parts[1])
    else:
        duration = default_duration

    move(ser, cmd, duration, speed)

ser.close()
print("DONE")
