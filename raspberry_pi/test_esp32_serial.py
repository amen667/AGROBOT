import glob
import os
import time
import serial

BAUD = 115200


def find_esp32_port():
    by_id_ports = glob.glob("/dev/serial/by-id/*")

    for port in by_id_ports:
        name = os.path.basename(port).lower()
        if any(key in name for key in ["cp210", "ch340", "wch", "silicon", "uart", "usb"]):
            return port

    ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    if ports:
        return ports[0]

    return None


def read_available(ser):
    time.sleep(0.2)
    while ser.in_waiting:
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            print(line)


def main():
    port = find_esp32_port()

    if port is None:
        print("No ESP32 serial port found.")
        print("Check ESP32 USB connection.")
        print("Run: ls /dev/ttyUSB* /dev/ttyACM*")
        return

    print(f"Using port: {port}")

    try:
        ser = serial.Serial(port, BAUD, timeout=1)
    except PermissionError:
        print("Permission denied.")
        print(f"Run: sudo chmod a+rw {port}")
        return

    time.sleep(2)
    ser.reset_input_buffer()

    print("Connected to ESP32.")
    print("Commands: f b l r s + - z")
    print("q = quit")

    while True:
        cmd = input("Command: ").strip()

        if cmd == "":
            continue

        if cmd == "q":
            ser.write(b"s")
            print("Stop sent. Exiting.")
            break

        if cmd in ["f", "b", "l", "r", "s", "+", "-", "z"]:
            ser.write(cmd.encode())
            read_available(ser)
        else:
            print("Invalid command.")

    ser.close()


if __name__ == "__main__":
    main()
