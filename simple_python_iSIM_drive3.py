import serial
import time

PORT = '/dev/tty.usbmodem1301'   # Change to your port
BAUD = 115200

def read_response(ser, timeout=1.0):
    deadline = time.time() + timeout
    lines = []
    while time.time() < deadline:
        if ser.in_waiting:
            line = ser.readline().decode(errors='replace').strip()
            if line:
                print(f"  << {line}")
                lines.append(line)
                deadline = time.time() + 0.3  # extend if data is still coming
    return lines

def send(ser, cmd):
    print(f"\n>> {cmd}")
    ser.write((cmd + '\n').encode())
    return read_response(ser)

def wait_for_move(ser, timeout=120, poll_interval=0.5):
    """Poll SR until current velocity is 0 (move complete) or timeout."""
    print("\nWaiting for move to complete...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        time.sleep(poll_interval)
        lines = send(ser, 'SR')
        for line in lines:
            if 'Current velocity:' in line:
                # Line looks like: "Current velocity: 0 usteps/10000s"
                parts = line.split()
                try:
                    velocity = int(parts[2])
                    if velocity == 0:
                        print("Move complete.")
                        return True
                except (IndexError, ValueError):
                    pass
    print("WARNING: Move timed out.")
    return False

with serial.Serial(PORT, BAUD, timeout=1) as ser:
    ser.setDTR(False)   # Prevent Arduino reset on connect
    time.sleep(2.0)
    ser.flushInput()

    send(ser, 'SS')
    send(ser, 'SC')
    send(ser, 'SO')
    send(ser, 'SM3000000')
    send(ser, 'SA200000')
    send(ser, 'SC')
    send(ser, 'SP22000') # 2.977 counts per microliter

    wait_for_move(ser, timeout=120)

    send(ser, 'SF')
