"""
Sends commands to the IDEX/Tic controller Arduino over USB serial.
Moves the stepper 5 full turns at 1 rev/sec, then de-energizes.

Motor: 200 full steps/rev, 1/2 stepping = 400 microsteps/rev
Speed: 1 rev/sec = 4,000,000 pulses/10,000s
Position: 5 revs = 2,000 microsteps

Install pyserial first:  pip install pyserial
"""

import serial
import time

# Update this to match your Arduino's port.
# Run: ls /dev/tty.usb*  in Terminal to find it.
PORT = "/dev/tty.usbmodem1201"
BAUD = 115200

def send_cmd(ser, cmd):
    ser.write((cmd + "\n").encode())
    time.sleep(0.1)
    # Read and print any response from the controller
    while ser.in_waiting:
        print(ser.readline().decode(), end="")

def wait_for_move(ser, timeout=60):
    """Poll status to keep Tic alive and wait until motor reaches target."""
    start = time.time()
    has_moved = False  # Don't check for zero velocity until motor has started
    while time.time() - start < timeout:
        ser.write(b"SR\n")
        time.sleep(0.5)
        while ser.in_waiting:
            line = ser.readline().decode()
            print(line, end="")
            if "Current velocity:" in line:
                # Extract velocity value
                try:
                    vel = int(line.split(":")[1].strip().split()[0])
                except (ValueError, IndexError):
                    vel = None
                if vel is not None and vel != 0:
                    has_moved = True
                if has_moved and vel == 0:
                    return True
    print("Warning: Timed out waiting for move to complete")
    return False

def main():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)  # Wait for Arduino to reset after serial connection

    # Drain any startup messages
    while ser.in_waiting:
        print(ser.readline().decode(), end="")

    print("--- Zeroing position ---")
    send_cmd(ser, "SC")

    print("--- Energizing motor ---")
    send_cmd(ser, "SO")

    print("--- Stopping any residual movement ---")
    send_cmd(ser, "SS")

    print("--- Setting max speed (1 rev/sec) ---")
    send_cmd(ser, "SM4000000")

    print("--- Setting acceleration ---")
    send_cmd(ser, "SA500000")

    print("--- Moving 5 revolutions ---") # 16200 is 5000uL of volume
    send_cmd(ser, "SP32400")

    # Poll status to keep Tic alive until move completes
    print("--- Waiting for move to complete ---")
    wait_for_move(ser)

    print("--- De-energizing motor ---")
    send_cmd(ser, "SF")

    ser.close()
    print("Done.")

if __name__ == "__main__":
    main()
