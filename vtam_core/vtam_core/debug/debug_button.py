#!/usr/bin/env python3
"""
Debug script to test button detection from eFlesh board.
No ROS required. Just reads serial and prints button state.
"""

import serial
import struct
import sys
import time

PORT = '/dev/serial/by-id/usb-Adafruit_QT_Py_M0_13CEB8D550555450382E3120FF140A34-if00'
BAUD = 115200
NUM_MAGS = 5
MAG_FRAME_SIZE = NUM_MAGS * 16  # 5 mags × 4 floats × 4 bytes
EXPECTED_FRAME_SIZE = MAG_FRAME_SIZE + 1  # +1 button byte

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else PORT

    print(f"Opening {port} @ {BAUD}")
    ser = serial.Serial(port, BAUD, timeout=0.1)

    print("Draining startup text (3s)...")
    time.sleep(3.0)
    ser.reset_input_buffer()

    print(f"Expected frame size: {EXPECTED_FRAME_SIZE} bytes")
    print("Listening for frames... Press Ctrl+C to quit.\n")

    buf = b''
    frame_count = 0
    bad_count = 0

    try:
        while True:
            buf += ser.read(ser.in_waiting or 1)
            while b'\r\n' in buf:
                frame, buf = buf.split(b'\r\n', 1)
                frame_count += 1

                if len(frame) == EXPECTED_FRAME_SIZE:
                    btn = frame[MAG_FRAME_SIZE]
                    sensor_bytes = frame[:MAG_FRAME_SIZE]
                    values = struct.unpack(f'<{NUM_MAGS * 4}f', sensor_bytes)

                    # Only print when button is pressed, or every 100th frame as heartbeat
                    if btn == 1:
                        print(f"[Frame {frame_count}] *** BUTTON PRESSED *** (btn byte = {btn})")
                    elif frame_count % 100 == 0:
                        print(f"[Frame {frame_count}] btn={btn}  mag0_xyz=({values[1]:.1f}, {values[2]:.1f}, {values[3]:.1f})")
                else:
                    bad_count += 1
                    if bad_count <= 20:
                        print(f"[Frame {frame_count}] BAD SIZE: got {len(frame)} bytes, expected {EXPECTED_FRAME_SIZE}")
                        print(f"  hex: {frame[:40].hex()}...")
                        # Try to decode as ASCII in case it's startup text
                        try:
                            text = frame.decode('ascii')
                            print(f"  ascii: {text}")
                        except:
                            pass

    except KeyboardInterrupt:
        print(f"\n\nDone. Total frames: {frame_count}, bad: {bad_count}")
    finally:
        ser.close()

if __name__ == '__main__':
    main()