#!/usr/bin/env python3
from dynamixel_sdk import *

# Address for wrist on Robot
DEVICENAME = '/dev/hello-dynamixel-wrist'

# Address for U2S2 with my PC
DEVICENAME = '/dev/ttyUSB1'
BAUDRATE = 115200
PROTOCOL_VERSION = 2.0 

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if not (portHandler.openPort() and portHandler.setBaudRate(BAUDRATE)):
    print("Failed to initialize port.")
    quit()

print("Scanning IDs 1-20...")
for dxl_id in range(1, 21):
    model, result, error = packetHandler.ping(portHandler, dxl_id)
    if result == COMM_SUCCESS:
        print(f"FOUND: [ID:{dxl_id:03d}] Model: {model}")
    # Optional: if result == COMM_RX_CORRUPT: print(f"NOISE on ID {dxl_id}")

portHandler.closePort()