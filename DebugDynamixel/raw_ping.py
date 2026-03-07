#!/usr/bin/env python3
from dynamixel_sdk import *

### Uncomment for each specifc test, and update the DEVICENAME as needed ##

# TEST #1: Address for wrist on Robot
DEVICENAME = '/dev/hello-dynamixel-wrist'

# TEST #2 Address for U2D2 with my PC
# DEVICENAME = '/dev/ttyUSB1'

# TEST #3: Address for wrist on Robot USB PORT
# DEVICENAME = '/dev/ttyUSB3'

BAUDRATE = 115200
PROTOCOL_VERSION = 2.0 

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if not (portHandler.openPort() and portHandler.setBaudRate(BAUDRATE)):
    print("Failed to initialize port.")
    quit()

print("Scanning IDs 1-20...")
for dxl_id in range(13, ):
    model, result, error = packetHandler.ping(portHandler, dxl_id)
    if result == COMM_SUCCESS:
        print(f"FOUND: [ID:{dxl_id:03d}] Model: {model}")

# dxl_id = 16
model, result, error = packetHandler.ping(portHandler, dxl_id)
if result == COMM_SUCCESS:
        print(f"FOUND: [ID:{dxl_id:03d}] Model: {model}")
portHandler.closePort()