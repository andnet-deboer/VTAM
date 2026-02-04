#!/usr/bin/env python3
import time
import sys
from stretch_body.dynamixel_XL430 import DynamixelXL430

# CONFIG
USB = '/dev/hello-dynamixel-wrist'
ID_GRIPPER = 14  # The Suspect
ID_YAW = 13      # The Control (Should stay stable)

print(f"--- STARTING CONNECTION STRESS TEST ---")
print(f"Target: Gripper (ID {ID_GRIPPER}) vs Yaw (ID {ID_YAW})")
print(f"USB: {USB}")
print("-------------------------------------------")

# Setup Motors (Low-level access only, no robot startup checks)
gripper = DynamixelXL430(ID_GRIPPER, USB)
yaw = DynamixelXL430(ID_YAW, USB)

fail_count = 0
total_count = 0

try:
    while True:
        total_count += 1
        
        # 1. Ping the Neighbor (Yaw) - Proves the bus is alive
        yaw_alive = yaw.do_ping()
        
        # 2. Ping the Gripper - The suspect
        gripper_alive = gripper.do_ping()
        
        # 3. Formulate Output
        status_msg = ""
        if yaw_alive and gripper_alive:
            status_msg = "\033[92m[PASS]\033[0m Both Motors Responding"
        elif yaw_alive and not gripper_alive:
            status_msg = "\033[91m[FAIL] YAW OK but GRIPPER DEAD (connector failure)\033[0m"
            fail_count += 1
        elif not yaw_alive:
            status_msg = "\033[93m[WARN] Entire Wrist Bus Lost (Short Circuit?)\033[0m"
            
        print(f"Test {total_count}: {status_msg}")
        
        # 4. Write to log file for your engineer
        with open("connector_failure_log.txt", "a") as f:
            t = time.strftime('%H:%M:%S')
            f.write(f"[{t}] Yaw:{yaw_alive} | Gripper:{gripper_alive}\n")
            
        time.sleep(0.1)

except KeyboardInterrupt:
    print(f"\nTest Complete. Gripper failed {fail_count} out of {total_count} times.")