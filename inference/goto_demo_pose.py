#!/usr/bin/env python3
"""
goto_demo_pose.py — Move Stretch to a consistent neutral pose before replay.

Neutral pose (from differential_ik.py neutral_q):
    lift:        0.90 m
    arm:         0.05 m (slightly extended)
    wrist_yaw:   0.0  rad
    wrist_pitch: 0.0  rad (flat, horizontal)
    wrist_roll:  0.0  rad
    gripper:     open (1.1 rad)
    base:        no motion

USAGE:
    python3 goto_demo_pose.py

RUN ON: Robot (stretch-se3-3047), vtam_robot_node.py must be running.
"""

import zmq
import pickle
import time
import numpy as np

ROBOT_IP       = "localhost"
ZMQ_ACTION_PORT = 4402

DEMO_POSE = np.array([
    0.0,    # base_x vel — no base motion
    0.0,    # base_y
    0.0,    # base_theta vel — no base motion
    0.875,   # lift
    0.04,   # arm extension
    0.0,    # wrist_roll
    -0.1,    # wrist_pitch — flat/horizontal
    0.0,    # wrist_yaw
    1.05,    # gripper — open
], dtype=np.float32)

def main():
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.connect(f"tcp://{ROBOT_IP}:{ZMQ_ACTION_PORT}")
    time.sleep(0.5)  # let ZMQ settle

    print("Moving to demo pose...")
    print(f"  lift:        {DEMO_POSE[3]:.2f} m")
    print(f"  arm:         {DEMO_POSE[4]:.3f} m")
    print(f"  wrist_pitch: {DEMO_POSE[6]:.2f} rad")
    print(f"  wrist_yaw:   {DEMO_POSE[7]:.2f} rad")
    print(f"  wrist_roll:  {DEMO_POSE[5]:.2f} rad")
    print(f"  gripper:     open ({DEMO_POSE[8]:.1f})")

    # Send several times to ensure delivery over ZMQ PUB
    msg = pickle.dumps({"joint": DEMO_POSE.tolist()})
    for _ in range(15):
        sock.send(msg)
        time.sleep(0.1)

    print("Done. Robot should be moving to demo pose.")
    sock.close()
    ctx.term()

if __name__ == "__main__":
    main()