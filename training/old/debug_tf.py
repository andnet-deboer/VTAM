#!/usr/bin/env python3
import os
import numpy as np
from mcap_ros2.reader import read_ros2_messages

def analyze_mcap_timing(mcap_path):
    # Full chain as discovered in your monitor debug
    tf_chain = [
        ("base_link", "link_mast"), ("link_mast", "link_head"),
        ("link_head", "link_head_pan"), ("link_head_pan", "link_head_tilt"),
        ("link_head_tilt", "camera_bottom_screw_frame"),
        ("camera_bottom_screw_frame", "camera_link"),
        ("camera_link", "camera_color_frame"),
        ("camera_color_frame", "camera_color_optical_frame"),
        ("camera_color_optical_frame", "umi_disconnect"),
        ("umi_disconnect", "umi_gripper")
    ]

    stats = {pair: {"count": 0, "start": None, "end": None, "static": False} for pair in tf_chain}
    pulse_ts = []
    
    print(f"--- ANALYZING: {os.path.basename(mcap_path)} ---")

    for msg in read_ros2_messages(mcap_path):
        topic = msg.channel.topic
        t = msg.publish_time_ns / 1e9

        if topic == "/sync_pulse":
            pulse_ts.append(t)
        elif topic in ["/tf", "/tf_static"]:
            is_static = (topic == "/tf_static")
            for tf in msg.ros_msg.transforms:
                pair = (tf.header.frame_id, tf.child_frame_id)
                if pair in stats:
                    stats[pair]["count"] += 1
                    stats[pair]["static"] = is_static or stats[pair]["static"]
                    if stats[pair]["start"] is None: stats[pair]["start"] = t
                    stats[pair]["end"] = t

    # Analysis Reporting
    if not pulse_ts:
        print("!! CRITICAL: No /sync_pulse found. Synchronization is impossible.")
    else:
        print(f"Sync Pulses: {len(pulse_ts)} (Start: {pulse_ts[0]:.3f}, End: {pulse_ts[-1]:.3f})")

    print("\nChain Link Consistency Check:")
    print(f"{'Link Pair':<60} | {'Count':<6} | {'Static?':<7} | {'Delay(s)':<8}")
    print("-" * 90)

    for pair in tf_chain:
        s = stats[pair]
        if s["count"] == 0:
            status = "MISSING"
            delay = "N/A"
        else:
            status = "FOUND"
            # Delay relative to the first sync pulse
            delay = f"{s['start'] - pulse_ts[0]:.3f}" if pulse_ts else "N/A"
        
        pair_str = f"{pair[0]} -> {pair[1]}"
        print(f"{pair_str:<60} | {s['count']:<6} | {str(s['static']):<7} | {delay:<8}")

    # Root Cause Deductions
    print("\n--- ROOT CAUSE DEDUCTION ---")
    if pulse_ts:
        for pair in tf_chain:
            if stats[pair]["count"] > 0 and stats[pair]["start"] > pulse_ts[-1]:
                 print(f"!! TIMING GAP: {pair[1]} started AFTER the last sync pulse. Data is unreachable.")
            elif stats[pair]["count"] > 0 and stats[pair]["start"] > (pulse_ts[0] + 1.0):
                 print(f"?? WARMUP: {pair[1]} arrived {stats[pair]['start'] - pulse_ts[0]:.2f}s late.")

if __name__ == "__main__":
    path = "data/raw/demo/demo_20260216_020448/demo_20260216_020448_0.mcap"
    analyze_mcap_timing(path)