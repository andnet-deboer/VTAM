#!/usr/bin/env python3
import os
import zarr
import numpy as np
import argparse
import numcodecs
from mcap_ros2.reader import read_ros2_messages

class ZarrSynchronizer:
    def __init__(self):
        self.sync_topic = "/sync_pulse"
        self.topic_map = {
            "/umi_gripper": "obs/ee_pose",
            "/joint_states": "obs/joint_states",
            "/tactile_gripper_controller": "obs/tactile_umi",
            "/camera_arm/color/image_rect_raw/compressed": "obs/video_wrist"
        }

    def process(self, input_path):
        zarr_path = input_path.replace(".mcap", ".zarr")
        print(f"Synchronizing Raw Data: {os.path.basename(input_path)}")

        data_buffer = {k: [] for k in self.topic_map.values()}
        ts_buffer = {k: [] for k in self.topic_map.values()}
        pulse_ts = []

        for msg in read_ros2_messages(input_path):
            topic = msg.channel.topic
            t = msg.publish_time_ns / 1e9

            if topic == self.sync_topic:
                pulse_ts.append(t)
            elif topic in self.topic_map:
                key = self.topic_map[topic]
                
                # Extract numerical data to ensure training compatibility
                if topic == "/joint_states":
                    data_buffer[key].append(np.array(msg.ros_msg.position, dtype=np.float32))
                elif topic == "/umi_gripper":
                    p = msg.ros_msg.pose.position
                    q = msg.ros_msg.pose.orientation
                    data_buffer[key].append(np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w], dtype=np.float32))
                elif "image" in topic or "compressed" in topic:
                    data_buffer[key].append(np.frombuffer(msg.ros_msg.data, dtype=np.uint8))
                else:
                    data_buffer[key].append(np.array(msg.ros_msg.data, dtype=np.float32))
                
                ts_buffer[key].append(t)

        if not pulse_ts:
            print("Error: No sync pulses found.")
            return

        store = zarr.DirectoryStore(zarr_path)
        root = zarr.group(store=store, overwrite=True)
        obs_group = root.create_group("obs")
        
        print(f"Processing {len(pulse_ts)} frames...")
        
        # Use Pickle codec to handle mixed types/bytes in Zarr
        object_codec = numcodecs.Pickle()

        for topic, key in self.topic_map.items():
            dataset_name = key.replace("obs/", "")
            
            if not data_buffer[key]:
                print(f"Warning: {topic} is empty. Padding with None.")
                synced_data = [None] * len(pulse_ts)
            else:
                indices = np.searchsorted(ts_buffer[key], pulse_ts)
                indices = np.clip(indices, 0, len(data_buffer[key]) - 1)
                synced_data = [data_buffer[key][i] for i in indices]
            
            # Standardize as numpy array
            try:
                final_array = np.array(synced_data)
                obs_group.create_dataset(
                    dataset_name, 
                    data=final_array,
                    chunks=(100, *final_array.shape[1:]),
                    overwrite=True
                )
            except (ValueError, TypeError):
                # Fallback to object codec for complex types (like image bytes)
                obs_group.create_dataset(
                    dataset_name, 
                    data=np.array(synced_data, dtype=object),
                    chunks=(100,),
                    overwrite=True,
                    object_codec=object_codec
                )

        print(f"Zarr Bake Complete: {os.path.basename(zarr_path)}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mcap", type=str, required=True)
    args = parser.parse_args()
    ZarrSynchronizer().process(args.mcap)