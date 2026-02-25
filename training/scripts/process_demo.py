#!/usr/bin/env python3
import os
import zarr
import numpy as np
import argparse
import numcodecs
from scipy.spatial.transform import Rotation
from mcap_ros2.reader import read_ros2_messages

class ZarrSynchronizer:
    def __init__(self):
        self.sync_topic = "/sync_pulse"
        self.topic_map = {
            "/tactile_gripper_controller": "obs/tactile_umi",
            "/camera_arm/color/image_rect_raw/compressed": "obs/video_wrist",
            "/gripper_width_normalized": "obs/gripper_position",
        }

        # SIMPLIFIED CHAIN
        # The detector already localizes umi_disconnect to base_link.
        self.tf_chain = [
            ("base_link", "umi_disconnect"),
            ("umi_disconnect", "umi_gripper")
        ]

    @staticmethod
    def tf_to_matrix(pose7):
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat(pose7[3:]).as_matrix()
        T[:3, 3] = pose7[:3]
        return T

    def extract_ee_pose(self, tf_data, tf_ts):
        """Stitches the pre-localized detector frames into ee_pose."""
        # Check if the detector actually fired in this bag
        if not tf_data[self.tf_chain[0]]:
            print(f"Warning: No UMI detections found in this bag.")
            return [], []

        ee_pose_data, ee_pose_ts = [], []
        # Use the detector's output to drive the sync timing
        drive_pair = self.tf_chain[0]
        ts_arrays = {p: np.array(tf_ts[p]) for p in self.tf_chain}

        for i in range(len(tf_data[drive_pair])):
            t = tf_ts[drive_pair][i]
            T_total = self.tf_to_matrix(tf_data[drive_pair][i])

            # Apply the static offset to the gripper center
            for pair in self.tf_chain[1:]:
                idx = np.searchsorted(ts_arrays[pair], t)
                idx = np.clip(idx - 1, 0, len(tf_data[pair]) - 1)
                T_total = T_total @ self.tf_to_matrix(tf_data[pair][idx])

            pos = T_total[:3, 3]
            quat = Rotation.from_matrix(T_total[:3, :3]).as_quat()
            ee_pose_data.append(np.concatenate([pos, quat]).astype(np.float32))
            ee_pose_ts.append(t)

        return ee_pose_data, ee_pose_ts

    def process(self, input_path):
        input_abs = os.path.abspath(input_path)
        input_filename = os.path.basename(input_abs)
        script_dir = os.path.dirname(os.path.abspath(__file__))
        vtam_root = os.path.abspath(os.path.join(script_dir, "..", ".."))

        path_parts = input_abs.split(os.sep)
        task_name = path_parts[path_parts.index("raw") + 1] if "raw" in path_parts else "default_task"
        zarr_path = os.path.join(vtam_root, "data", "processed", task_name, input_filename.replace(".mcap", ".zarr"))
        os.makedirs(os.path.dirname(zarr_path), exist_ok=True)

        print(f"--- VTAM SIMPLIFIED PIPELINE ---\nInput: {input_abs}\nOutput: {zarr_path}")

        data_buffer = {k: [] for k in list(self.topic_map.values()) + ["obs/ee_pose"]}
        ts_buffer = {k: [] for k in list(self.topic_map.values()) + ["obs/ee_pose"]}
        pulse_ts, tf_data, tf_ts = [], {p: [] for p in self.tf_chain}, {p: [] for p in self.tf_chain}

        for msg in read_ros2_messages(input_abs):
            topic, t = msg.channel.topic, msg.publish_time_ns / 1e9
            if topic == self.sync_topic:
                pulse_ts.append(t)
            elif topic == "/tf":
                for tf in msg.ros_msg.transforms:
                    pair = (tf.header.frame_id, tf.child_frame_id)
                    if pair in tf_data:
                        tr, ro = tf.transform.translation, tf.transform.rotation
                        tf_data[pair].append(np.array([tr.x, tr.y, tr.z, ro.x, ro.y, ro.z, ro.w]))
                        tf_ts[pair].append(t)

            elif topic == "/gripper_width_normalized":
                val = np.array([msg.ros_msg.data], dtype=np.float32)
                data_buffer["obs/gripper_position"].append(val)
                ts_buffer["obs/gripper_position"].append(t)

            elif topic in self.topic_map:
                key = self.topic_map[topic]
                val = np.frombuffer(msg.ros_msg.data, dtype=np.uint8) if "compressed" in topic else np.array(msg.ros_msg.data, dtype=np.float32)
                data_buffer[key].append(val); ts_buffer[key].append(t)

        if not pulse_ts:
            print("Error: No sync pulses found."); return

        ee_data, ee_ts = self.extract_ee_pose(tf_data, tf_ts)
        data_buffer["obs/ee_pose"], ts_buffer["obs/ee_pose"] = ee_data, ee_ts
        print(f"  TF ee_pose: {len(ee_data)} samples extracted")

        # Write Zarr
        root = zarr.group(store=zarr.DirectoryStore(zarr_path), overwrite=True)
        obs_group = root.create_group("obs")
        for key in data_buffer.keys():
            name = key.replace("obs/", "")
            if not data_buffer[key]:
                synced = [None] * len(pulse_ts)
            else:
                idx = np.clip(np.searchsorted(ts_buffer[key], pulse_ts), 0, len(data_buffer[key]) - 1)
                synced = [data_buffer[key][i] for i in idx]
            try:
                obs_group.create_dataset(name, data=np.array(synced), chunks=(100, *np.array(synced).shape[1:]), overwrite=True)
            except:
                obs_group.create_dataset(name, data=np.array(synced, dtype=object), chunks=(100,), overwrite=True, object_codec=numcodecs.Pickle())

        print(f"Zarr Processing Complete: {os.path.basename(zarr_path)}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mcap", type=str, required=True)
    ZarrSynchronizer().process(parser.parse_args().mcap)