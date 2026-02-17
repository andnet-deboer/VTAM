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
            "/camera_arm/color/image_rect_raw/compressed": "obs/video_wrist"
        }

        # TF chain for EE pose: base_link - umi_disconnect - umi_gripper
        self.tf_chain = [
            ("base_link", "umi_disconnect"),
            ("umi_disconnect", "umi_gripper"),
        ]

    @staticmethod
    def tf_to_matrix(pose7):
        """Convert [x,y,z,qx,qy,qz,qw] to 4x4 homogeneous matrix."""
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat(pose7[3:]).as_matrix()
        T[:3, 3] = pose7[:3]
        return T

    def extract_ee_pose(self, tf_data, tf_ts):
        """Chain TF transforms into ee_pose (7,) vectors."""
        missing = [p for p in self.tf_chain if len(tf_data[p]) == 0]
        if missing:
            print(f"Warning: Missing TF frames: {missing}. ee_pose will be empty.")
            return [], []

        ee_pose_data = []
        ee_pose_ts = []

        base_pair = self.tf_chain[0]
        # Pre-convert downstream pair timestamps to arrays for searchsorted
        downstream_ts = {p: np.array(tf_ts[p]) for p in self.tf_chain[1:]}

        for i in range(len(tf_data[base_pair])):
            t = tf_ts[base_pair][i]
            T_chain = self.tf_to_matrix(tf_data[base_pair][i])

            for pair in self.tf_chain[1:]:
                idx = np.searchsorted(downstream_ts[pair], t)
                idx = np.clip(idx, 0, len(tf_data[pair]) - 1)
                T_chain = T_chain @ self.tf_to_matrix(tf_data[pair][idx])

            pos = T_chain[:3, 3]
            quat = Rotation.from_matrix(T_chain[:3, :3]).as_quat()  # XYZW
            ee_pose_data.append(np.concatenate([pos, quat]).astype(np.float32))
            ee_pose_ts.append(t)

        return ee_pose_data, ee_pose_ts

    def process(self, input_path):
        input_abs = os.path.abspath(input_path)
        input_filename = os.path.basename(input_abs)

        script_dir = os.path.dirname(os.path.abspath(__file__))
        vtam_root = os.path.abspath(os.path.join(script_dir, "..", ".."))

        path_parts = input_abs.split(os.sep)
        task_name = "default_task"
        if "raw" in path_parts:
            task_name = path_parts[path_parts.index("raw") + 1]

        processed_base = os.path.join(vtam_root, "data", "processed", task_name)
        os.makedirs(processed_base, exist_ok=True)

        zarr_filename = input_filename.replace(".mcap", ".zarr")
        zarr_path = os.path.join(processed_base, zarr_filename)

        print(f"--- VTAM PROCESSING PIPELINE ---")
        print(f"Input MCAP: {input_abs}")
        print(f"Output Zarr: {zarr_path}")

        # --- Buffers ---
        data_buffer = {k: [] for k in self.topic_map.values()}
        ts_buffer = {k: [] for k in self.topic_map.values()}
        pulse_ts = []

        tf_data = {pair: [] for pair in self.tf_chain}
        tf_ts = {pair: [] for pair in self.tf_chain}

        # --- Single pass over MCAP ---
        for msg in read_ros2_messages(input_abs):
            topic = msg.channel.topic
            t = msg.publish_time_ns / 1e9

            if topic == self.sync_topic:
                pulse_ts.append(t)
            elif topic == "/tf":
                for tf in msg.ros_msg.transforms:
                    pair = (tf.header.frame_id, tf.child_frame_id)
                    if pair in tf_data:
                        tr = tf.transform.translation
                        ro = tf.transform.rotation
                        tf_data[pair].append(np.array([
                            tr.x, tr.y, tr.z, ro.x, ro.y, ro.z, ro.w
                        ], dtype=np.float64))
                        tf_ts[pair].append(t)
            elif topic in self.topic_map:
                key = self.topic_map[topic]
                if "image" in topic or "compressed" in topic:
                    data_buffer[key].append(np.frombuffer(msg.ros_msg.data, dtype=np.uint8))
                else:
                    data_buffer[key].append(np.array(msg.ros_msg.data, dtype=np.float32))
                ts_buffer[key].append(t)

        if not pulse_ts:
            print("Error: No sync pulses found in MCAP.")
            return

        # --- Chain TF into ee_pose ---
        ee_data, ee_ts = self.extract_ee_pose(tf_data, tf_ts)
        data_buffer["obs/ee_pose"] = ee_data
        ts_buffer["obs/ee_pose"] = ee_ts

        print(f"  TF ee_pose: {len(ee_data)} samples extracted")
        print(f"Processing {len(pulse_ts)} synchronized frames...")

        # --- Write Zarr ---
        store = zarr.DirectoryStore(zarr_path)
        root = zarr.group(store=store, overwrite=True)
        obs_group = root.create_group("obs")

        object_codec = numcodecs.Pickle()
        all_keys = list(self.topic_map.values()) + ["obs/ee_pose"]

        for key in all_keys:
            dataset_name = key.replace("obs/", "")

            if not data_buffer[key]:
                print(f"Warning: {key} is empty. Padding with None.")
                synced_data = [None] * len(pulse_ts)
            else:
                indices = np.searchsorted(ts_buffer[key], pulse_ts)
                indices = np.clip(indices, 0, len(data_buffer[key]) - 1)
                synced_data = [data_buffer[key][i] for i in indices]

            try:
                final_array = np.array(synced_data)
                obs_group.create_dataset(
                    dataset_name,
                    data=final_array,
                    chunks=(100, *final_array.shape[1:]),
                    overwrite=True
                )
            except (ValueError, TypeError):
                obs_group.create_dataset(
                    dataset_name,
                    data=np.array(synced_data, dtype=object),
                    chunks=(100,),
                    overwrite=True,
                    object_codec=object_codec
                )

        print(f"Zarr Processing Complete: {os.path.basename(zarr_path)}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mcap", type=str, required=True)
    args = parser.parse_args()
    ZarrSynchronizer().process(args.mcap)