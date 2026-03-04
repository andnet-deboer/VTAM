#!/usr/bin/env python3
"""
chunk_bag.py — Split a session MCAP bag into per-episode MCAP bags.

Pipeline position:
    record_demo_node.py → session.mcap → chunk_bag.py → processed/episode_000.mcap, ...
                                                                ↓
                                                        process_demo.py

Episode boundaries are read from /recording/active:
    Bool(data=True)  — episode start
    Bool(data=False) — episode stop

Each output bag:
    - All original topics trimmed to the episode window
    - /progress (std_msgs/Float32) injected at each /sync_pulse timestamp [0.0 → 1.0]

Usage:
    python3 chunk_bag.py path/to/session/ data/processed/pickup_cup/

    Output: data/processed/pickup_cup/episode_000.mcap, episode_001.mcap, ...
"""

import argparse
import glob
import os
import struct
import yaml

from mcap.reader import make_reader
from mcap.writer import Writer
from mcap_ros2.reader import read_ros2_messages
from tqdm import tqdm

# ── Configuration ──────────────────────────────────────────────────────────────

EPISODE_TOPIC = "/recording/active"
SYNC_TOPIC    = "/sync_pulse"
CONFIG_PATH = os.path.expanduser("~/VTAM/vtam_core/config/record.yaml")

MIN_EPISODE_FRAMES = 30

def load_topics(config_path: str) -> list[str]:
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config['/**']['ros__parameters']['session_topics']

EPISODE_TOPICS = load_topics(CONFIG_PATH)

# ── Helpers ────────────────────────────────────────────────────────────────────

def resolve_mcap(path: str) -> str:
    if path.endswith(".mcap") and os.path.isfile(path):
        return path
    mcaps = glob.glob(os.path.join(path, "*.mcap"))
    if len(mcaps) == 1:
        return mcaps[0]
    if len(mcaps) > 1:
        raise ValueError(f"Multiple .mcap files found in {path}: {mcaps}")
    raise FileNotFoundError(f"No .mcap file found at {path}")


def index_episodes(mcap_path: str) -> list[tuple[int, int]]:
    episodes = []
    start_ns = None

    for msg in read_ros2_messages(mcap_path, topics=[EPISODE_TOPIC]):
        if msg.ros_msg.data is True and start_ns is None:
            start_ns = msg.publish_time_ns
        elif msg.ros_msg.data is False and start_ns is not None:
            episodes.append((start_ns, msg.publish_time_ns))
            start_ns = None

    if start_ns is not None:
        print("  WARNING: Last episode has no stop marker — discarding.")

    return episodes

def encode_float32(value: float) -> bytes:
    """Encode std_msgs/Float32 as CDR bytes."""
    return b'\x00\x01\x00\x00' + struct.pack('<f', value)


# ── Core ───────────────────────────────────────────────────────────────────────

def chunk_bag(bag_path: str, output_dir: str):
    mcap_path = resolve_mcap(bag_path)
    print(f"Source: {mcap_path}")

    episodes = index_episodes(mcap_path)
    if not episodes:
        print("No complete episodes found.")
        return
    print(f"Found {len(episodes)} episode(s).")

    # Read all messages once
    raw_messages = []   # (publish_time_ns, topic, raw_bytes)
    channel_meta = {}   # topic → (schema, channel)

    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for schema, channel, message in tqdm(
            reader.iter_messages(topics=EPISODE_TOPICS), desc="Reading"
        ):
            raw_messages.append((message.publish_time, channel.topic, message.data))
            if channel.topic not in channel_meta:
                channel_meta[channel.topic] = (schema, channel)

    os.makedirs(output_dir, exist_ok=True)

    valid_count = 0
    for ep_idx, (start_ns, end_ns) in enumerate(episodes):
        out_path = os.path.join(output_dir, f"episode_{ep_idx:03d}.mcap")

        # Slice messages to episode window
        episode_msgs = [
            (ts, topic, data)
            for ts, topic, data in raw_messages
            if start_ns <= ts <= end_ns
        ]

        # Count sync pulses — this is the true episode length
        sync_timestamps = sorted(
            ts for ts, topic, _ in episode_msgs if topic == SYNC_TOPIC
        )
        N = len(sync_timestamps)

        if N < MIN_EPISODE_FRAMES:
            print(f"  Episode {ep_idx}: {N} sync frames — discarding (too short)")
            continue

        # Build progress map: sync_ts_ns → progress_value [0.0, 1.0]
        progress_map = {
            ts: float(i) / max(N - 1, 1)
            for i, ts in enumerate(sync_timestamps)
        }

        # Write episode bag
        with open(out_path, "wb") as f:
            writer = Writer(f)
            writer.start()

            # Register original channels
            channel_ids = {}
            for topic, (schema, channel) in channel_meta.items():
                if any(t == topic for _, t, _ in episode_msgs):
                    sid = writer.register_schema(
                        name=schema.name,
                        encoding=schema.encoding,
                        data=schema.data,
                    )
                    cid = writer.register_channel(
                        topic=topic,
                        message_encoding=channel.message_encoding,
                        schema_id=sid,
                        metadata=dict(channel.metadata),
                    )
                    channel_ids[topic] = cid

            # Register /progress channel
            progress_schema_id = writer.register_schema(
                name="std_msgs/msg/Float32",
                encoding="ros2msg",
                data=b"float32 data\n",
            )
            progress_channel_id = writer.register_channel(
                topic="/progress",
                message_encoding="cdr",
                schema_id=progress_schema_id,
            )

            # Write all messages in time order
            for ts, topic, data in sorted(episode_msgs, key=lambda x: x[0]):
                if topic not in channel_ids:
                    continue

                writer.add_message(
                    channel_id=channel_ids[topic],
                    log_time=ts,
                    data=data,
                    publish_time=ts,
                )

                # Inject /progress alongside each /sync_pulse message
                if topic == SYNC_TOPIC:
                    writer.add_message(
                        channel_id=progress_channel_id,
                        log_time=ts,
                        data=encode_float32(progress_map[ts]),
                        publish_time=ts,
                    )

            writer.finish()

        print(f"  Episode {ep_idx}: {N} frames → {out_path}")
        valid_count += 1

    print(f"\nDone. {valid_count}/{len(episodes)} episodes saved to {output_dir}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Split a session bag into episode bags.")
    parser.add_argument("demo_name", type=str, help="Demo name (e.g. default_demo)")
    parser.add_argument("--session", type=str, default=None, help="Specific session folder (optional)")
    args = parser.parse_args()

    raw_base  = os.path.expanduser(f"~/VTAM/data/raw/{args.demo_name}")
    out_base  = os.path.expanduser(f"~/VTAM/data/processed/{args.demo_name}")

    if args.session:
        sessions = [os.path.join(raw_base, args.session)]
    else:
        sessions = sorted(glob.glob(os.path.join(raw_base, "session_*")))

    if not sessions:
        print(f"No sessions found in {raw_base}")
        exit(1)

    for session in sessions:
        print(f"\nProcessing: {session}")
        chunk_bag(session, out_base)