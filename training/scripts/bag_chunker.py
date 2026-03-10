#!/usr/bin/env python3
"""
chunk_bag.py — Split a session MCAP bag into per-episode MCAP bags.

Pipeline position:
    record_demo_node.py → session.mcap → chunk_bag.py → chunked/episode_000.mcap, ...
                                                                ↓
                                                        process_demo.py

Episode boundaries are read from /recording/active:
    Bool(data=True)  — episode start
    Bool(data=False) — episode stop

Each output bag:
    - All original topics trimmed to the episode window
    - /progress (std_msgs/Float32) injected at each /sync_pulse timestamp [0.0 → 1.0]

Usage:
    python3 chunk_bag.py path/to/session/ data/chunked/pickup_cup/

    Output: data/chunked/pickup_cup/episode_000.mcap, episode_001.mcap, ...
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
SYNC_TOPIC = "/sync_pulse"
CONFIG_PATH = os.path.expanduser("~/VTAM/src/vtam_core/config/record.yaml")

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

def chunk_bag(bag_path: str, output_dir: str, episode_offset: int = 0) -> int:
    mcap_path = resolve_mcap(bag_path)
    print(f"Source: {mcap_path}")

    episodes = index_episodes(mcap_path)
    if not episodes:
        print("No complete episodes found.")
        return episode_offset
    print(f"Found {len(episodes)} episode(s).")

    os.makedirs(output_dir, exist_ok=True)

    # Pass 1: collect channel metadata + sync timestamps per episode (lightweight)
    channel_meta = {}
    sync_per_episode = [[] for _ in episodes]

    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for schema, channel, message in tqdm(reader.iter_messages(topics=EPISODE_TOPICS), desc="Indexing"):
            ts = message.publish_time
            if channel.topic not in channel_meta:
                channel_meta[channel.topic] = (schema, channel)
            if channel.topic == SYNC_TOPIC:
                for i, (start_ns, end_ns) in enumerate(episodes):
                    if start_ns <= ts <= end_ns:
                        sync_per_episode[i].append(ts)

    # Build progress maps
    progress_maps = []
    valid_episodes = []
    for i, (sync_timestamps, (start_ns, end_ns)) in enumerate(zip(sync_per_episode, episodes)):
        sync_timestamps = sorted(sync_timestamps)
        N = len(sync_timestamps)
        if N < MIN_EPISODE_FRAMES:
            print(f"  Episode {i}: {N} sync frames — discarding (too short)")
            progress_maps.append(None)
            valid_episodes.append(False)
        else:
            progress_maps.append({ts: float(j) / max(N - 1, 1) for j, ts in enumerate(sync_timestamps)})
            valid_episodes.append(True)

    # Pass 2: single stream, write all episodes simultaneously
    out_files = []
    writers = []
    channel_ids_per_ep = []
    progress_channel_ids = []

    for i, (start_ns, end_ns) in enumerate(episodes):
        if not valid_episodes[i]:
            out_files.append(None)
            writers.append(None)
            channel_ids_per_ep.append(None)
            progress_channel_ids.append(None)
            continue

        out_path = os.path.join(output_dir, f"episode_{episode_offset + i:03d}.mcap")
        f = open(out_path, "wb")
        writer = Writer(f)
        writer.start()

        channel_ids = {}
        for topic, (schema, channel) in channel_meta.items():
            sid = writer.register_schema(name=schema.name, encoding=schema.encoding, data=schema.data)
            cid = writer.register_channel(topic=topic, message_encoding=channel.message_encoding, schema_id=sid, metadata=dict(channel.metadata))
            channel_ids[topic] = cid

        progress_schema_id = writer.register_schema(name="std_msgs/msg/Float32", encoding="ros2msg", data=b"float32 data\n")
        progress_channel_id = writer.register_channel(topic="/progress", message_encoding="cdr", schema_id=progress_schema_id)

        out_files.append(f)
        writers.append(writer)
        channel_ids_per_ep.append(channel_ids)
        progress_channel_ids.append(progress_channel_id)
        

    # Single streaming pass
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for schema, channel, message in tqdm(reader.iter_messages(topics=EPISODE_TOPICS), desc="Chunking"):
            ts = message.publish_time
            for i, (start_ns, end_ns) in enumerate(episodes):
                if not valid_episodes[i] or not (start_ns <= ts <= end_ns):
                    continue
                if channel.topic not in channel_ids_per_ep[i]:
                    continue

                writers[i].add_message(channel_id=channel_ids_per_ep[i][channel.topic], log_time=ts, data=message.data, publish_time=ts)

                if channel.topic == SYNC_TOPIC and ts in progress_maps[i]:
                    writers[i].add_message(channel_id=progress_channel_ids[i], log_time=ts, data=encode_float32(progress_maps[i][ts]), publish_time=ts)

    # Close all writers
    valid_count = 0
    for i, writer in enumerate(writers):
        if writer is not None:
            writer.finish()
            out_files[i].close()
            print(f"  Episode {i}: → episode_{i:03d}.mcap")
            valid_count += 1
    
    print(f"\nDone. {valid_count}/{len(episodes)} episodes saved to {output_dir}")
    return episode_offset + len(episodes)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Split a session bag into episode bags.")
    parser.add_argument("demo_name", type=str, help="Demo name (e.g. default_demo)")
    parser.add_argument("--session", type=str, default=None, help="Specific session folder (optional)")
    args = parser.parse_args()

    raw_base  = os.path.expanduser(f"~/VTAM/data/raw/{args.demo_name}")
    out_base  = os.path.expanduser(f"~/VTAM/data/chunked/{args.demo_name}")

    if args.session:
        sessions = [os.path.join(raw_base, args.session)]
    else:
        sessions = sorted(glob.glob(os.path.join(raw_base, "session_*")))

    if not sessions:
        print(f"No sessions found in {raw_base}")
        exit(1)

    episode_offset = 0
    for session in sessions:
        print(f"\nProcessing: {session}")
        episode_offset = chunk_bag(session, out_base, episode_offset)