import asyncio
import time
import json
import numpy as np
import zarr
from foxglove_websocket.server import FoxgloveServer
from differential_ik import TrajectoryRetargeter

async def main():
    zarr_path = "../../data/processed/demo/demo_20260216_020448_0.zarr"
    root = zarr.open(zarr_path, mode='r')
    ee_pose = np.array(root['obs/ee_pose'])
    
    # 1. Initialize Retargeter
    # This triggers your modified project_to_workspace with the 180-deg X-flip
    retargeter = TrajectoryRetargeter()
    local_pos, _, _ = retargeter.project_to_workspace(ee_pose[:, :3], ee_pose[:, 3:])

    async with FoxgloveServer("0.0.0.0", 8765, "X_Flip_Validator") as server:
        chan_id = await server.add_channel({
            "topic": "/ik_validation_path",
            "encoding": "json",
            "schemaName": "ros.nav_msgs.Path",
        })

        print("\n[RUNNING] Server started on ws://localhost:8765")

        # 2. Build the Path with consistent Frame IDs
        path_poses = []
        for i in range(len(local_pos)):
            path_poses.append({
                "header": {"frame_id": "base_link"}, # FIX: Added frame_id to every pose
                "pose": {
                    "position": {
                        "x": float(local_pos[i, 0]), 
                        "y": float(local_pos[i, 1]), 
                        "z": float(local_pos[i, 2])
                    },
                    "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}
                }
            })

        # Main message wrapper
        message = {
            "header": {
                "frame_id": "base_link", 
                "stamp": {"sec": 0, "nsec": 0}
            },
            "poses": path_poses
        }

        while True:
            # Send serialized JSON bytes
            await server.send_message(
                chan_id, 
                time.time_ns(), 
                json.dumps(message).encode("utf-8")
            )
            await asyncio.sleep(1.0)

if __name__ == "__main__":
    asyncio.run(main())