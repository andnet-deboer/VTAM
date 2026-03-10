#!/usr/bin/env python3
"""
record_demo_node.py — ROS2 node for recording demonstration sessions.

Architecture:
    - One rosbag per session, started explicitly via /start_session service
    - Multiple episodes per bag, delimited by /recording/active Bool messages
    - Button press toggles episode start/stop within the running bag
    - Offline chunking (chunk_bag.py) splits the session bag into individual episodes

Usage:
    ros2 run vtam_core record_demo_node --ros-args -p demo_name:=pickup_cup

    Start the session bag when ready to collect:
        ros2 service call /start_session std_srvs/srv/SetBool "{data: true}"

    Toggle episodes:
        ros2 service call /record_demo std_srvs/srv/SetBool "{data: true}"

    Or wire a physical button to /record_demo.

Topics Published:
    /recording/active  (std_msgs/Bool)  — True=episode start, False=episode stop

Services:
    /start_session  (std_srvs/SetBool) — start the session bag (3s warmup)
    /record_demo    (std_srvs/SetBool) — toggle episode recording on/off
"""

import os
import signal
import subprocess
import time
from datetime import datetime
from sensor_msgs.msg import JointState
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

# Topics to record in every session bag
SESSION_TOPICS = [
    "/sync_pulse",
    "/tf",
    "/tf_static",
    "/recording/active",
    "/camera_arm/color/image_rect_raw/compressed",
    "/camera_arm/color/camera_info",
    "/camera/color/image_raw/compressed",
    "/camera/color/camera_info",
    "/gripper_width_normalized",
    "/joint_states",
    "/tactile_gripper_controller",
    "/umi_cube_pose",
    "/tactile_left",
    "/tactile_right",
    "/umi_cube_pose_raw",
]

# Seconds to wait after bag launch before allowing episode recording.
# Ensures camera frames and TF tree are fully populated.
WARMUP_SECONDS = 3.0

# Maximum session bag size before auto-stop
MAX_BAG_SIZE_GB = 40.0


class RecordDemoNode(Node):
    """
    Manages a single long-running rosbag session with multiple episodes.

    Episode boundaries are marked by publishing to /recording/active:
        Bool(data=True)  — episode start
        Bool(data=False) — episode stop

    The bag is started explicitly via /start_session and runs until
    Ctrl+C or disk limit is reached.
    """

    def __init__(self):
        super().__init__('record_demo_node')
        self.declare_parameter('demo_name', 'demo')

        self._sync_pub = self.create_publisher(JointState, '/sync_pulse', 10)
        self.create_timer(1.0 / 30.0, self._sync_tick)

        # Session state
        self._bag_process = None
        self._session_bag_path = None
        self._episode_active = False
        self._warming_up = False
        self._max_bytes = MAX_BAG_SIZE_GB * 1024 ** 3

        # Data directories
        self._save_dir = os.path.expanduser("~/VTAM/data/raw")
        self._tts_cache = os.path.expanduser("~/VTAM/data/assets/tts_cache")
        os.makedirs(self._save_dir, exist_ok=True)
        os.makedirs(self._tts_cache, exist_ok=True)

        # Episode marker publisher
        latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._episode_pub = self.create_publisher(Bool, '/recording/active', latching_qos)

        # Services
        self._start_srv = self.create_service(SetBool, 'start_session', self._start_session_callback)
        self._record_srv = self.create_service(SetBool, 'record_demo', self._toggle_callback)

        # Storage watchdog
        self.create_timer(1.0, self._storage_watchdog)

        # Unmute speakers
        self._init_audio()

        self.get_logger().info(
            "Recorder ready. Call /start_session to begin bag recording."
        )

    # ── Audio ─────────────────────────────────────────────────────────────────

    def _init_audio(self):
        """Unmute system audio for feedback sounds."""
        try:
            subprocess.run(["amixer", "set", "Master", "unmute"], capture_output=True)
            subprocess.run(["amixer", "set", "Master", "80%"], capture_output=True)
        except Exception:
            self.get_logger().warn("Could not configure system volume.")

    def _play_sound(self, sound_type: str):
        """Play a pre-generated TTS feedback sound (non-blocking)."""
        files = {"start": "start.mp3", "stop": "stop.mp3", "ready": "ready.mp3"}
        path = os.path.join(self._tts_cache, files.get(sound_type, ""))
        if os.path.exists(path):
            subprocess.Popen(["mpg123", "-q", path])
        else:
            text = {"start": "Recording", "stop": "Stopped", "ready": "Ready"}.get(sound_type, "")
            subprocess.Popen(["espeak", text])

    def _sync_tick(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "master_clock"
        self._sync_pub.publish(msg)

    # ── Session bag ───────────────────────────────────────────────────────────

    def _start_session(self):
        demo_name = self.get_parameter('demo_name').get_parameter_value().string_value
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        task_dir = os.path.join(self._save_dir, demo_name)
        os.makedirs(task_dir, exist_ok=True)

        self._session_bag_path = os.path.join(task_dir, f"session_{timestamp}")

        cmd = [
            "ros2", "bag", "record", "-s", "mcap",
            "-o", self._session_bag_path,
        ] + SESSION_TOPICS

        self.get_logger().info(f"Starting session bag: {self._session_bag_path}")
        self._bag_process = subprocess.Popen(cmd, start_new_session=True)
        self._warming_up = True

        # Non-blocking warmup — node keeps spinning
        self._warmup_timer = self.create_timer(WARMUP_SECONDS, self._warmup_done)


    def _stop_session(self):
        if self._bag_process is None:
            return

        if self._episode_active:
            self._stop_episode()

        self.get_logger().info("Closing session bag...")
        threading.Thread(target=self._shutdown_bag, daemon=True).start()
        
    # ── Episode control ───────────────────────────────────────────────────────

    def _start_episode(self):
        """Mark episode start in the bag."""
        self._episode_active = True
        self._episode_pub.publish(Bool(data=True))
        self._play_sound("start")
        self.get_logger().info("Episode STARTED")

    def _stop_episode(self):
        """Mark episode stop in the bag."""
        self._episode_active = False
        self._episode_pub.publish(Bool(data=False))
        self._play_sound("stop")
        self.get_logger().info("Episode STOPPED")

    # ── Service callbacks ─────────────────────────────────────────────────────

    def _start_session_callback(self, request, response):
        if self._bag_process is not None:
            # Session running — stop it
            self._stop_session()
            response.success = True
            response.message = f"Session stopped: {self._session_bag_path}"
            return response

        # No session — start one
        self._start_session()
        response.success = True
        response.message = f"Session started: {self._session_bag_path}"
        return response

    def _toggle_callback(self, request, response):
        """Toggle episode recording on/off."""
        if self._bag_process is None or self._bag_process.poll() is not None:
            response.success = False
            response.message = "No active session. Call /start_session first."
            return response

        if self._warming_up:
            response.success = False
            response.message = "Session warming up, please wait."
            return response

        if not self._episode_active:
            self._start_episode()
            response.success = True
            response.message = f"Episode started in: {self._session_bag_path}"
        else:
            self._stop_episode()
            response.success = True
            response.message = "Episode stopped."

        self.get_logger().info(response.message)
        return response

    # ── Watchdog ──────────────────────────────────────────────────────────────

    def _storage_watchdog(self):
        """Auto-stop session if bag exceeds size limit."""
        if self._session_bag_path is None:
            return
        size = self._get_dir_size(self._session_bag_path)
        if size > self._max_bytes:
            self.get_logger().error(
                f"Bag size limit ({MAX_BAG_SIZE_GB}GB) exceeded. Stopping session."
            )
            self._stop_session()

    def _get_dir_size(self, path: str) -> int:
        total = 0
        if not os.path.exists(path):
            return 0
        for dirpath, _, filenames in os.walk(path):
            for f in filenames:
                fp = os.path.join(dirpath, f)
                if not os.path.islink(fp):
                    total += os.path.getsize(fp)
        return total

    def _warmup_done(self):
        self._warmup_timer.cancel()
        self._warming_up = False
        self.get_logger().info("Session ready. Call /record_demo to start an episode.")
        self._play_sound("ready")

    def _shutdown_bag(self):
        try:
            pgid = os.getpgid(self._bag_process.pid)
            os.killpg(pgid, signal.SIGINT)
            self._bag_process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            os.killpg(pgid, signal.SIGKILL)
        except Exception as e:
            self.get_logger().error(f"Error closing session bag: {e}")
        finally:
            self._bag_process = None
            self.get_logger().info(f"Session saved: {self._session_bag_path}")

def main():
    rclpy.init()
    node = RecordDemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_session()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()