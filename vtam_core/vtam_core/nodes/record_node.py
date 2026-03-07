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
from nav_msgs.msg import Path
from sensor_msgs.msg import Image

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, DurabilityPolicy


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
        self.declare_parameter('session_topics', rclpy.Parameter.Type.STRING_ARRAY)

        # Session state
        self._bag_process = None
        self._session_bag_path = None
        self._episode_active = False
        self._max_bytes = MAX_BAG_SIZE_GB * 1024 ** 3

        self._last_toggle_time = 0.0
        self._TOGGLE_DEBOUNCE = 2.0  # seconds

        self.declare_parameter('camera_topics', rclpy.Parameter.Type.STRING_ARRAY)
        self._camera_last_seen = {}  # topic -> last msg time
        self._camera_subs = []
        self._CAMERA_TIMEOUT = 2.0  # seconds with no msg = dead

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

        self.target_path = None
        self.sub_trajectory = self.create_subscription(
            Path, 
            '/umi_trajectory', 
            self._trajectory_callback, 
            latching_qos
        )

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
        files = {"start": "start.mp3", "stop": "stop.mp3", "ready": "ready.mp3", "error": "error.mp3"}
        path = os.path.join(self._tts_cache, files.get(sound_type, ""))
        if os.path.exists(path):
            subprocess.Popen(["mpg123", "-q", path])
        else:
            text = {"start": "Recording", "stop": "Stopped", "ready": "Ready", "error": "Error. Camera missing."}.get(sound_type, "")
            subprocess.Popen(["espeak", text])
    # ── Session bag ───────────────────────────────────────────────────────────

    def _start_session(self):
        """
        Launch the session-level rosbag and block during warmup.
        All episodes in this session share one bag file.
        """

        camera_topics = self.get_parameter('camera_topics').get_parameter_value().string_array_value
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        for topic in camera_topics:
            self._camera_last_seen[topic] = None
            sub = self.create_subscription(
                Image, topic,
                lambda msg, t=topic: self._camera_heartbeat(t),
                sensor_qos
            )
            self._camera_subs.append(sub)
        demo_name = self.get_parameter('demo_name').get_parameter_value().string_value
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        task_dir = os.path.join(self._save_dir, demo_name)
        os.makedirs(task_dir, exist_ok=True)

        self._session_bag_path = os.path.join(task_dir, f"session_{timestamp}")

        # Fetch topics from the configs
        topics = self.get_parameter('session_topics').get_parameter_value().string_array_value
        
        if not topics:
            self.get_logger().error("No session topics found in config!")
            return

        cmd = [
            "ros2", "bag", "record", "-s", "mcap",
            "-o", self._session_bag_path,
        ] + list(topics) 

        self.get_logger().info(f"Starting session bag: {self._session_bag_path}")
        self.get_logger().info(f"Warming up MCAP buffers ({WARMUP_SECONDS}s)...")

        self._bag_process = subprocess.Popen(cmd, start_new_session=True)

        # Block during warmup — ensures camera I-frames and TF tree are populated
        # before any episode is allowed to start
        time.sleep(WARMUP_SECONDS)

        self.get_logger().info("Session ready. Call /record_demo to start an episode.")
        self._play_sound("ready")

    def _stop_session(self):
        """Cleanly shut down the session bag."""
        for sub in self._camera_subs:
                self.destroy_subscription(sub)
        self._camera_subs.clear()
        self._camera_last_seen.clear()
        if self._bag_process is None:
            return

        # Ensure active episode is closed before bag shuts down
        if self._episode_active:
            self._stop_episode()

        self.get_logger().info("Closing session bag...")
        try:
            pgid = os.getpgid(self._bag_process.pid)
            os.killpg(pgid, signal.SIGINT)

            # Wait up to 3s for clean exit
            for _ in range(30):
                if self._bag_process.poll() is not None:
                    break
                time.sleep(0.1)

            if self._bag_process.poll() is None:
                os.killpg(pgid, signal.SIGKILL)

        except Exception as e:
            self.get_logger().error(f"Error closing session bag: {e}")
        finally:
            self._bag_process = None
            self.get_logger().info(f"Session saved: {self._session_bag_path}")
            self.get_logger().info("Run chunk_bag.py to extract individual episodes.")

    def _trajectory_callback(self, msg):
        """Processes the interpolated path from the UMI detector."""
        if len(msg.poses) > 0:
            # Captures the most recent smoothed pose for the robot to follow
            self.target_path = msg.poses[-1]

    def _camera_heartbeat(self, topic):
        self._camera_last_seen[topic] = self.get_clock().now()

    def _check_cameras_alive(self) -> tuple[bool, list[str]]:
        """Return (all_ok, list_of_dead_topics)."""
        now = self.get_clock().now()
        dead = []
        for topic, last in self._camera_last_seen.items():
            if last is None or (now - last).nanoseconds / 1e9 > self._CAMERA_TIMEOUT:
                dead.append(topic)
        return (len(dead) == 0, dead)

    # ── Episode control ───────────────────────────────────────────────────────

    def _start_episode(self):
        """Mark episode start in the bag, only if cameras are alive."""
        ok, dead = self._check_cameras_alive()
        if not ok:
            self.get_logger().error(
                f"REFUSING to start episode — no data on: {dead}"
            )
            self._play_sound("error")
            return False
        self._episode_active = True
        self._episode_pub.publish(Bool(data=True))
        self._play_sound("start")
        self.get_logger().info("Episode STARTED")
        return True

    def _stop_episode(self):
        """Mark episode stop in the bag."""
        self._episode_active = False
        self._episode_pub.publish(Bool(data=False))
        self._play_sound("stop")
        self.get_logger().info("Episode STOPPED")

    # ── Service callbacks ─────────────────────────────────────────────────────

    def _start_session_callback(self, request, response):
        if self._bag_process is not None:
            self._stop_session()
            response.success = True
            response.message = "Session stopped."
        else:
            self._start_session()
            response.success = True
            response.message = f"Session started: {self._session_bag_path}"
        return response

    def _toggle_callback(self, request, response):
        """Toggle episode recording on/off."""
        now = time.time()
        if now - self._last_toggle_time < self._TOGGLE_DEBOUNCE:
            response.success = False
            response.message = "Debounced — too fast."
            return response
        self._last_toggle_time = now
        
        if self._bag_process is None or self._bag_process.poll() is not None:
            response.success = False
            response.message = "No active session. Call /start_session first."
            self.get_logger().error(response.message)
            return response

        if not self._episode_active:
            if not self._start_episode():
                response.success = False
                response.message = "Episode blocked — camera topics missing. Check connections."
                self.get_logger().error(response.message)
                return response
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