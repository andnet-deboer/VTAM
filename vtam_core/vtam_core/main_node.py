#!/usr/bin/env python3
"""VTAM Monitor — pip install textual"""
import subprocess, signal, os, time, threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import JointState, CameraInfo
from geometry_msgs.msg import PoseStamped
from textual.app import App, ComposeResult
from textual.widgets import Static, Header

_ts, _force = {}, [0.0]

TOPICS = [
    ('d405',     'D405 Arm Camera',   '/camera_arm/color/camera_info',   CameraInfo),
    ('d435i',    'D435i Head Camera',  '/camera/color/camera_info',      CameraInfo),
    ('eflesh',   'eFlesh Tactile',     '/tactile_gripper_controller',    Float32MultiArray),
    ('grip',     'Gripper',            '/gripper_width',                 Float32),
    ('joints',   'Joint States',       '/joint_states',                  JointState),
    ('cube',     'UMI Cube Pose',      '/umi_cube_pose',                 PoseStamped),
    ('gripper_p','UMI Gripper Pose',   '/umi_gripper_pose',              PoseStamped),
]

class Monitor(Node):
    def __init__(self):
        super().__init__('vtam_monitor')
        for tid, _, topic, mtype in TOPICS:
            _ts[tid] = 0.0
            self.create_subscription(mtype, topic, lambda m, t=tid: _ts.__setitem__(t, time.time()), 1)
        self.create_subscription(Float32, '/gripper_width_normalized', lambda m: _force.__setitem__(0, float(m.data)), 10)

class VTAMApp(App):
    CSS = """
    Screen { padding: 1 2; }
    #title { text-style: bold; margin-bottom: 1; }
    #force { margin-top: 1; }
    """
    TITLE = "VTAM"

    def compose(self) -> ComposeResult:
        yield Header()
        yield Static("VTAM System Monitor", id="title")
        for tid, *_ in TOPICS:
            yield Static('', id=tid)
        yield Static('', id='force')

    def on_mount(self):
        self.set_interval(0.1, self.tick)

    def tick(self):
        now = time.time()
        for tid, label, *_ in TOPICS:
            alive = _ts.get(tid, 0) and (now - _ts[tid]) < 2.0
            sym, clr = ('✓', 'green') if alive else ('○', 'yellow')
            self.query_one(f'#{tid}').update(f'[{clr}]{sym}[/] {label}')
        f = max(0.0, min(_force[0], 1.0))
        w = int(f * 40)
        self.query_one('#force').update(f'Force  {"█" * w}{"░" * (40 - w)}  {f:.0%}')

def main():
    proc = subprocess.Popen(
        ['ros2', 'launch', 'vtam_core', 'vtam_record.launch.py'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    rclpy.init()
    node = Monitor()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    try:
        VTAMApp().run()
    finally:
        try: os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except OSError: pass
        try: node.destroy_node(); rclpy.shutdown()
        except Exception: pass

if __name__ == '__main__':
    main()