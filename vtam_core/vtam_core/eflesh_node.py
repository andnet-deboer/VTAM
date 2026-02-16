#!/usr/bin/env python3

import time
import struct
import threading
import numpy as np
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool


class SkinSerialReader:
    """Replaces AnySkinProcess — reads binary sensor frames + button byte."""

    def __init__(self, port, baud=115200, num_mags=5):
        self.num_mags = num_mags
        # Each mag sends t, x, y, z as floats (4 bytes each) = 16 bytes per mag
        self.mag_frame_size = num_mags * 16
        self.total_frame_size = self.mag_frame_size + 1  # +1 for button byte

        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.latest_data = None
        self.latest_button = 0
        self.lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)

    def start(self):
        # Drain startup text (I2C scan, Init MLX messages, etc.)
        time.sleep(6.0)
        self.ser.reset_input_buffer()
        self.thread.start()

    def _read_loop(self):
        buf = b''
        while self.running:
            buf += self.ser.read(self.ser.in_waiting or 1)
            while b'\r\n' in buf:
                frame, buf = buf.split(b'\r\n', 1)
                if len(frame) == self.total_frame_size:
                    sensor_bytes = frame[:self.mag_frame_size]
                    btn = frame[self.mag_frame_size]
                    # Parse 5 mags x 4 floats = 20 floats
                    values = struct.unpack(f'<{self.num_mags * 4}f', sensor_bytes)
                    with self.lock:
                        self.latest_data = np.array(values)
                        self.latest_button = btn

    def get_data(self):
        """Returns (np.array of 20 floats, button_state) or (None, 0)"""
        with self.lock:
            return self.latest_data, self.latest_button

    def stop(self):
        self.running = False
        self.thread.join(timeout=2.0)
        self.ser.close()


class EFleshNode(Node):
    def __init__(self):
        super().__init__('eflesh_driver')

        # --- Parameters ---
        self.declare_parameter('umi_gripper',
            '/dev/serial/by-id/usb-Adafruit_QT_Py_M0_13CEB8D550555450382E3120FF140A34-if00')

        port_umi = self.get_parameter('umi_gripper').get_parameter_value().string_value

        # --- Publishers ---
        self.pub_left = self.create_publisher(Float32MultiArray, '/tactile_left', 10)
        self.pub_right = self.create_publisher(Float32MultiArray, '/tactile_right', 10)
        self.pub_umi = self.create_publisher(Float32MultiArray, '/tactile_gripper_controller', 10)

        self.stream_left = None
        self.stream_right = None

        # --- Serial Reader (replaces AnySkinProcess) ---
        self.get_logger().info(f'Connecting UMI Controller Sensor on: {port_umi}')
        self.reader = SkinSerialReader(port=port_umi)
        self.reader.start()

        # --- Calibration ---
        self.get_logger().info('Waiting for sensors to settle (3s)...')
        time.sleep(3.0)

        self.get_logger().info('Calibrating Baselines...')
        self.baseline_umi = self._calibrate()

        if self.baseline_umi is None:
            self.get_logger().error('Failed to calibrate sensors. Exiting.')
            raise SystemExit

        self.get_logger().info('Calibration Complete.')

        # --- Record Service Client ---
        self.record_client = self.create_client(SetBool, 'record_demo')
        self.is_recording = False

        # --- Button State: seed from actual hardware, don't arm until ready ---
        _, initial_btn = self.reader.get_data()
        self.prev_button = initial_btn
        self.button_armed = True

        # --- Master Sync Handshake ---
        self.sync_sub = self.create_subscription(
            JointState,
            '/sync_pulse',
            self.sync_callback,
            10
        )
        self.get_logger().info('✓ EFlesh Node Latched to Master 10Hz Clock')

    def _calibrate(self, samples=50):
        """Collect samples and compute baseline mean."""
        collected = []
        for _ in range(samples * 20):
            data, _ = self.reader.get_data()
            if data is not None:
                collected.append(data)
                if len(collected) >= samples:
                    return np.mean(collected, axis=0)
            time.sleep(0.05)
        return None

    def create_msg(self, data):
        """Helper to wrap numpy array into Float32MultiArray."""
        msg = Float32MultiArray()
        dim = MultiArrayDimension()
        dim.label = "tactile_data"
        dim.size = len(data)
        dim.stride = len(data)
        msg.layout.dim.append(dim)
        msg.data = data.tolist()
        return msg

    def sync_callback(self, pulse_msg):
        """
        Triggered exactly when the Robot Node says 'Go'.
        Latching at 10Hz keeps jitter under 10ms for Diffusion Policy.
        """
        data, button = self.reader.get_data()

        # Publish tactile data
        if data is not None:
            corrected = data - self.baseline_umi
            # Strip temperature: each mag is [t, x, y, z], keep only [x, y, z]
            reshaped = corrected.reshape(5, 4)
            xyz_only = reshaped[:, 1:].flatten()  # 15 values
            msg = self.create_msg(xyz_only)
            self.pub_umi.publish(msg)

        # Only check button after system is fully initialized
        if self.button_armed:
            # Detect rising edge: button went 0 -> 1
            if button == 1 and self.prev_button == 0:
                self.toggle_recording()
            self.prev_button = button

    def toggle_recording(self):
        if not self.record_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('record_demo service not available')
            return

        req = SetBool.Request()
        req.data = not self.is_recording
        future = self.record_client.call_async(req)
        future.add_done_callback(self._record_response)

    def _record_response(self, future):
        try:
            result = future.result()
            if result.success:
                self.is_recording = not self.is_recording
                state = "RECORDING" if self.is_recording else "STOPPED"
                self.get_logger().info(f'🔴 {state}: {result.message}')
            else:
                self.get_logger().error(f'Record failed: {result.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def destroy_node(self):
        if self.stream_left:
            self.stream_left.pause_streaming()
            self.stream_left.join()
        if self.stream_right:
            self.stream_right.pause_streaming()
            self.stream_right.join()
        self.reader.stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = EFleshNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()