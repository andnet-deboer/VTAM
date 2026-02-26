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

    def __init__(self, port, baud=115200, num_mags=5, has_button=False):
        self.num_mags = num_mags
        self.has_button = has_button
        # Each mag sends t, x, y, z as floats (4 bytes each) = 16 bytes per mag
        self.mag_frame_size = num_mags * 16
        self.total_frame_size = self.mag_frame_size + (1 if has_button else 0)

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
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
                if not data:
                    continue
                buf += data

                while b'\r\n' in buf:
                    frame, buf = buf.split(b'\r\n', 1)
                    if len(frame) == self.total_frame_size:
                        sensor_bytes = frame[:self.mag_frame_size]
                        btn = frame[self.mag_frame_size] if self.has_button else 0
                        values = struct.unpack(f'<{self.num_mags * 4}f', sensor_bytes)
                        with self.lock:
                            self.latest_data = np.array(values)
                            self.latest_button = btn
            except Exception as e:
                print(f"Serial error detected: {e}")
                time.sleep(0.1)

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
        self.declare_parameter('left_finger', '/dev/serial/by-id/usb-Adafruit_QT_Py_M0_6DE50CD050555450382E3120FF121544-if00')
        self.declare_parameter('right_finger', '/dev/serial/by-id/usb-Adafruit_QT_Py_M0_ADFE7C8B50584C43322E3120FF0F241E-if00')
        
        port_umi = self.get_parameter('umi_gripper').get_parameter_value().string_value
        port_left = self.get_parameter('left_finger').get_parameter_value().string_value
        port_right = self.get_parameter('right_finger').get_parameter_value().string_value

        # --- Publishers ---
        self.pub_left = self.create_publisher(Float32MultiArray, '/tactile_left', 10)
        self.pub_right = self.create_publisher(Float32MultiArray, '/tactile_right', 10)
        self.pub_umi = self.create_publisher(Float32MultiArray, '/tactile_gripper_controller', 10)

        # --- Serial Reader (replaces AnySkinProcess) ---
        self.get_logger().info(f'Connecting UMI Controller Sensor on: {port_umi}')
        self.reader = SkinSerialReader(port=port_umi, has_button=True)
        self.reader.start()

        # --- Initialize Left and Right Readers ---
        self.reader_left = SkinSerialReader(port=port_left, has_button=False)
        self.reader_right = SkinSerialReader(port=port_right, has_button=False)
        
        self.reader_left.start()
        self.reader_right.start()

        # --- Sensitivity ---
        self.declare_parameter('sensitivity_umi', 1.0)
        self.declare_parameter('sensitivity_left', 3.0)
        self.declare_parameter('sensitivity_right', 3.0)

        self.gain_umi = self.get_parameter('sensitivity_umi').get_parameter_value().double_value
        self.gain_left = self.get_parameter('sensitivity_left').get_parameter_value().double_value
        self.gain_right = self.get_parameter('sensitivity_right').get_parameter_value().double_value

        # --- Rolling baseline alpha ---
        # Lower = slower adaptation (better for fast touches, ~0.01)
        # Higher = faster adaptation (better if arm moves a lot, ~0.05)
        # At 10Hz, alpha=0.02 gives ~5 second time constant
        self.declare_parameter('alpha', 0.001)
        self.alpha = self.get_parameter('alpha').get_parameter_value().double_value

        self.declare_parameter('contact_threshold', 10.0)    # epsilon: minimum magnitude to consider contact
        self.declare_parameter('decay_rate_threshold', 100.0)   # delta: max negative dB/dt before calling it recovery
        self.contact_threshold = self.get_parameter('contact_threshold').get_parameter_value().double_value
        self.decay_rate_threshold = self.get_parameter('decay_rate_threshold').get_parameter_value().double_value
        
        # --- Get Baselines ---
        self.get_logger().info('Calibrating Baselines...')
        self.baseline_umi = self._calibrate(self.reader)
        self.baseline_left = self._calibrate(self.reader_left)
        self.baseline_right = self._calibrate(self.reader_right)

        if self.baseline_left is None:
            self.get_logger().error('⚠ LEFT calibration FAILED — sensor not streaming')
        if self.baseline_right is None:
            self.get_logger().error('⚠ RIGHT calibration FAILED — sensor not streaming')
        
        self.prev_raw_left = None
        self.prev_raw_right = None

        # --- Dynamic baselines for finger sensors (rolling high-pass filter) ---
        self.dyn_baseline_left = self.baseline_left.copy() if self.baseline_left is not None else None
        self.dyn_baseline_right = self.baseline_right.copy() if self.baseline_right is not None else None

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

    def _calibrate(self, reader, samples=50):   
        """Updated to accept the specific reader instance"""
        collected = []
        for _ in range(100):
            data, _ = reader.get_data()
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

    def _process_static(self, raw_data, baseline, publisher, gain):
        """UMI gripper controller: static baseline (orientation doesn't change much)."""
        if raw_data is not None and baseline is not None:
            corrected = (raw_data - baseline) * gain
            reshaped = corrected.reshape(5, 4)
            xyz_only = reshaped[:, 1:].flatten()
            publisher.publish(self.create_msg(xyz_only))

    def _process_rolling(self, raw_data, dyn_baseline, prev_raw, publisher, gain):
        if raw_data is None or dyn_baseline is None:
            return dyn_baseline, prev_raw

        corrected = (raw_data - dyn_baseline) * gain
        reshaped = corrected.reshape(5, 4)
        xyz_only = reshaped[:, 1:].flatten()

        magnitude = np.linalg.norm(xyz_only)

        if prev_raw is not None:
            dB = raw_data - prev_raw
            dB_mag = np.linalg.norm(dB)
            dB_sign = -1.0 if np.dot(dB.flatten(), (raw_data - dyn_baseline).flatten()) < 0 else 1.0
            rate = dB_sign * dB_mag

            elevated = magnitude > self.contact_threshold
            recovering = rate < -self.decay_rate_threshold

            # Contact = elevated AND not passively recovering
            in_contact = elevated and not recovering

            if not in_contact:
                dyn_baseline = (1 - self.alpha) * dyn_baseline + self.alpha * raw_data
        else:
            dyn_baseline = (1 - self.alpha) * dyn_baseline + self.alpha * raw_data

        publisher.publish(self.create_msg(xyz_only))
        return dyn_baseline, raw_data

    def sync_callback(self, pulse_msg):
        data_u, button = self.reader.get_data()
        data_l, _ = self.reader_left.get_data()
        data_r, _ = self.reader_right.get_data()

        # UMI controller: static baseline (mounted fixed on gripper body)
        self._process_static(data_u, self.baseline_umi, self.pub_umi, self.gain_umi)

        # Finger sensors: rolling baseline (orientation changes as arm moves)
        self.dyn_baseline_left, self.prev_raw_left = self._process_rolling(
            data_l, self.dyn_baseline_left, self.prev_raw_left, self.pub_left, self.gain_left)
        self.dyn_baseline_right, self.prev_raw_right = self._process_rolling(
            data_r, self.dyn_baseline_right, self.prev_raw_right, self.pub_right, self.gain_right)
        
        if self.button_armed:
            if button == 1 and self.prev_button == 0:
                self.toggle_recording()
            self.prev_button = button

    def toggle_recording(self):
        if not self.record_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('record_demo service not available')
            return

        # Re-baseline finger sensors BEFORE starting a new recording
        if not self.is_recording:
            self._reset_baselines()

        req = SetBool.Request()
        req.data = not self.is_recording
        future = self.record_client.call_async(req)
        future.add_done_callback(self._record_response)

    def _reset_baselines(self):
        """Snap dynamic baselines to current readings, zeroing out
        magnetic drift and residual TPU deformation."""
        self.get_logger().info('Re-baselining finger sensors...')

        raw_left, _ = self.reader_left.get_data()
        raw_right, _ = self.reader_right.get_data()
        raw_umi, _ = self.reader.get_data()

        if raw_left is not None:
            self.dyn_baseline_left = raw_left.copy()
            self.prev_raw_left = None  # reset derivative tracking too
        if raw_right is not None:
            self.dyn_baseline_right = raw_right.copy()
            self.prev_raw_right = None
        if raw_umi is not None:
            self.baseline_umi = raw_umi.copy()

        self.get_logger().info('Baselines reset.')

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
        self.reader.stop()
        self.reader_left.stop()
        self.reader_right.stop()
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