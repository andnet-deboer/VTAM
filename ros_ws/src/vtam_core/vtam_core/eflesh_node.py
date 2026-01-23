#!/usr/bin/env python3

import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from anyskin import AnySkinProcess

class EFleshNode(Node):
    def __init__(self):
        super().__init__('eflesh_driver')

        # --- Parameters ---
        # Default to the specific paths
        # self.declare_parameter('port_left', '/dev/serial/by-path/pci-0000:00:14.0-usb-0:2.3.3.1.3:1.0')
        # self.declare_parameter('port_right', '/dev/serial/by-path/pci-0000:00:14.0-usb-0:2.3.3.1.4:1.0')
        self.declare_parameter('port_left', '/dev/serial/by-path/pci-0000:00:14.0-usbv2-0:1:1.0 ')
        self.declare_parameter('port_right', '/dev/serial/by-path/pci-0000:00:14.0-usbv2-0:1:1.0 ')
        self.declare_parameter('publish_rate', 60.0)

        port_left = self.get_parameter('port_left').get_parameter_value().string_value
        port_right = self.get_parameter('port_right').get_parameter_value().string_value
        rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # --- Publishers ---
        # Publishing a flat array of 15 floats [x0, y0, z0, x1, y1, z1, ... x4, y4, z4]
        self.pub_left = self.create_publisher(Float32MultiArray, '/tactile_left', 10)
        self.pub_right = self.create_publisher(Float32MultiArray, '/tactile_right', 10)

        # --- Initialize Hardware ---
        self.get_logger().info(f'Connecting Left Sensor on: {port_left}')
        self.stream_left = self.init_sensor(port_left)
        
        self.get_logger().info(f'Connecting Right Sensor on: {port_right}')
        self.stream_right = self.init_sensor(port_right)

        # --- Calibration ---
        self.get_logger().info('Waiting for sensors to settle (2s)...')
        time.sleep(2.0)
        
        self.get_logger().info('Calibrating Baselines...')
        self.baseline_left = self.get_valid_baseline(self.stream_left)
        self.baseline_right = self.get_valid_baseline(self.stream_right)
        
        if self.baseline_left is None or self.baseline_right is None:
            self.get_logger().error('Failed to calibrate sensors. Exiting.')
            raise SystemExit

        self.get_logger().info('Calibration Complete. Publishing data...')

        # --- Timer Loop ---
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

    def init_sensor(self, port):
        try:
            stream = AnySkinProcess(num_mags=5, port=port)
            stream.start()
            return stream
        except Exception as e:
            self.get_logger().error(f'Failed to connect to {port}: {e}')
            return None

    def get_valid_baseline(self, stream, retries=20):
        if stream is None: return None
        for _ in range(retries):
            data = stream.get_data(num_samples=10)
            if data and len(data) > 0:
                arr = np.array(data)
                # Strip timestamp (col 0) and average the rest
                if arr.shape[1] > 1:
                    return np.mean(arr[:, 1:], axis=0)
            time.sleep(0.1)
        return None

    def create_msg(self, data):
        """Helper to wrap numpy array into Float32MultiArray"""
        msg = Float32MultiArray()
        # Define layout (optional but good practice)
        dim = MultiArrayDimension()
        dim.label = "tactile_data"
        dim.size = 15
        dim.stride = 15
        msg.layout.dim.append(dim)
        msg.data = data.tolist()
        return msg

    def timer_callback(self):
        # Process Left
        if self.stream_left:
            raw = self.stream_left.get_data(num_samples=1)
            if raw:
                # [0][1:] removes the wrapper list and the timestamp
                data = np.array(raw[0][1:]) - self.baseline_left
                msg = self.create_msg(data)
                self.pub_left.publish(msg)

        # Process Right
        if self.stream_right:
            raw = self.stream_right.get_data(num_samples=1)
            if raw:
                data = np.array(raw[0][1:]) - self.baseline_right
                msg = self.create_msg(data)
                self.pub_right.publish(msg)

    def destroy_node(self):
        # Clean up threads
        if self.stream_left:
            self.stream_left.pause_streaming()
            self.stream_left.join()
        if self.stream_right:
            self.stream_right.pause_streaming()
            self.stream_right.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = EFleshNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()