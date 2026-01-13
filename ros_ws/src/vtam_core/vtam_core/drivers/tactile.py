import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import time

class TactileBridge(Node):
    def __init__(self):
        super().__init__('tactile_bridge')
        # Publisher for your sensor data
        self.pub = self.create_publisher(Float32MultiArray, '/tactile_sensors', 10)
        
        # Hardware Connection (Update port if needed)
        try:
            self.serial = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
            self.get_logger().info("Connected to Arduino on /dev/ttyACM0")
        except serial.SerialException:
            self.get_logger().warn("Arduino not found! Publishing zeros for testing.")
            self.serial = None

        self.create_timer(0.02, self.read_and_publish) # 50Hz

    def read_and_publish(self):
        data = []
        if self.serial and self.serial.in_waiting:
            try:
                line = self.serial.readline().decode('utf-8').strip()
                # Assuming CSV format from Arduino: "0.1,0.5,0.2..."
                data = [float(x) for x in line.split(',')]
            except ValueError:
                return
        else:
            # Dummy data if no hardware attached (for software dev)
            data = [0.0] * 5 

        msg = Float32MultiArray()
        msg.data = data
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TactileBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()