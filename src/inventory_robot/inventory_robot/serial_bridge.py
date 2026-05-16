import os
import serial
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        self.port = '/dev/ttyUSB0'
        self.baud = 115200
        self.ser = None

        if not os.path.exists(self.port):
            self.get_logger().error(f"{self.port} not found! Check Arduino cable.")
        else:
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
                time.sleep(2)  # Arduino reset grace
                self.get_logger().info(f"Serial bridge on {self.port} @ {self.baud}")
            except serial.SerialException as e:
                self.get_logger().error(f"Serial open failed: {e}")

        self.range_pub = self.create_publisher(Range, '/ultrasonic/range', 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        if self.ser:
            self.create_timer(0.02, self.read_serial)

    def cmd_callback(self, msg: Twist):
        if self.ser is None:
            return
        line = f"V,{msg.linear.x:.2f},{msg.angular.z:.2f}\n"
        try:
            self.ser.write(line.encode())
        except serial.SerialException as e:
            self.get_logger().warn(f"Serial write error: {e}")

    def read_serial(self):
        if self.ser is None:
            return
        try:
            line = self.ser.readline().decode().strip()
            if line.startswith("R,"):
                parts = line.split(",")
                if len(parts) >= 2:
                    val = float(parts[1])
                    msg = Range()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.radiation_type = Range.ULTRASOUND
                    msg.field_of_view = 0.5
                    msg.min_range = 0.02
                    msg.max_range = 4.0
                    msg.range = val
                    self.range_pub.publish(msg)
        except ValueError:
            self.get_logger().debug(f"Bad serial frame: {line}")
        except Exception as e:
            self.get_logger().debug(f"Serial read error: {e}")

def main():
    rclpy.init()
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
