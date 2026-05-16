import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import serial
import time

class SerialBridge(Node):

    def __init__(self):
        super().__init__('serial_bridge')


        self.port = '/dev/ttyUSB0'
        self.baud = 115200

        self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        time.sleep(2)

        self.range_pub = self.create_publisher(Range, '/ultrasonic/range', 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        self.timer = self.create_timer(0.02, self.read_serial)

        self.get_logger().info("Serial bridge started.")

    def cmd_callback(self, msg):
        line = f"V,{msg.linear.x:.2f},{msg.angular.z:.2f}\n"
        self.ser.write(line.encode())

    def read_serial(self):
        try:
            line = self.ser.readline().decode().strip()
            if line.startswith("R,"):
                val = float(line.split(",")[1])

                msg = Range()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.radiation_type = Range.ULTRASOUND
                msg.field_of_view = 0.5
                msg.min_range = 0.02
                msg.max_range = 4.0
                msg.range = val

                self.range_pub.publish(msg)

        except:
            pass

def main():
    rclpy.init()
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
