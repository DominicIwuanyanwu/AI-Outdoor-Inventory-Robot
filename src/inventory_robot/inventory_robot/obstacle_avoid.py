import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32

class ObstacleAvoid(Node):
    def __init__(self):
        super().__init__('obstacle_avoid')

        # Distance thresholds (metres)
        self.stop_dist = 0.25
        self.slow_dist = 0.40
        self.wall_dist = 0.18

        # Speeds
        self.speed_fast = 0.15
        self.speed_slow = 0.08
        self.backup_speed = -0.15
        self.backup_time = 0.6
        self.post_scan_backup = 0.5   # NEW: back up after scanning a box

        # Turning
        self.turn_in_place = True     # NEW: True = spin on spot (diff drive)
        self.turn_linear_speed = 0.08 # Only used if turn_in_place = False
        self.turn_rate = 1.0          # INCREASED for sharper turns
        self.turn_time = 1.5          # INCREASED to actually clear the obstacle

        # State
        self.scan_active = False
        self.scan_was_active = False  # NEW: detects end-of-scan
        self.state = "FORWARD"
        self.state_until = 0.0
        self.current_turn_dir = 1.0   # 1.0 = left, -1.0 = right

        # Debounce counters
        self.wall_count = 0
        self.wall_threshold = 2
        self.obstacle_count = 0
        self.obstacle_threshold = 2

        # Camera
        self.visual_proximity = 0.0

        # ROS
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Range, '/ultrasonic/range', self.on_range, 10)
        self.create_subscription(Bool, '/scan_active', self.on_scan_active, 10)
        self.create_subscription(Float32, '/camera/visual_proximity', self.on_visual, 10)

        self.get_logger().info("ObstacleAvoid: V2 turn-in-place + post-scan escape")

    def on_visual(self, msg: Float32):
        self.visual_proximity = float(msg.data)

    def on_scan_active(self, msg: Bool):
        was_active = self.scan_active
        self.scan_active = bool(msg.data)

        if self.scan_active:
            self.publish_cmd(0.0, 0.0)
            self.state = "STOPPED"
        elif was_active:
            # NEW: we just finished scanning -> trigger escape maneuver
            self.scan_was_active = True

    def publish_cmd(self, lin_x, ang_z):
        t = Twist()
        t.linear.x = float(lin_x)
        t.angular.z = float(ang_z)
        self.cmd_pub.publish(t)

    def on_range(self, msg: Range):
        r = float(msg.range)
        now = time.time()

        # 1. Scanning mode -> absolute stop (scanner is boss)
        if self.scan_active:
            self.publish_cmd(0.0, 0.0)
            return

        # 2. POST-SCAN ESCAPE: back straight up from the box we just scanned
        if self.scan_was_active:
            self.scan_was_active = False
            self.get_logger().info("POST-SCAN: backing away from object")
            self.state = "BACKING_UP"
            self.state_until = now + self.post_scan_backup
            self.publish_cmd(self.backup_speed, 0.0)
            return

        # 3. State-machine timeouts (non-blocking)
        if now < self.state_until:
            if self.state == "BACKING_UP":
                self.publish_cmd(self.backup_speed, 0.0)
                return
            elif self.state == "TURNING":
                lin = 0.0 if self.turn_in_place else self.turn_linear_speed
                self.publish_cmd(lin, self.turn_rate * self.current_turn_dir)
                return
            elif self.state == "STOPPED":
                self.publish_cmd(0.0, 0.0)
                return

        # 4. State expired -> reset to FORWARD for fresh evaluation
        if self.state != "FORWARD":
            self.state = "FORWARD"

        # 5. Bad sensor reading -> cruise
        if r <= 0.0 or r > 4.0:
            self.publish_cmd(self.speed_fast, 0.0)
            return

        # 6. Camera fusion
        cam = self.visual_proximity

        # 7. WALL: way too close -> back up straight
        if r < self.wall_dist:
            self.wall_count += 1
            if self.wall_count >= self.wall_threshold:
                self.get_logger().warning(f"WALL! US:{r:.2f}m VIS:{cam:.2f}")
                self.state = "BACKING_UP"
                self.state_until = now + self.backup_time
                self.wall_count = 0
                return
        else:
            self.wall_count = 0

        # 8. OBSTACLE: turn IN PLACE (no forward motion)
        cam_blocked = cam >= 0.35 and r < 1.0
        if r < self.stop_dist or cam_blocked:
            self.obstacle_count += 1
            if self.obstacle_count >= self.obstacle_threshold:
                # Alternate direction so we don't oscillate in a corner
                self.current_turn_dir *= -1

                self.state = "TURNING"
                self.state_until = now + self.turn_time
                self.obstacle_count = 0

                dstr = "LEFT" if self.current_turn_dir > 0 else "RIGHT"
                self.get_logger().info(f"AVOID US:{r:.2f}m VIS:{cam:.2f} -> TURN {dstr}")
                return
        else:
            self.obstacle_count = 0

        # 9. SLOW zone
        cam_cautious = cam >= 0.15 and r < 1.5
        if r < self.slow_dist or cam_cautious:
            self.publish_cmd(self.speed_slow, 0.0)
            return

        # 10. CLEAR -> full speed
        self.publish_cmd(self.speed_fast, 0.0)


def main():
    rclpy.init()
    node = ObstacleAvoid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
