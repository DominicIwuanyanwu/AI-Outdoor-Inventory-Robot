
"""
obstacle_avoid_node.py

Purpose
-------
This node handles simple autonomous navigation using the ultrasonic sensor.

Normal behaviour:
- move forward when the path is clear
- turn away when an obstacle is too close
- simple wall escape
- curved turns 
Important integration note
--------------------------
This node now listens to a /scan_active topic.

Why?
Because when the camera node finds a barcode, the robot must stop and scan
instead of continuing to drive forward.

So:
- if scan_active is False -> obstacle avoidance controls movement
- if scan_active is True  -> obstacle avoidance pauses and publishes stop

"""
#!/usr/bin/env python3
"""
obstacle_avoid_node.py - With wall escape and curved turns
"""
import time
import random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class ObstacleAvoid(Node):
    def __init__(self):
        super().__init__('obstacle_avoid')

        # Distance thresholds
        self.stop_dist = 0.25
        self.slow_dist = 0.40
        self.wall_dist = 0.18        # Very close - need to back up
        
        # Movement settings  
        self.turn_rate = 0.8
        self.turn_time = 1.2         # Slightly longer turns
        
        # CRITICAL FIX: Curved motion instead of spin-in-place
        self.turn_linear_speed = 0.08  # Small forward motion while turning
        self.backup_speed = -0.15
        self.backup_time = 0.6
        
        self.speed_fast = 0.15
        self.speed_slow = 0.08

        # State management
        self.scan_active = False
        self.state = "FORWARD"
        self.state_until = 0.0
        
        # Oscillation detection
        self.turn_history = []       # Track recent turns
        self.oscillation_threshold = 3  # If we turn L,R,L or R,L,R, back up
        
        # Debouncing
        self.obstacle_count = 0
        self.obstacle_threshold = 2
        self.wall_count = 0
        self.wall_threshold = 2

        # ROS interfaces
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.range_sub = self.create_subscription(Range, '/ultrasonic/range', self.on_range, 10)
        self.scan_sub = self.create_subscription(Bool, '/scan_active', self.on_scan_active, 10)

        self.get_logger().info("ObstacleAvoid: With WALL ESCAPE and CURVED TURNS")

    def on_scan_active(self, msg: Bool):
        self.scan_active = bool(msg.data)
        if self.scan_active:
            self.publish_cmd(0.0, 0.0)
            self.state = "STOPPED"

    def publish_cmd(self, lin_x: float, ang_z: float):
        msg = Twist()
        msg.linear.x = float(lin_x)
        msg.angular.z = float(ang_z)
        self.cmd_pub.publish(msg)

    def detect_oscillation(self, new_direction):
        """Detect if we're stuck turning left-right-left-right"""
        self.turn_history.append(new_direction)
        if len(self.turn_history) > 4:
            self.turn_history.pop(0)
        
        # Check for alternating pattern (L,R,L or R,L,R)
        if len(self.turn_history) >= 3:
            recent = self.turn_history[-3:]
            if recent[0] == recent[2] and recent[0] != recent[1]:
                return True
        return False

    def on_range(self, msg: Range):
        if self.scan_active:
            self.publish_cmd(0.0, 0.0)
            return

        r = float(msg.range)
        now = time.time()

        # State machine handling
        if now < self.state_until:
            if self.state == "BACKING_UP":
                self.publish_cmd(self.backup_speed, 0.0)
                return
            elif self.state == "TURNING":
                # CRITICAL FIX: Curved motion (forward + turn) instead of spin
                self.publish_cmd(self.turn_linear_speed, self.current_turn_rate)
                return
            elif self.state == "STOPPED":
                self.publish_cmd(0.0, 0.0)
                return

        # Reset state when timeout expires
        if now >= self.state_until and self.state != "FORWARD":
            self.state = "FORWARD"

        # Validity check
        if r <= 0.0 or r > 4.0:
            if self.state == "FORWARD":
                self.publish_cmd(self.speed_fast, 0.0)
            return

        # WALL DETECTION - Too close, must back up
        if r < self.wall_dist:
            self.wall_count += 1
            if self.wall_count >= self.wall_threshold:
                self.get_logger().warning(f"WALL HIT! Backing up from {r:.2f}m")
                self.state = "BACKING_UP"
                self.state_until = now + self.backup_time
                self.wall_count = 0
                return
        else:
            self.wall_count = 0

        # NORMAL OBSTACLE AVOIDANCE
        if r < self.stop_dist:
            self.obstacle_count += 1
            
            if self.obstacle_count >= self.obstacle_threshold:
                # Choose turn direction
                turn_dir = 1.0 if random.random() > 0.5 else -1.0
                
                # Check for oscillation (stuck pattern)
                if self.detect_oscillation(turn_dir):
                    self.get_logger().error("STUCK IN OSCILLATION! Forcing backup")
                    self.state = "BACKING_UP"
                    self.state_until = now + self.backup_time * 1.5
                    self.obstacle_count = 0
                    return
                
                self.current_turn_rate = self.turn_rate * turn_dir
                self.state = "TURNING"
                self.state_until = now + self.turn_time
                self.obstacle_count = 0
                
                dir_str = "LEFT" if turn_dir > 0 else "RIGHT"
                self.get_logger().info(f"Obstacle at {r:.2f}m - CURVED TURN {dir_str}")
                return
                
        elif r < self.slow_dist:
            self.obstacle_count = 0
            if self.state == "FORWARD":
                self.publish_cmd(self.speed_slow, 0.0)
        else:
            self.obstacle_count = 0
            self.turn_history.clear()  # Clear history when moving freely
            if self.state == "FORWARD" 
                self.publish_cmd(self.speed_fast, 0.0)

def main():
    rclpy.init()
    node = ObstacleAvoid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
