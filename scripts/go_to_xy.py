#!/usr/bin/env python3

# Follow the following steps to create and run this script

# cd ~/tiago_public_ws/scripts
# touch lift_arm.py
# chmod +x lift_arm.py

# to run the script
# python3 /home/anduw/tiago_public_ws/scripts/go_to_xy.py 2.6 -8.6

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import sys

class GoToXY(Node):
    def __init__(self, target_x, target_y):
        super().__init__('go_to_xy')

        self.target_x = target_x
        self.target_y = target_y

        # UPDATED TOPIC NAME
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/mobile_base_controller/odom', self.odom_callback, 10)

        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        
        # Distance Tracking
        self.total_distance = 0.0
        self.prev_x, self.prev_y = None, None

        self.timer = self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Track distance traveled
        if self.prev_x is not None:
            step = math.sqrt((self.x - self.prev_x)**2 + (self.y - self.prev_y)**2)
            self.total_distance += step
        self.prev_x, self.prev_y = self.x, self.y

        # Quaternion to Yaw
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    def control_loop(self):
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        distance = math.sqrt(dx**2 + dy**2)

        # Stop condition (Increased tolerance for high speed)
        if distance < 0.1:
            self.stop_robot()
            self.get_logger().info(f"DONE! Arrived at ({self.x:.2f}, {self.y:.2f})")
            self.get_logger().info(f"Odometer: {self.total_distance:.2f} meters traveled.")
            rclpy.shutdown()
            return

        angle_to_goal = math.atan2(dy, dx)
        angle_error = math.atan2(math.sin(angle_to_goal - self.yaw), math.cos(angle_to_goal - self.yaw))

        cmd = Twist()

        # LOGIC: Faster Rotation and High-Speed Linear
        if abs(angle_error) > 0.2:  # About 11 degrees
            # Rotate faster in place (capped at 1.5 rad/s)
            cmd.angular.z = max(min(2.0 * angle_error, 1.5), -1.5)
            cmd.linear.x = 0.0
        else:
            # Drive fast (capped at 2.0 m/s)
            # Use a proportional gain (0.8) so it slows down as it gets closer
            cmd.linear.x = min(1.2, 0.5 * distance)
            # Keep steering while moving
            cmd.angular.z = 0.5 * angle_error

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

def main():
    rclpy.init()
    if len(sys.argv) < 3:
        print("Usage: python3 go_to_xy.py <x> <y>")
        return
    node = GoToXY(float(sys.argv[1]), float(sys.argv[2]))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
