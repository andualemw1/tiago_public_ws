#!/usr/bin/env python3

# python3 /home/anduw/tiago_public_ws/scripts/tiago_pp.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import math
import time

class TiagoMaster(Node):
    def __init__(self):
        super().__init__('tiago_master')
        
        # 1. Base Controller (Navigation)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/mobile_base_controller/odom', self.odom_callback, 10)
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0

        # 2. Arm Action Client
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        
        # 3. Hand (Gripper) Action Client
        self.hand_client = ActionClient(self, FollowJointTrajectory, '/hand_controller/follow_joint_trajectory')

        self.get_logger().info("TIAGo Master Node Initialized. Starting Mission...")
        self.run_mission()

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0-2.0*(q.y*q.y + q.z*q.z))

    # --- ACTION HELPERS ---

    def move_arm(self, positions):
        self.get_logger().info(f"Moving arm to {positions}")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 4
        goal.trajectory.points.append(point)
        self.arm_client.wait_for_server()
        return self.arm_client.send_goal_async(goal)

    def move_hand(self, pos):
        # pos: 0.045 is open, 0.0 is closed
        self.get_logger().info(f"Setting gripper to {pos}")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        point = JointTrajectoryPoint()
        point.positions = [pos, pos]
        point.time_from_start.sec = 1
        goal.trajectory.points.append(point)
        self.hand_client.wait_for_server()
        return self.hand_client.send_goal_async(goal)

    # --- MISSION SEQUENCE ---

    def run_mission(self):
        # STAGE 1: Drive to Table 1 (Approx coordinates)
        # Note: You can call your GoToXY logic here
        
        # STAGE 2: Open Hand
        self.move_hand(0.045)
        time.sleep(2)

        # STAGE 3: Reach for Bottle
        # Standard "Reach" pose: [arm_1 to arm_7]
        self.move_arm([0.2, -0.6, 0.0, 1.2, 1.57, 0.0, 0.0])
        time.sleep(5)

        # STAGE 4: Close Hand (Grasp)
        self.move_hand(0.01) # 0.01 creates a tight grip on a bottle
        time.sleep(2)

        # STAGE 5: Tuck Arm (Safety pose for driving)
        self.move_arm([0.0, -1.3, 0.0, 1.9, 0.0, 0.0, 0.0])
        time.sleep(5)

        # STAGE 6: Drive to Table 2 (Other side of room)
        self.get_logger().info("Driving to Table 2...")
        # (Insert navigation logic here to move to x=-2, y=3)

        # STAGE 7: Place and Release
        self.move_arm([0.2, -0.6, 0.0, 1.2, 1.57, 0.0, 0.0]) # Reach out
        time.sleep(4)
        self.move_hand(0.045) # Drop
        self.get_logger().info("Mission Complete!")

def main():
    rclpy.init()
    node = TiagoMaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

