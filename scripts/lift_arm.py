#!/usr/bin/env python3

# Follow the following steps to create and run this script
# cd ~/tiago_public_ws/scripts
# touch lift_arm.py
# chmod +x lift_arm.py

# to run the script
# python3 /home/anduw/tiago_public_ws/scripts/lift_arm.py 0.2 -0.6 0.0 1.2 1.57 0.0 0.0
# python3 /home/anduw/tiago_public_ws/scripts/lift_arm.py 0.0 1.6 0.0 0.1 -1.57 0.0 0.0

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
import sys

class ArmMover(Node):
    def __init__(self, positions):
        super().__init__('arm_mover')

        # These must match the names seen in /joint_states exactly
        self.joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint',
            'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]

        self.target_positions = positions

        # Subscription to "Arm Odometer" (Joint States)
        # We use this to track the arm's current physical state
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Action Client for the arm controller
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        self.get_logger().info("Waiting for Arm Action Server...")
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available! Is the simulation running?")
            return
            
        self.send_goal()

    def joint_state_callback(self, msg):
        """ This function acts as the odometer for the arm joints """
        current_positions = []
        for name in self.joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                current_positions.append(msg.position[idx])
        
        # Display current arm state in the terminal periodically
        if len(current_positions) == 7:
            pos_str = ", ".join([f"{p:.2f}" for p in current_positions])
            # self.get_logger().info(f"Current Arm Positions: [{pos_str}]", throttle_duration_sec=1.0)

    def send_goal(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.target_positions
        # Time to reach the target (4 seconds provides a smooth, fast lift)
        point.time_from_start.sec = 4 
        point.time_from_start.nanosec = 0

        goal.trajectory.points.append(point)

        self.get_logger().info(f"Sending goal to lift arm to: {self.target_positions}")
        
        self._send_goal_future = self.client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by TIAGo! (Check for self-collision)")
            return

        self.get_logger().info("Goal accepted. Moving arm...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        self.get_logger().info("Arm movement complete!")
        # Successfully reached the table, shutting down node
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    # We expect 7 joint values as arguments
    if len(sys.argv) != 8:
        print("Usage: python3 lift_arm.py j1 j2 j3 j4 j5 j6 j7")
        print("Example: python3 lift_arm.py 0.0 -1.3 0.0 1.9 0.0 0.0 0.0")
        return

    try:
        positions = [float(v) for v in sys.argv[1:8]]
        node = ArmMover(positions)
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
