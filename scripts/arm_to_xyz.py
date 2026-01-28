#!/usr/bin/env python3

# Follow the following steps to create and run this script
# cd ~/tiago_public_ws/scripts
# touch lift_arm.py
# chmod +x lift_arm.py

# to run the script
# python3 /home/anduw/tiago_public_ws/scripts/arm_to_xyz.py 0.3 0.5 0.85

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveGroupSequenceRequest
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from geometry_msgs.msg import Pose

class TiagoCommander(Node):
    def __init__(self):
        super().__init__('tiago_commander')
        
        # IK Client
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        # Try adding the namespace if the service isn't found
        # self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        # MoveGroup Action Client
        self._action_client = ActionClient(self, MoveGroup, 'move_group')
        
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for IK service...')
        self.get_logger().info('TIAGo Commander Ready.')

    def solve_ik(self, x, y, z):
        request = GetPositionIK.Request()
        ik_request = PositionIKRequest()
        ik_request.group_name = 'arm_torso'
        ik_request.avoid_collisions = True
        ik_request.pose_stamped.header.frame_id = 'base_footprint'
        ik_request.pose_stamped.pose.position.x = x
        ik_request.pose_stamped.pose.position.y = y
        ik_request.pose_stamped.pose.position.z = z
        ik_request.pose_stamped.pose.orientation.w = 1.0 
        
        request.ik_request = ik_request
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def move_to_joints(self, joint_state):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm_torso'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        
        # Set Joint Constraints from the IK solution
        constraints = Constraints()
        for name, pos in zip(joint_state.name, joint_state.position):
            if "arm" in name or "torso" in name:
                jc = JointConstraint()
                jc.joint_name = name
                jc.position = pos
                jc.tolerance_above = 0.01
                jc.tolerance_below = 0.01
                jc.weight = 1.0
                constraints.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(constraints)
        
        self.get_logger().info('Sending execution goal to Gazebo...')
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)

def main():
    if len(sys.argv) != 4:
        print("Usage: python3 move_tiago.py <x> <y> <z>")
        return

    x, y, z = map(float, sys.argv[1:])
    rclpy.init()
    commander = TiagoCommander()

    # 1. Get the math done
    ik_result = commander.solve_ik(x, y, z)

    if ik_result.error_code.val == 1:
        print("✅ IK Success! Moving robot...")
        # 2. Execute the movement
        future = commander.move_to_joints(ik_result.solution.joint_state)
        rclpy.spin_until_future_complete(commander, future)
        print("🏁 Movement Finished!")
    else:
        print(f"❌ IK Failed (Error {ik_result.error_code.val}). Robot will not move.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()

