#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Vector3
from rclpy.action import ActionClient

# Правильный импорт твоего Action
from moveit_pnp_server.action import MotionPlan  

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')

        # Клиент Action
        self.arm_client = ActionClient(self, MotionPlan, '/move_group')

        self.get_logger().info("Waiting for MoveGroup action server...")
        self.arm_client.wait_for_server()
        self.get_logger().info("MoveGroup ready!")

    def send_action_goal(self, command_type, pose, grab_width=0.05, size=Vector3(x=0.1, y=0.1, z=0.1)):
        # Создаём Goal
        goal = MotionPlan.Goal()
        goal.command_type = command_type
        goal.target_pose = pose
        goal.grab_width = grab_width
        goal.object_size = size

        self.get_logger().info(
            f"Sending goal: {['MOVE','PICK','PLACE'][command_type]} "
            f"to ({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f})"
        )

        # Отправка Goal
        send_goal_future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by MoveGroup")
            return False

        # Ждём результат
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result

        if result.error_code != 1:
            self.get_logger().error(f"Action failed with error code {result.error_code}")
            return False

        self.get_logger().info("Action succeeded!")
        return True

    def run_pick_and_place(self):
        # Позиция PICK на столе 1
        pick_pose = Pose()
        pick_pose.position.x = 0.4
        pick_pose.position.y = 0.0
        pick_pose.position.z = 0.35
        pick_pose.orientation.w = 1.0

        # Позиция PLACE на столе 2
        place_pose = Pose()
        place_pose.position.x = 0.0
        place_pose.position.y = 0.4
        place_pose.position.z = 0.35
        place_pose.orientation.w = 1.0

        self.get_logger().info("Starting Pick & Place sequence...")

        # PICK
        if not self.send_action_goal(1, pick_pose):
            self.get_logger().error("Pick failed, aborting!")
            return

        # PLACE
        if not self.send_action_goal(2, place_pose):
            self.get_logger().error("Place failed!")
            return

        self.get_logger().info("Pick & Place completed successfully!")

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    node.run_pick_and_place()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
