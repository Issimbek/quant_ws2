#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose, Vector3
from moveit_pnp_server.action import MotionPlan
import threading
import time

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')

        # Убедись, что имя экшена 'motion_plan_action' совпадает с тем, что в твоем сервере!
        # Если в сервере /move_group, замени здесь.
        self.arm_client = ActionClient(self, MotionPlan, '/motion_plan_action')

        self.get_logger().info("Waiting for Action server...")
        self.arm_client.wait_for_server()
        self.get_logger().info("Connected to Server!")

    def send_goal_sync(self, command_type, x, y, z, grab_width=0.03):
        goal = MotionPlan.Goal()
        goal.command_type = command_type
        
        goal.target_pose.position.x = x
        goal.target_pose.position.y = y
        goal.target_pose.position.z = z
        
        # ВАЖНО: Ориентация. w=1.0 — это захват смотрит вперед.
        # Для того чтобы смотреть ВНИЗ (на кубик), нужно повернуть захват.
        # Если твой сервер сам правит ориентацию, оставь как есть. 
        # Если нет — используй эти значения для наклона вниз:
        goal.target_pose.orientation.x = 1.0 
        goal.target_pose.orientation.y = 0.0
        goal.target_pose.orientation.z = 0.0
        goal.target_pose.orientation.w = 0.0
        
        goal.grab_width = grab_width
        goal.object_size = Vector3(x=0.1, y=0.1, z=0.1)

        cmd_name = ["MOVE", "PICK", "PLACE"][command_type]
        self.get_logger().info(f"--- Sending: {cmd_name} to ({x}, {y}, {z}) ---")

        # Отправляем цель
        send_goal_future = self.arm_client.send_goal_async(goal)
        
        # Ждем подтверждения от сервера (принял/отклонил)
        while not send_goal_future.done():
            time.sleep(0.1)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Goal {cmd_name} REJECTED by server")
            return False

        self.get_logger().info(f"Goal {cmd_name} accepted, executing...")

        # Ждем завершения выполнения
        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.1)
            
        status = result_future.result().status
        result = result_future.result().result

        if result.error_code == 1:
            self.get_logger().info(f"SUCCESS: {cmd_name} completed!")
            return True
        else:
            self.get_logger().error(f"FAILED: {cmd_name} with error_code: {result.error_code}")
            return False

    def run_sequence(self):
        # Даем время системе инициализироваться
        time.sleep(2.0)
        
        self.get_logger().info("=== STARTING SEQUENCE ===")

        # 1. Сначала MOVE в безопасную точку чуть выше и в стороне
        if not self.send_goal_sync(0, 0.3, -0.2, 0.5):
            self.get_logger().warn("Could not move to home, trying next...")

        # 2. PICK кубика (x=0.4, y=0, z=0.35)
        if not self.send_goal_sync(1, 0.4, 0.0, 0.35, grab_width=0.03):
            self.get_logger().error("PICK FAILED")
            return

        # 3. PLACE на другой стол (x=0, y=0.4, z=0.35)
        if not self.send_goal_sync(2, 0.0, 0.4, 0.35, grab_width=0.08):
            self.get_logger().error("PLACE FAILED")
            return

        self.get_logger().info("=== ALL STEPS COMPLETED! ===")

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()

    # Запускаем логику в отдельном потоке
    thread = threading.Thread(target=node.run_sequence, daemon=True)
    thread.start()

    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()