#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.msg import CollisionObject, PlanningScene, AttachedCollisionObject
from moveit_msgs.srv import GetCartesianPath, ApplyPlanningScene
from moveit_msgs.action import ExecuteTrajectory
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import GripperCommand
import tf2_ros

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')
        
        self.box_size = 0.03
        self.p1 = [0.4, 0.0, 0.35]
        self.p2 = [0.0, 0.4, 0.35]
        self.safe_z = 0.55
        
        # Клиенты
        self.scene_cli = self.create_client(ApplyPlanningScene, 'apply_planning_scene')
        self.cartesian_srv = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.execute_action = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.gripper_client = ActionClient(self, GripperCommand, '/panda_hand_controller/gripper_cmd')

        # Спавним кубик как препятствие
        time.sleep(1.0) # Ждем инициализацию
        self.update_scene_object(action="ADD", pos=self.p1)
        time.sleep(1.0)
        self.run_mission()

    def update_scene_object(self, action, pos=None):
        scene = PlanningScene(is_diff=True)
        
        if action == "ADD":
            co = CollisionObject()
            co.id = "movable_box"
            co.header.frame_id = "panda_link0"
            box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[self.box_size]*3)
            co.primitives.append(box)
            p = Pose()
            p.position.x, p.position.y, p.position.z = pos
            p.orientation.w = 1.0
            co.primitive_poses.append(p)
            co.operation = CollisionObject.ADD
            scene.world.collision_objects.append(co)
            
        elif action == "ATTACH":
            aco = AttachedCollisionObject()
            aco.link_name = "panda_hand"
            aco.object.id = "movable_box"
            aco.object.operation = CollisionObject.ADD
            # РАЗРЕШАЕМ СТОЛКНОВЕНИЕ С ПАЛЬЦАМИ
            aco.touch_links = ["panda_leftfinger", "panda_rightfinger"]
            scene.robot_state.attached_collision_objects.append(aco)
            
        elif action == "DETACH":
            aco = AttachedCollisionObject()
            aco.object.id = "movable_box"
            aco.object.operation = CollisionObject.REMOVE
            scene.robot_state.attached_collision_objects.append(aco)

        self.scene_cli.call_async(ApplyPlanningScene.Request(scene=scene))

    def move_gripper(self, position):
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = 10.0
        self.gripper_client.wait_for_server()
        self.gripper_client.send_goal_async(goal)
        self.get_logger().info(f"Гриппер -> {position} м")

    def run_mission(self):
        q_down = self.get_quaternion_down()
        
        # 1. Захват
        self.move_gripper(0.08)
        self.plan_and_move([self.create_pose(self.p1[0], self.p1[1], self.safe_z, q_down), 
                           self.create_pose(self.p1[0], self.p1[1], self.p1[2], q_down)])
        
        self.move_gripper(0.02) # Зажимаем
        time.sleep(1.0)         # Ждем физического касания
        self.update_scene_object(action="ATTACH") # Логически крепим к пальцам
        
        # 2. Перенос
        self.plan_and_move([self.create_pose(self.p1[0], self.p1[1], self.safe_z, q_down), 
                           self.create_pose(self.p2[0], self.p2[1], self.safe_z, q_down), 
                           self.create_pose(self.p2[0], self.p2[1], self.p2[2], q_down)])
        
        # 3. Разжим
        self.move_gripper(0.08)
        time.sleep(0.5)
        self.update_scene_object(action="DETACH") # Отпускаем
        
        # Отлет
        self.plan_and_move([self.create_pose(self.p2[0], self.p2[1], self.safe_z, q_down)])
        self.get_logger().info("Миссия завершена!")

    def plan_and_move(self, waypoints):
        req = GetCartesianPath.Request()
        req.header.frame_id = "panda_link0"
        req.group_name = "panda_arm"
        req.waypoints = waypoints
        req.max_step = 0.01
        
        future = self.cartesian_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        
        if res and res.fraction > 0.8:
            goal = ExecuteTrajectory.Goal(trajectory=res.solution)
            self.execute_action.wait_for_server()
            send_goal_future = self.execute_action.send_goal_async(goal)
            while not send_goal_future.done():
                rclpy.spin_once(self, timeout_sec=0.1)
            return True
        return False

    def get_quaternion_down(self):
        rot = R.from_euler('xyz', [-180.0, 0.0, -45.0], degrees=True)
        q = rot.as_quat()
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    def create_pose(self, x, y, z, q):
        p = Pose()
        p.position.x = float(x)
        p.position.y = float(y)
        p.position.z = float(z)
        p.orientation = q
        return p

def main():
    rclpy.init()
    node = PickAndPlaceNode()
    rclpy.spin(node)

if __name__ == "__main__":
    main()