#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.msg import CollisionObject, PlanningScene, AttachedCollisionObject, RobotState
from moveit_msgs.srv import GetCartesianPath, ApplyPlanningScene
from moveit_msgs.action import ExecuteTrajectory
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import GripperCommand

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')
        
        self.box_size = 0.03
        self.p1 = [0.4, 0.0, 0.35]
        self.p2 = [0.0, 0.4, 0.35]
        self.safe_z = 0.55
        
        self.gripper_open = 0.04
        self.gripper_close = 0.012
        
        self.scene_cli = self.create_client(ApplyPlanningScene, 'apply_planning_scene')
        self.cartesian_srv = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.execute_action = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.gripper_client = ActionClient(self, GripperCommand, '/panda_hand_controller/gripper_cmd')

        time.sleep(2.0) 
        self.run_mission()

    def update_scene_object(self, action, pos=None):
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True # Важно для устранения пустых JointState
        
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
            aco.object.header.frame_id = "panda_hand"
            
            # --- ОПИСАНИЕ ФОРМЫ (критично!) ---
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [0.03, 0.03, 0.03]
            aco.object.primitives.append(box)
            
            # --- ПОЗИЦИЯ (относительно кисти) ---
            p = Pose()
            p.position.z = 0.08  # Начни с этого значения, если утонет — меняй
            p.orientation.w = 1.0
            aco.object.primitive_poses.append(p)
            
            aco.object.operation = CollisionObject.ADD
            aco.touch_links = ["panda_leftfinger", "panda_rightfinger", "panda_hand"]
            
            # --- ОТПРАВКА ---
            scene = PlanningScene()
            scene.is_diff = True
            
            # Удаляем из мира (чтобы не было конфликта)
            co_remove = CollisionObject()
            co_remove.id = "movable_box"
            co_remove.operation = CollisionObject.REMOVE
            
            scene.world.collision_objects.append(co_remove)
            scene.robot_state.is_diff = True
            scene.robot_state.attached_collision_objects.append(aco)
            
            self.scene_cli.call_async(ApplyPlanningScene.Request(scene=scene))

            

        elif action == "DETACH":
            # 1. Отвязываем (удаляем из робота)
            aco = AttachedCollisionObject()
            aco.object.id = "movable_box"
            aco.object.operation = CollisionObject.REMOVE
            
            # 2. Создаем объект в мире в координатах, где сейчас кисть
            # Получаем текущее положение кисти (допустим, ты знаешь, куда приехал)
            co = CollisionObject()
            co.id = "movable_box"
            co.header.frame_id = "panda_link0" # Мир
            
            # ВАЖНО: Координаты куба должны соответствовать координатам кисти panda_hand
            # Сейчас рука в точке p2. Поэтому ставим куб в p2.
            p = Pose()
            p.position.x = self.p2[0]
            p.position.y = self.p2[1]
            p.position.z = self.p2[2] # Высота второго стола
            p.orientation.w = 1.0
            
            box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.03, 0.03, 0.03])
            co.primitives.append(box)
            co.primitive_poses.append(p)
            co.operation = CollisionObject.ADD
            
            # 3. Отправляем ОДНИМ пакетом (ApplyPlanningScene)
            scene = PlanningScene()
            scene.is_diff = True
            scene.robot_state.is_diff = True
            # Удаляем из прикрепленных
            scene.robot_state.attached_collision_objects.append(aco)
            # Добавляем в мир
            scene.world.collision_objects.append(co)
            
            self.scene_cli.call_async(ApplyPlanningScene.Request(scene=scene))

    def move_gripper(self, position):
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = 10.0
        self.gripper_client.wait_for_server()
        self.gripper_client.send_goal_async(goal)
        self.get_logger().info(f"Гриппер -> {position} м")
        time.sleep(1.0) # Даем время контроллеру "успокоиться"

    def run_mission(self):
        q_down = self.get_quaternion_down()
        input(1)
        self.get_logger().info("Шаг 1: Подготовка")
        self.move_gripper(self.gripper_open)
        self.plan_and_move([self.create_pose(0.3, 0.0, 0.6, q_down)])
        
        self.update_scene_object(action="ADD", pos=self.p1)
        time.sleep(1.0)
        input(2)
        self.get_logger().info("Шаг 2: Захват")
        self.plan_and_move([
            self.create_pose(self.p1[0], self.p1[1], self.safe_z, q_down), 
            self.create_pose(self.p1[0], self.p1[1], self.p1[2], q_down)
        ])
        
        self.move_gripper(self.gripper_close)
        self.update_scene_object(action="ATTACH")
        time.sleep(1.0) # Ждем синхронизации сцены
        input(3)
        self.get_logger().info("Шаг 3: Перенос")
        self.plan_and_move([
            self.create_pose(self.p1[0], self.p1[1], self.safe_z, q_down),
            self.create_pose(self.p2[0], self.p2[1], self.safe_z, q_down),
            self.create_pose(self.p2[0], self.p2[1], self.p2[2], q_down)
        ])
        input(4)
        self.get_logger().info("Шаг 4: Сброс")
        self.move_gripper(self.gripper_open)
        self.update_scene_object(action="DETACH")
        time.sleep(0.5)
        input(5)
        self.plan_and_move([self.create_pose(self.p2[0], self.p2[1], self.safe_z, q_down)])
        self.get_logger().info("Миссия завершена!")

    def plan_and_move(self, waypoints):
        req = GetCartesianPath.Request()
        req.header.frame_id = "panda_link0"
        req.group_name = "panda_arm"
        req.waypoints = waypoints
        req.max_step = 0.01
        
        req.avoid_collisions = True
        # Явно указываем использовать ТЕКУЩЕЕ состояние как начало
        start_state = RobotState()
        start_state.is_diff = True
        req.start_state = start_state

        future = self.cartesian_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        
        if res and res.fraction > 0.8:
            goal = ExecuteTrajectory.Goal(trajectory=res.solution)
            self.execute_action.wait_for_server()
            send_goal_future = self.execute_action.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_goal_future)
            
            # Ждем завершения выполнения траектории
            result_future = send_goal_future.result().get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
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