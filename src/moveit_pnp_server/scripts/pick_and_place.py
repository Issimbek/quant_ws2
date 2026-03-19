#!/usr/bin/env python3
import rclpy
import math
from rclpy.duration import Duration
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

        self.gripper_open = 0.058
        self.gripper_close = 0.012

        # Ручной offset кубика относительно panda_hand после attach
        self.grasp_offset_z = 0.03

        self.scene_cli = self.create_client(ApplyPlanningScene, 'apply_planning_scene')
        self.cartesian_srv = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.execute_action = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.gripper_action = ActionClient(self, GripperCommand, '/panda_hand_controller/gripper_cmd')

        time.sleep(2.0)
        self.run_mission()

    def apply_scene_sync(self, scene):
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self.scene_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def update_scene_object(self, action, pos=None):
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True

        if action == "ADD":
            co = CollisionObject()
            co.id = "movable_box"
            co.header.frame_id = "panda_link0"
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [self.box_size] * 3
            co.primitives.append(box)
            p = Pose()
            p.position.x, p.position.y, p.position.z = pos
            p.orientation.w = 1.0
            co.primitive_poses.append(p)
            co.operation = CollisionObject.ADD
            scene.world.collision_objects.append(co)
            # НИКАКИХ AttachedCollisionObject здесь!
        elif action == "ATTACH":
            aco = AttachedCollisionObject()
            aco.link_name = "panda_hand"
            aco.object.id = "movable_box"
            aco.object.header.frame_id = "panda_hand"
            
            # Геометрия куба обязательна даже при ATTACH в некоторых версиях ROS 2
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [self.box_size] * 3
            aco.object.primitives.append(box)

            p = Pose()
            p.position.z = 0.103 # Центр куба относительно фланца
            p.orientation.w = 1.0
            aco.object.primitive_poses.append(p)
            aco.touch_links = ["panda_leftfinger", "panda_rightfinger", "panda_hand"]
            
            # Удаляем из мира
            co_remove = CollisionObject()
            co_remove.id = "movable_box"
            co_remove.operation = CollisionObject.REMOVE
            scene.world.collision_objects.append(co_remove)
            scene.robot_state.attached_collision_objects.append(aco)

        elif action == "DETACH":
            # 1. Сначала говорим: убери объект с робота
            aco = AttachedCollisionObject()
            aco.link_name = "panda_hand"
            aco.object.id = "movable_box"
            aco.object.operation = CollisionObject.REMOVE
            scene.robot_state.attached_collision_objects.append(aco)

            # 2. Добавляем его обратно в мир в точку p2 (ОБЯЗАТЕЛЬНО С ГЕОМЕТРИЕЙ)
            co = CollisionObject()
            co.id = "movable_box"
            co.header.frame_id = "panda_link0"
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [self.box_size] * 3
            co.primitives.append(box)
            p = Pose()
            p.position.x, p.position.y, p.position.z = self.p2[0], self.p2[1], self.p2[2]
            p.orientation.w = 1.0
            co.primitive_poses.append(p)
            co.operation = CollisionObject.ADD
            scene.world.collision_objects.append(co)

        self.apply_scene_sync(scene)

    def move_gripper(self, width):
        self.get_logger().info(f"Гриппер -> {width} м")
        
        # Создаем объект Goal
        goal_msg = GripperCommand.Goal()
        
        # Заполняем вложенное поле 'command'
        # .position — ширина раскрытия, .max_effort — сила сжатия
        goal_msg.command.position = width
        goal_msg.command.max_effort = 10.0 

        # Ждем сервер (чтобы не упасть с AttributeError)
        if not self.gripper_action.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Action сервер гриппера не найден!")
            return

        return self.gripper_action.send_goal_async(goal_msg)

    def run_mission(self):
        q_down = self.get_quaternion_down()

        # --- ГЕОМЕТРИЯ (Центр куба у нас 0.316) ---
        # 0.103 — это расстояние от фланца до центра пальцев гриппера Panda
        # Мы добавляем +0.105, чтобы быть на пару миллиметров выше и не бить стол
        z_pick = self.p1[2] + 0.105  # Итого ~0.421
        z_up = self.p1[2] + 0.25     # Высота полета ~0.566
        
        # Точка сброса p2 на той же высоте
        z_drop = self.p2[2] + 0.105

        # --- ШАГ 1: ПОДЛЕТ ---
        self.get_logger().info('1. Подлет над кубом')
        self.plan_and_move([
            self.create_pose(self.p1[0], self.p1[1], z_up, q_down)
        ])

        # --- ШАГ 2: СПУСК ---
        self.get_logger().info('2. Опускаюсь к кубу')
        # Включаем True! Если расчет неверный, MoveIt просто выдаст ошибку, 
        # но НЕ пустит робота в стол.
        success = self.plan_and_move([
            self.create_pose(self.p1[0], self.p1[1], z_pick, q_down)
        ], avoid_collisions=True)

        # Если 0.331 слишком низко для коллизий, пробуем принудительно (на свой страх и риск)
        if not success:
            self.get_logger().warn('MoveIt боится стола, пробую опуститься принудительно...')
            self.plan_and_move([
                self.create_pose(self.p1[0], self.p1[1], z_pick, q_down)
            ], avoid_collisions=False)

        # --- ШАГ 3: ЗАХВАТ ---
        self.get_logger().info('3. Сжимаю гриппер')
        self.move_gripper(0.01) 
        self.update_scene_object(action="ATTACH")
        time.sleep(0.5)

        # --- ШАГ 4: ПОДЪЕМ ---
        self.get_logger().info('4. Подъем куба')
        # После ATTACH всегда используем False, иначе он боится куба в руках
        self.plan_and_move([
            self.create_pose(self.p1[0], self.p1[1], z_up, q_down)
        ], avoid_collisions=False)

        # --- ШАГ 5: ПЕРЕНОС ---
        self.get_logger().info('5. Перенос в точку p2')
        self.plan_and_move([
            self.create_pose(self.p2[0], self.p2[1], z_up, q_down),
            self.create_pose(self.p2[0], self.p2[1], z_drop, q_down)
        ], avoid_collisions=False)

        self.get_logger().info('6. Отпускаю куб')
        self.move_gripper(0.04) # Раскрываем пальцы
        
        # Сначала отсоединяем объект
        self.update_scene_object(action="DETACH")
        
        # ВАЖНО: Даем время MoveIt "забыть" про куб перед полетом
        time.sleep(1.0) 
        
        # --- ШАГ 7: ОТЛЕТ (с игнорированием коллизий, чтобы точно улетел) ---
        self.get_logger().info('7. Завершение миссии - Отлет')
        self.plan_and_move([
            self.create_pose(self.p2[0], self.p2[1], self.p2[2] + 0.2, q_down)
        ], avoid_collisions=False) # Ставим False, чтобы уйти от куба без ошибок
        
    def plan_and_move(self, waypoints,avoid_collisions=True):
        req = GetCartesianPath.Request()
        req.header.frame_id = "panda_link0"
        req.group_name = "panda_arm"
        req.waypoints = waypoints
        req.max_step = 0.01
        req.jump_threshold = 0.0

    # --- ДОБАВЬ ЭТИ ДВЕ СТРОЧКИ ---
    # Это уберет ошибки про 0.000000 в логах и заставит робота двигаться плавно
        req.max_velocity_scaling_factor = 0.1     # 10% от макс. скорости
        req.max_acceleration_scaling_factor = 0.1 # 10% от макс. ускорения
    # ------------------------------

        req.avoid_collisions = avoid_collisions
    
        # Прямо перед планированием просим актуальное состояние
        start_state = RobotState()
        start_state.is_diff = True 
        # Если куб уже прикреплен, он ДОЛЖЕН быть в этом состоянии
        req.start_state = start_state

        # Добавь эти параметры, чтобы MoveIt не игнорировал запрос
        req.max_velocity_scaling_factor = 0.1
        req.max_acceleration_scaling_factor = 0.1

        future = self.cartesian_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

    # Проверка на успех (теперь логи будут показывать реальный процент)
        if res and res.fraction > 0.8:
        # Если хочешь, тут можно вставить блок с "time_from_start", 
        # который мы обсуждали раньше, чтобы еще сильнее замедлить движение.
        
            goal = ExecuteTrajectory.Goal(trajectory=res.solution)
            self.execute_action.wait_for_server()
    
            self.get_logger().info(f"Выполняю движение (успех: {res.fraction*100:.1f}%)")
            send_goal_future = self.execute_action.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_goal_future)

            result_future = send_goal_future.result().get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            return True
    
        self.get_logger().error(f"Не удалось построить путь. Fraction: {res.fraction if res else 'None'}")
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