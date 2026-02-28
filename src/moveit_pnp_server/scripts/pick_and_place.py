#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time

from moveit_msgs.srv import GetMotionPlan, GetCartesianPath, ApplyPlanningScene
from moveit_msgs.action import ExecuteTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import Constraints, PositionConstraint, PlanningScene, AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import GripperCommand

class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place')
        self.GRIPPER_OFFSET = 0.058
        
        # Клиент для обновления сцены (нужен для Attach)
        self.scene_cli = self.create_client(ApplyPlanningScene, 'apply_planning_scene')
        
        self.kinematic_srv = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.cartesian_srv = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.execute_action = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.gripper_action = ActionClient(self, GripperCommand, '/panda_hand_controller/gripper_cmd')
        
        self.get_logger().info('System Ready with Object Attachment support.')

    def attach_object(self, object_id="movable_box"):
        """ Логически привязывает объект к руке робота """
        aco = AttachedCollisionObject()
        aco.link_name = "panda_link8" # Линк, к которому привязываем
        aco.object.id = object_id
        aco.object.header.frame_id = "panda_link8"
        
        # Указываем, какие части робота могут касаться объекта (пальцы),
        # чтобы MoveIt не считал это столкновением-аварией
        aco.touch_links = ["panda_leftfinger", "panda_rightfinger", "panda_hand"]
        
        # Важно: Переводим объект из статуса "в мире" в статус "привязан"
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.attached_collision_objects.append(aco)
        scene.robot_state.is_diff = True
        
        req = ApplyPlanningScene.Request()
        req.scene = scene
        self.scene_cli.call_async(req)
        self.get_logger().info(f'Object {object_id} attached to the gripper.')

    def set_gripper(self, position):
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 30.0
        self.gripper_action.wait_for_server()
        self.gripper_action.send_goal_async(goal)
        time.sleep(1.5)

    def plan_kinematic(self, x, y, z):
        req = GetMotionPlan.Request()
        mpr = req.motion_plan_request
        mpr.group_name = "panda_arm"
        mpr.start_state.is_diff = True
        
        target_pose = PoseStamped()
        target_pose.header.frame_id = "panda_link0"
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z + self.GRIPPER_OFFSET
        target_pose.pose.orientation.x = 1.0 
        
        goal_const = Constraints()
        pos_const = PositionConstraint()
        pos_const.header.frame_id = "panda_link0"
        pos_const.link_name = "panda_link8"
        pos_const.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01]))
        pos_const.constraint_region.primitive_poses.append(target_pose.pose)
        goal_const.position_constraints.append(pos_const)
        mpr.goal_constraints.append(goal_const)

        future = self.kinematic_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res and res.motion_plan_response.error_code.val == 1:
            self.execute(res.motion_plan_response.trajectory)
            return True
        return False

    def plan_cartesian(self, x, y, z):
        req = GetCartesianPath.Request()
        req.header.frame_id = "panda_link0"
        req.group_name = "panda_arm"
        req.link_name = "panda_link8"
        
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z + self.GRIPPER_OFFSET
        target_pose.orientation.x = 1.0 
        
        req.waypoints = [target_pose]
        req.max_step = 0.01
        
        future = self.cartesian_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res and res.fraction > 0.9:
            self.execute(res.solution)
            return True
        return False

    def execute(self, trajectory):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        self.execute_action.wait_for_server()
        self.execute_action.send_goal_async(goal)
        time.sleep(3.0)

def main():
    rclpy.init()
    node = PickAndPlace()
    X, Y, Z = 0.4, 0.0, 0.35

    # 1. Подготовка
    node.set_gripper(0.04)

    # 2. Подлет
    node.get_logger().info('Moving to hover point...')
    node.plan_kinematic(X, Y, Z + 0.15)

    # 3. Спуск
    node.get_logger().info('Descending...')
    node.plan_cartesian(X, Y, Z)

    # 4. ЗАХВАТ
    node.get_logger().info('Grasping...')
    node.set_gripper(0.022)
    
    # --- ВОТ ТУТ МАГИЯ ---
    # Мы вручную говорим MoveIt, что теперь кубик "в руке"
    node.attach_object("movable_box")
    # ---------------------

    # 5. Подъем (теперь кубик полетит вверх вместе с рукой)
    node.get_logger().info('Lifting up...')
    node.plan_cartesian(X, Y, Z + 0.15)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()