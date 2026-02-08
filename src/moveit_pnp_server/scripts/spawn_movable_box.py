#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from moveit_msgs.srv import ApplyPlanningScene

class SpawnBoxNode(Node):
    def __init__(self):
        super().__init__('spawn_box_node')

        self.cli = self.create_client(ApplyPlanningScene, 'apply_planning_scene')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for apply_planning_scene service...')

        # --- Collision object ---
        co = CollisionObject()
        co.id = "movable_box"
        co.header.frame_id = "world"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.03, 0.03, 0.03]

        pose = Pose()
        pose.position.x = 0.4
        pose.position.y = 0.0
        pose.position.z = 0.35
        pose.orientation.w = 1.0

        co.primitives = [box]
        co.primitive_poses = [pose]
        co.operation = CollisionObject.ADD

        # --- Цвет (красный) ---
        color = ObjectColor()
        color.id = "movable_box"
        color.color.r = 1.0
        color.color.g = 0.0
        color.color.b = 0.0
        color.color.a = 1.0  # непрозрачный

        # --- Planning scene ---
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(co)
        scene.object_colors.append(color)

        req = ApplyPlanningScene.Request()
        req.scene = scene

        self.cli.call_async(req)
        self.get_logger().info("Red movable box spawned!")

def main(args=None):
    rclpy.init(args=args)
    node = SpawnBoxNode()
    rclpy.spin_once(node, timeout_sec=1)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
