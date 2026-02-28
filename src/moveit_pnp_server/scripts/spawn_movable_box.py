#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene, ObjectColor
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from moveit_msgs.srv import ApplyPlanningScene

class SpawnBoxNode(Node):
    def __init__(self):
        super().__init__('spawn_box_node')

        # Клиент к сервису MoveIt для обновления сцены
        self.cli = self.create_client(ApplyPlanningScene, 'apply_planning_scene')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Ожидание сервиса MoveIt...')

        self.spawn_box()

    def spawn_box(self):
        # 1. Описание объекта коллизии
        co = CollisionObject()
        co.id = "movable_box"
        co.header.frame_id = "panda_link0" # Базовый фрейм робота

        # Твои размеры: 0.03 x 0.03 x 0.03
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.03, 0.03, 0.03]

        # Твоя позиция: 0.4, 0.0, 0.35
        pose = Pose()
        pose.position.x = 0.4
        pose.position.y = 0.0
        pose.position.z = 0.35
        pose.orientation.w = 1.0

        co.primitives.append(box)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD

        # 2. Настройка цвета (Красный)
        color = ObjectColor()
        color.id = "movable_box"
        color.color.r = 1.0
        color.color.g = 0.0
        color.color.b = 0.0
        color.color.a = 1.0

        # 3. Формирование сцены
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(co)
        scene.object_colors.append(color)

        # 4. Отправка запроса
        req = ApplyPlanningScene.Request()
        req.scene = scene
        
        self.get_logger().info("Отправка запроса на спавн кубика...")
        self.future = self.cli.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = SpawnBoxNode()
    
    # Ждем подтверждения от MoveIt, прежде чем закрыть ноду
    rclpy.spin_until_future_complete(node, node.future)
    
    if node.future.result() is not None:
        node.get_logger().info("✅ Кубик успешно заспавнен!")
    else:
        node.get_logger().error("❌ Ошибка при спавне кубика")
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()