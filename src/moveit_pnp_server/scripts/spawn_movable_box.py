#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene

class BoxSpawner(Node):
    def __init__(self):
        super().__init__('spawn_box_node')
        self.cli = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Сервис /apply_planning_scene недоступен, ждем...')

    def send_request(self):
        # Описываем объект
        collision_object = CollisionObject()
        collision_object.header.frame_id = "panda_link0"  # Имя базового линка твоего робота
        collision_object.id = "movable_box"
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.03, 0.03, 0.03] # Размеры куба
        
        pose = Pose()
        pose.position.x = 0.4
        pose.position.y = 0.0
        pose.position.z = 0.35
        pose.orientation.w = 1.0
        
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD
        
        # Запрос к планировщику
        request = ApplyPlanningScene.Request()
        request.scene.world.collision_objects.append(collision_object)
        request.scene.is_diff = True
        
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    spawner = BoxSpawner()
    response = spawner.send_request()
    
    if response and response.success:
        print("КУБИК УСПЕШНО ДОБАВЛЕН!")
    else:
        print("Ошибка: не удалось добавить кубик.")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()