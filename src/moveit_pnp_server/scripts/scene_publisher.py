#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

class ScenePublisher(Node):
    def __init__(self):
        super().__init__('scene_publisher')
        self.pub = self.create_publisher(CollisionObject, '/collision_object', 10)
        self.timer = self.create_timer(1.0, self.publish_scene)

    def publish_scene(self):
        objects = [
            {
                "id": "box1",
                "dims": [0.3, 0.5, 0.3],
                "pos": [0.4, 0, 0.15],
                "ori": [0,0,0,1]
            },
            {
                "id": "box2",
                "dims": [0.5, 0.3, 0.3],
                "pos": [0,0.4,0.15],
                "ori": [0,0,0,1]
            }
        ]

        for obj_data in objects:
            obj = CollisionObject()
            obj.header.frame_id = "world"
            obj.id = obj_data["id"]
            obj.operation = CollisionObject.ADD

            # Box primitive
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [float(d) for d in obj_data["dims"]]
            obj.primitives.append(box)

            # Pose
            pose = Pose()
            pose.position.x = float(obj_data["pos"][0])
            pose.position.y = float(obj_data["pos"][1])
            pose.position.z = float(obj_data["pos"][2])
            pose.orientation.x = float(obj_data["ori"][0])
            pose.orientation.y = float(obj_data["ori"][1])
            pose.orientation.z = float(obj_data["ori"][2])
            pose.orientation.w = float(obj_data["ori"][3])
            obj.primitive_poses.append(pose)

            self.pub.publish(obj)
            self.get_logger().info(f'Published {obj.id}')

def main(args=None):
    rclpy.init(args=args)
    node = ScenePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
