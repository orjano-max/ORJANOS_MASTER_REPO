import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Point, Quaternion





class ScenePublisher(Node):
    def __init__(self, filename):
        super().__init__('scene_publisher')
        self.scene_pub = self.create_publisher(PlanningScene, 'planning_scene', 10)
        self.file = filename
        self.frame_id_ = "world"

    def load_scene(self):
        with open(self.file, 'r') as f:
            scene_str = f.read()
        lines = scene_str.splitlines()
        scene_name = lines.pop(0)  # first line is the scene name
        collision_objects = []
        for i, line in enumerate(lines):
            if line.startswith('*'):
                obj_name = line[2:]
                pose = lines[i+1].split(' ')
                quat = lines[i+2].strip().split(' ')
                scale = lines[i+3].split(' ')
                shape_type = lines[i+4].strip()
                if shape_type == 'box':
                    collision_obj = CollisionObject()
                    collision_obj.id = obj_name
                    collision_obj.header.frame_id = self.frame_id_
                    collision_obj.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(map(float, scale))))
                    collision_obj.primitive_poses.append(Pose(
                        position=Point(x=float(pose[0]), y=float(pose[1]), z=float(pose[2])),
                        orientation=Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3]))))
                    collision_objects.append(collision_obj)
                elif shape_type == 'cylinder':
                    collision_obj = CollisionObject()
                    collision_obj.id = obj_name
                    collision_obj.header.frame_id = self.frame_id_
                    collision_obj.primitives.append(SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=list(map(float, scale))))
                    collision_obj.primitive_poses.append(Pose(
                        position=Point(x=float(pose[0]), y=float(pose[1]), z=float(pose[2])),
                        orientation=Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3]))))
                    collision_objects.append(collision_obj)
        return collision_objects

    def publish_scene(self):
        msg = PlanningScene()
        msg.is_diff = True
        msg.robot_model_name = 'vx300'
        msg.world.collision_objects = self.load_scene()
        self.scene_pub.publish(msg)
        self.get_logger().info('Scene published.')

def main(args=None):
    rclpy.init(args=args)
    filename = '/home/orjan/git/ORJANOS_MASTER_REPO/code/params/scene_geometry.scene'
    node = ScenePublisher(filename)
    node.publish_scene()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()