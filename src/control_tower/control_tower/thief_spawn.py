import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose 
from std_msgs.msg import Int32
from gazebo_msgs.srv import SpawnEntity
import random
import os

class ThiefSpawn(Node):
    def __init__(self):
        super().__init__('thief_spawn')
        
        self.spawn_service = self.create_client(
            SpawnEntity,
            '/spawn_entity'
        )
        
        self.spawn_service.wait_for_service()
        
        self.thief_publisher = self.create_publisher(
            Int32,
            'thief_spawn',
            10
        )

        self.coordinates = [
            #(1.0, 7.0, 0.5),
            (1.0, 3.0, 0.5),
        ]
        
        self.spawn_random_object()
    
    def spawn_random_object(self):
        x, y, z = random.choice(self.coordinates)
    
        # 모델 파일 경로 읽기
        sdf_file_path = os.path.join(
            os.getenv('HOME'), 
            'person_walking', 
            'model.sdf'
        )
    
        if not os.path.exists(sdf_file_path):
            self.get_logger().error(f"모델링을 읽어올 수 없음: {sdf_file_path}")
            return
    
        with open(sdf_file_path, 'r') as sdf_file:
            sdf_data = sdf_file.read()
        
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        request = SpawnEntity.Request()
        request.name = 'thief_spawn'
        request.xml = sdf_data
        request.robot_namespace = ''
        request.initial_pose = pose

        future = self.spawn_service.call_async(request)
        future.add_done_callback(self.spawn_response_callback)
        
    def thief_spawn_topic(self):
        msg = Int32()
        msg.data = 1
        self.thief_publisher.publish(msg)
        self.get_logger().info('토픽 발행 완료')
        
    def spawn_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info("도둑 등장!")
            self.thief_spawn_topic()
        except Exception as e:
            self.get_logger().error(f"도둑 생성 실패 : {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ThiefSpawn()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
