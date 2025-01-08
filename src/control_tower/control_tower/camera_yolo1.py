import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tkinter import Tk, Canvas, Label
from PIL import Image as PILImage, ImageTk
import yaml
from pathlib import Path
import threading

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image as ROSImage

class OdomNode(Node):
    def __init__(self, map_viewer):
        super().__init__('odom_listener')
        self.map_viewer = map_viewer

        # '/tb1/odom' 토픽 구독
        self.subscription_tb1 = self.create_subscription(
            Odometry,
            '/tb1/odom',
            self.odom_callback_tb1,
            10
        )

        # '/tb2/odom' 토픽 구독
        self.subscription_tb2 = self.create_subscription(
            Odometry,
            '/tb2/odom',
            self.odom_callback_tb2,
            10
        )

        # '/tb1/camera/image_raw' 토픽 구독
        self.subscription_tb1_camera = self.create_subscription(
            ROSImage,
            '/tb1/camera/image_raw',
            self.tb1_camera_callback,
            10
        )

        # '/tb2/camera/image_raw' 토픽 구독
        self.subscription_tb2_camera = self.create_subscription(
            ROSImage,
            '/tb2/camera/image_raw',
            self.tb2_camera_callback,
            10
        )

        # 초기 위치 설정
        self.x_tb2 = 0.0
        self.y_tb2 = 0.0
        self.x_tb1 = 0.0
        self.y_tb1 = 0.0

        # 이미지 변환을 위한 CvBridge 객체
        self.bridge = CvBridge()


    def odom_callback_tb2(self, msg):
        """'/tb2/odom' 토픽의 메시지를 처리하는 콜백"""
        self.x_tb2 = msg.pose.pose.position.x
        self.y_tb2 = msg.pose.pose.position.y
        self.get_logger().info(f'TB2 Position: x={self.x_tb2}, y={self.y_tb2}')
        self.map_viewer.update_robot_position(self.x_tb2, self.y_tb2, robot='tb2')

    def odom_callback_tb1(self, msg):
        """'/tb1/odom' 토픽의 메시지를 처리하는 콜백"""
        self.x_tb1 = msg.pose.pose.position.x
        self.y_tb1 = msg.pose.pose.position.y
        self.get_logger().info(f'TB1 Position: x={self.x_tb1}, y={self.y_tb1}')
        self.map_viewer.update_robot_position(self.x_tb1, self.y_tb1, robot='tb1')

    def tb1_camera_callback(self, msg):
        """'/tb1/camera/image_raw' 토픽의 카메라 이미지를 처리하는 콜백"""
        try:
            # ROS 이미지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # OpenCV 이미지를 Tkinter에서 사용할 수 있는 형식으로 변환
            pil_image = PILImage.fromarray(cv_image)
            self.map_viewer.update_tb1_camera_image(pil_image)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def tb2_camera_callback(self, msg):
        """'/tb2/camera/image_raw' 토픽의 이미지를 처리하는 콜백"""
        try:
            # ROS 이미지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # OpenCV 이미지를 PIL 이미지로 변환하여 Tkinter에서 사용할 수 있도록 함
            pil_image = PILImage.fromarray(cv_image)
            self.map_viewer.update_tb2_camera_image(pil_image)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")


class MapViewer:
    def __init__(self, root, map_file, resolution, origin):
        self.root = root
        self.map_file = map_file
        self.resolution = resolution
        self.origin = origin

        # 맵 데이터 로드 (YAML 파일에서)
        self.map_data = self.load_map_data(map_file)

        # 맵 이미지 로드
        self.map_image = PILImage.open(self.map_data['image'])

        # 이미지 크기 추출 (픽셀 단위)
        self.map_width, self.map_height = self.map_image.size

        # Tkinter 캔버스 설정
        self.canvas = Canvas(root, width=800, height=800)
        self.canvas.grid(row=0, column=0)

        # 맵 이미지를 800x800 크기로 리사이즈
        self.map_image = self.map_image.resize((800, 800))
        self.map_photo = ImageTk.PhotoImage(self.map_image)

        # 캔버스에 맵 이미지 표시
        self.canvas.create_image(0, 0, anchor='nw', image=self.map_photo)

        # 로봇 위치를 표시할 점들 (TB2, TB1)
        self.robot_dot_tb2 = self.canvas.create_oval(
            400, 400, 405, 405, fill="red"  # TB2의 초기 위치 (중앙)
        )
        self.robot_dot_tb1 = self.canvas.create_oval(
            400, 400, 405, 405, fill="blue"  # TB1의 초기 위치 (중앙)
        )

        # tb1 카메라 이미지를 표시할 라벨
        self.camera_label_tb1 = Label(root)
        self.camera_label_tb1.grid(row=0, column=1)

        # tb2 카메라 이미지를 표시할 라벨
        self.camera_label_tb2 = Label(root)
        self.camera_label_tb2.grid(row=0, column=2)

    def load_map_data(self, map_file):
        """YAML 파일에서 맵 데이터를 로드하는 함수"""
        with open(map_file, 'r') as file:
            map_data = yaml.safe_load(file)
        
        map_path = Path(map_file).parent / map_data['image']
        map_data['image'] = str(map_path)  # 상대 경로를 절대 경로로 변경

        return map_data

    def update_robot_position(self, x, y, robot='tb2'):
        """로봇의 위치를 맵에 업데이트하는 함수"""
        # 원점을 중앙으로 설정하고, 해상도를 고려하여 맵 상의 위치 계산
        canvas_x = (x - self.origin[0]) / self.resolution
        canvas_y = (y - self.origin[1]) / self.resolution

        # 맵 크기에 맞춰 비례적으로 위치 계산
        scale_x = 800 / self.map_width
        scale_y = 800 / self.map_height

        # 실제 맵 크기 비율을 고려하여 위치 조정
        canvas_x *= scale_x
        canvas_y *= scale_y

        # y 좌표 반전 (왼쪽 하단 -> 왼쪽 상단)
        canvas_y = 800 - canvas_y

        # 로봇 점 위치 업데이트
        if robot == 'tb2':
            self.canvas.coords(self.robot_dot_tb2, canvas_x - 5, canvas_y - 5, canvas_x + 5, canvas_y + 5)
        elif robot == 'tb1':
            self.canvas.coords(self.robot_dot_tb1, canvas_x - 5, canvas_y - 5, canvas_x + 5, canvas_y + 5)


    def update_tb1_camera_image(self, pil_image):
        """카메라 이미지를 Tkinter 라벨에 업데이트하는 함수"""
        pil_image = pil_image.resize((400, 400))  # 카메라 이미지를 라벨 크기에 맞게 리사이즈
        camera_photo = ImageTk.PhotoImage(pil_image)
        self.camera_label_tb1.config(image=camera_photo)
        self.camera_label_tb1.image = camera_photo  # 이미지를 라벨에 할당하여 계속 갱신되도록 합니다.

    def update_tb2_camera_image(self, pil_image):
        """카메라 이미지를 Tkinter 라벨에 업데이트하는 함수"""
        pil_image = pil_image.resize((400, 400))  # 카메라 이미지를 라벨 크기에 맞게 리사이즈
        camera_photo = ImageTk.PhotoImage(pil_image)
        self.camera_label_tb2.config(image=camera_photo)
        self.camera_label_tb2.image = camera_photo  # 이미지를 라벨에 할당하여 계속 갱신되도록 합니다.



def main(args=None):
    rclpy.init(args=args)

    # Tkinter 애플리케이션 생성
    root = Tk()
    root.title("Control Tower")

    # map.yaml 파일 경로
    map_file = '/home/rokey/Desktop/map.yaml'  # 실제 map.yaml 경로로 수정

    # 맵 해상도 및 원점 값
    resolution = 0.07  # m/픽셀
    origin = [-6.32, -6.87, 0]  # 맵 원점 (x, y)

    # Tkinter 기반 맵 뷰어 생성
    map_viewer = MapViewer(root, map_file, resolution, origin)

    # ROS 2 노드 생성
    odom_node = OdomNode(map_viewer)

    # 주기적으로 tkinter 윈도우 업데이트
    def update_gui():
        # 실시간으로 좌표를 tkinter 맵에 업데이트
        map_viewer.update_robot_position(odom_node.x_tb2, odom_node.y_tb2, robot='tb2')
        map_viewer.update_robot_position(odom_node.x_tb1, odom_node.y_tb1, robot='tb1')
        root.after(100, update_gui)  # 100ms마다 갱신

    # ROS 2 노드를 spin하여 메시지 처리
    def ros_spin():
        rclpy.spin(odom_node)

    # ROS spin을 별도의 스레드에서 실행
    ros_thread = threading.Thread(target=ros_spin)
    ros_thread.daemon = True
    ros_thread.start()

    # Tkinter GUI 실행
    update_gui()
    root.mainloop()


if __name__ == '__main__':
    main()