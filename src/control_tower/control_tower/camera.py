import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import FollowWaypoints
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import Int32

from PIL import Image as PILImage, ImageTk
from tkinter import Tk, Canvas, Label

from ultralytics import YOLO
from cv_bridge import CvBridge
from pathlib import Path

import os
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose 

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

        # '/thief_spawn' 토픽 구독
        self.thief_subscription = self.create_subscription(
            Int32,
            '/thief_spawn',
            self.thief_spawn_callback,
            10
        ) 

        # tb1 순찰 퍼블리셔
        self.tb1_find_publisher = ActionClient(
            self,
            FollowWaypoints,
            '/tb1/follow_waypoints'
        )
        
        # tb2 지원 퍼블리셔
        self.tb2_go_publisher = ActionClient(
            self,
            NavigateToPose,
            '/tb2/navigate_to_pose'
        )

        # tb1 도둑 중앙 보정 퍼블리셔
        self.tb1_follow_publisher = self.create_publisher(
            Twist,
            '/tb1/cmd_vel',
            10
        )

        # tb2 도둑 중앙 보정 퍼블리셔
        self.tb2_follow_publisher = self.create_publisher(
            Twist,
            '/tb2/cmd_vel',
            10
        )

        self.spawn_ball_publisher = self.create_client(
            SpawnEntity,
            '/spawn_entity'
        )
        self.spawn_ball_publisher.wait_for_service()

        # 초기 위치 설정
        self.x_tb1 = 0.0
        self.y_tb1 = 0.0        
        self.x_tb2 = 0.0
        self.y_tb2 = 0.0

        # 도둑 스폰 확인
        self.thief_check = False

        # 도둑 찾음 여부
        self.tb1_thief_find = False
        self.tb2_thief_find = False

        # tb2 출발 신호 여부
        self.tb2_go = False

        # 도둑 스탑 카운트
        self.stop_count = 0

        # 도둑 체포 카운트
        self.arrest_count = 0

        # 도둑 바운딩 박스 크기
        self.tb1_area = 0
        self.tb2_area = 0

        # 도둑 좌표
        self.thief_x = 0.0
        self.thief_y = 0.0
        self.thief_z = 0.0

        self.tb1_thief_x_center = 0.0
        self.tb1_thief_y_center = 0.0
        self.tb2_thief_x_center = 0.0
        self.tb2_thief_y_center = 0.0        

        # tb1 순찰 지점
        self.waypoints = [
            (4.1, 6.5),
            (3.2, 5.8),
            (4.1, 6.5),
            (-4.6, 6.3),
            (-4.6, 1.7),
            (-4.6, -3.2),
            (3.2, -4.0),
        ]
        self.current_waypoint_idx = 0

        # 이미지 변환을 위한 CvBridge 객체
        self.bridge = CvBridge()

        # Yolov8 모델 로드
        self.model = YOLO("yolov8n.pt")

        # tb1 카메라 프레임
        self.tb1_camera_frame = None
        self.tb2_camera_frame = None

        # 감지 객체 저장 변수
        self.tb1_detection_info = ""
        self.tb2_detection_info = ""

        # 타이머 설정
        self.timer_detection = self.create_timer(0.5, self.tb1_detection_publish)
        self.timer_detection = self.create_timer(0.5, self.tb2_detection_publish)

        # tb1 waypoint 취소 변수
        self._goal_handle = None
        self.is_goal_completed = False

        # tb2 waypoint 취소 변수
        self.goal_handle = None

    def spawn_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info("<<<<<<<<< 체포 완료 >>>>>>>>>")
        except Exception as e:
            self.get_logger().error(f"<< 체포 실패 >> {e}")

    def spawn_ball_object(self):
        x = self.thief_x
        y = self.thief_y
        z = 7.0
    
        # 모델 파일 경로 읽기
        sdf_file_path = os.path.join(
            os.getenv('HOME'), 
            'submarine', 
            'model.sdf'
        )
    
        if not os.path.exists(sdf_file_path):
            self.get_logger().error(f"모델링을 읽어올 수 없음: {sdf_file_path}")
            return
    
        with open(sdf_file_path, 'r') as sdf_file:
            sdf_data = sdf_file.read()
        
        pose = Pose()
        pose.position.x = x + 1.0
        pose.position.y = y
        pose.position.z = z

        request = SpawnEntity.Request()
        request.name = 'arrest'
        request.xml = sdf_data
        request.robot_namespace = ''
        request.initial_pose = pose

        future = self.spawn_ball_publisher.call_async(request)
        future.add_done_callback(self.spawn_response_callback)

    def tb2_send_goal(self):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = self.thief_x
        goal.pose.pose.position.y = self.thief_y
        goal.pose.pose.position.z = self.thief_z
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = 0.0
        goal.pose.pose.orientation.w = 1.0

        # 목표 전송
        goal_handle_future = self.tb2_go_publisher.send_goal_async(goal, feedback_callback = self.tb2_feedback)
        goal_handle_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('tb2 목표 전달 실패')
            return
        self.get_logger().info('<<<<<<<<< tb2 지원 출동 >>>>>>>>>')
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def tb2_feedback(self, feedback_msg):
        action_feedback = feedback_msg.feedback
        #self.get_logger().info(f"Action feedback: {action_feedback}")

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'tb2 목표 도달')


    def cancel_current_goal(self):
        if self._goal_handle is not None and not self.is_goal_completed:
            self.get_logger().info("tb1 도둑 발견 / 이동 중지")
            self.tb1_find_publisher._cancel_goal_async(self._goal_handle)
            self._goal_handle = None
            self.is_goal_completed = False

    def tb2_cancel_current_goal(self):
        if self.goal_handle is not None:
            self.get_logger().info("tb2 도둑 발견 / 이동 중지")
            self.tb2_go_publisher._cancel_goal_async(self.goal_handle)
            self.goal_handle = None

    def tb1_see_thief(self):
        #self.get_logger().info(f"^^^^^^^^^^^^{self.tb1_thief_y_center}^^^^^^^^^^^")
        if self.tb1_thief_x_center < 310:
            self.get_logger().info("tb1 왼쪽 회전")
            
            move_cmd = Twist()
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.03
            self.tb1_follow_publisher.publish(move_cmd)

        elif 330 < self.tb1_thief_x_center:
            self.get_logger().info("tb1 오른쪽 회전")
            
            move_cmd = Twist()
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = -0.03
            self.tb1_follow_publisher.publish(move_cmd)

        elif self.tb1_thief_y_center < 145:
            self.get_logger().info("tb1 접근")

            move_cmd = Twist()
            move_cmd.linear.x = 0.1
            move_cmd.angular.z = 0.0
            self.tb1_follow_publisher.publish(move_cmd)

        else:
            self.get_logger().info("tb1 정지")
            
            move_cmd = Twist()
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            self.tb1_follow_publisher.publish(move_cmd)

            self.stop_count += 1

            if self.stop_count == 5:

                # 도둑을 발견한 tb1의 좌표
                self.thief_x = self.x_tb1
                self.thief_y = self.y_tb1
                self.thief_z = 0.5

                # tb2 이동
                self.get_logger().info("<<<<<<<<< 지원 요청 >>>>>>>>>")
                self.tb2_send_goal()
                self.tb2_go = True

    def tb2_see_thief(self):
        #self.get_logger().info(f"@@@@@@@@@@@{self.tb2_thief_y_center}@@@@@@@@@@")

        if self.tb2_thief_x_center < 310:
            self.get_logger().info("tb2 왼쪽 회전")
            
            move_cmd = Twist()
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.03
            self.tb2_follow_publisher.publish(move_cmd)

        elif 330 < self.tb2_thief_x_center:
            self.get_logger().info("tb2 오른쪽 회전")
            
            move_cmd = Twist()
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = -0.03
            self.tb2_follow_publisher.publish(move_cmd)

        elif self.tb2_thief_y_center < 145:
            self.get_logger().info("tb2 접근")

            move_cmd = Twist()
            move_cmd.linear.x = 0.1
            move_cmd.angular.z = 0.0
            self.tb2_follow_publisher.publish(move_cmd)

        else:
            self.get_logger().info("tb2 정지")
            
            move_cmd = Twist()
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            self.tb2_follow_publisher.publish(move_cmd)

            self.arrest_count += 1

            if self.arrest_count == 5:
                self.spawn_ball_object()


    def tb1_detection_publish(self):
        results = self.model(self.tb1_camera_frame)

        self.tb1_detection_info = self.tb1_draw_bounding_boxes(self.tb1_camera_frame, results)

        if 'person' in self.tb1_detection_info and self.tb1_area > 10000 and self.thief_check == True:
            # 도둑 감지 여부
            self.tb1_thief_find = True

            # tb1 이동 중지
            self.cancel_current_goal()

            # tb1 도둑 보기
            self.tb1_see_thief()

        # 도둑 감지 했으나 시야에 없을 경우
        elif 'person' not in self.tb1_detection_info and self.tb1_thief_find == True:
            if self.tb1_thief_x_center < 310:
                self.get_logger().info("tb1 시야 이탈 / 왼쪽 회전")
                move_cmd = Twist()
                move_cmd.angular.z = 0.3
                self.tb1_follow_publisher.publish(move_cmd)
            
            elif 300 < self.tb1_thief_x_center:
                self.get_logger().info("tb1 시야 이탈 / 오른쪽 회전")
                move_cmd = Twist()
                move_cmd.angular.z = -0.3
                self.tb1_follow_publisher.publish(move_cmd)

    def tb2_detection_publish(self):
        results = self.model(self.tb2_camera_frame)

        self.tb2_detection_info = self.tb2_draw_bounding_boxes(self.tb2_camera_frame, results)

        if 'person' in self.tb2_detection_info and self.tb2_area > 10000 and self.tb2_go == True:
            # 도둑 감지 여부
            self.tb2_thief_find = True

            # tb2 이동 중지
            self.tb2_cancel_current_goal()

            # tb2 도둑 보기
            self.tb2_see_thief()

        # 도둑 감지 했으나 시야에 없을 경우
        elif 'person' not in self.tb2_detection_info and self.tb2_thief_find == True:
            if self.tb2_thief_x_center < 310:
                self.get_logger().info("tb2 시야 이탈 / 왼쪽 회전")
                move_cmd = Twist()
                move_cmd.angular.z = 0.3
                self.tb2_follow_publisher.publish(move_cmd)
            
            elif 300 < self.tb2_thief_x_center:
                self.get_logger().info("tb2 시야 이탈 / 오른쪽 회전")
                move_cmd = Twist()
                move_cmd.angular.z = -0.3
                self.tb2_follow_publisher.publish(move_cmd)

    # 클래스별로 신뢰도가 가장 높은 객체만 표시
    def tb1_draw_bounding_boxes(self, frame, results):
        class_best_boxes = {}

        for result in results[0].boxes.data.tolist():
            x1, y1, x2, y2 = map(int, result[:4])
            score = result[4]
            class_id = int(result[5])
            class_name = self.model.names[class_id]

            # person 바운딩 박스 넓이 저장
            if class_id == 0:
                width = x2 - x1
                height = y2 - y1
                self.tb1_area = width * height
                self.tb1_thief_x_center = (x1 + x2) / 2
                self.tb1_thief_y_center = (y1 + y2) / 2

            if class_id not in class_best_boxes:
                class_best_boxes[class_id] = {"score": score, "bbox": (x1, y1, x2, y2), "class_name": class_name}
            elif score > class_best_boxes[class_id]["score"]:
                class_best_boxes[class_id] = {"score": score, "bbox": (x1, y1, x2, y2), "class_name": class_name}

        detection_info = ""
        for class_id, best_box in class_best_boxes.items():
            x1, y1, x2, y2 = best_box["bbox"]
            score = best_box["score"]
            class_name = best_box["class_name"]
            detection_info += f"{class_name} ({score:.2f})\n"

        return detection_info
    
    # 클래스별로 신뢰도가 가장 높은 객체만 표시
    def tb2_draw_bounding_boxes(self, frame, results):
        class_best_boxes = {}

        for result in results[0].boxes.data.tolist():
            x1, y1, x2, y2 = map(int, result[:4])
            score = result[4]
            class_id = int(result[5])
            class_name = self.model.names[class_id]

            # person 바운딩 박스 넓이 저장
            if class_id == 0:
                width = x2 - x1
                height = y2 - y1
                self.tb2_area = width * height
                self.tb2_thief_x_center = (x1 + x2) / 2
                self.tb2_thief_y_center = (y1 + y2) / 2

            if class_id not in class_best_boxes:
                class_best_boxes[class_id] = {"score": score, "bbox": (x1, y1, x2, y2), "class_name": class_name}
            elif score > class_best_boxes[class_id]["score"]:
                class_best_boxes[class_id] = {"score": score, "bbox": (x1, y1, x2, y2), "class_name": class_name}

        detection_info = ""
        for class_id, best_box in class_best_boxes.items():
            x1, y1, x2, y2 = best_box["bbox"]
            score = best_box["score"]
            class_name = best_box["class_name"]
            detection_info += f"{class_name} ({score:.2f})\n"

        return detection_info

    def odom_callback_tb2(self, msg):
        self.x_tb2 = msg.pose.pose.position.x
        self.y_tb2 = msg.pose.pose.position.y

        #self.get_logger().info(f'TB2 Position: x={self.x_tb2}, y={self.y_tb2}')
        self.map_viewer.update_robot_position(self.x_tb2, self.y_tb2, robot='tb2')

    def odom_callback_tb1(self, msg):
        self.x_tb1 = msg.pose.pose.position.x
        self.y_tb1 = msg.pose.pose.position.y

        #self.get_logger().info(f'TB1 Position: x={self.x_tb1}, y={self.y_tb1}')
        self.map_viewer.update_robot_position(self.x_tb1, self.y_tb1, robot='tb1')

    # /tb1/camera/image_raw' 토픽의 카메라 이미지
    def tb1_camera_callback(self, msg):
        try:
            # ROS 이미지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.tb1_camera_frame = cv_image

            # OpenCV 이미지를 Tkinter에서 사용할 수 있는 형식으로 변환
            pil_image = PILImage.fromarray(cv_image)
            self.map_viewer.update_tb1_camera_image(pil_image)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    # /tb2/camera/image_raw' 토픽의 카메라 이미지
    def tb2_camera_callback(self, msg):
        try:
            # ROS 이미지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.tb2_camera_frame = cv_image

            # OpenCV 이미지를 PIL 이미지로 변환하여 Tkinter에서 사용할 수 있도록 함
            pil_image = PILImage.fromarray(cv_image)
            self.map_viewer.update_tb2_camera_image(pil_image)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def send_next_waypoint(self):
        if self.current_waypoint_idx < len(self.waypoints):
            goal = FollowWaypoints.Goal()

            goal.poses = []
            # 목표 설정
            for waypoint in self.waypoints[self.current_waypoint_idx:]:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = waypoint[0]
                pose.pose.position.y = waypoint[1]
                pose.pose.orientation.w = 1.0

                goal.poses.append(pose)
            
            # 목표 발행
            future = self.tb1_find_publisher.send_goal_async(goal, feedback_callback = self.feedback_callback)
            future.add_done_callback(self.goal_handle_callback)

    def feedback_callback(self, feedback_msg):
        current_waypoint = feedback_msg.feedback.current_waypoint
        self.get_logger().info(f'{current_waypoint+1}번째 순찰구역 이동 중')
        if current_waypoint >= len(self.waypoints) - 1:
            self.is_goal_completed = True

    def goal_handle_callback(self, future):
        try:
            self._goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Goal handling failed: {e}") 

    def thief_spawn_callback(self, msg):
        temp = msg.data
        if temp == 1:
            self.thief_check = True
            self.get_logger().info("thief_spawn")

            self.send_next_waypoint()
        else:
            pass


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
        self.camera_label_tb1 = Label(root, text="Police Detector Cam", compound="bottom", font=("*font", 16))
        self.camera_label_tb1.grid(row=0, column=1)

        # tb2 카메라 이미지를 표시할 라벨
        self.camera_label_tb2 = Label(root, text="Police Attacker Cam", compound="bottom", font=("*font", 16))
        self.camera_label_tb2.grid(row=0, column=2)

    # YAML 파일에서 맵 데이터 로드
    def load_map_data(self, map_file):
        with open(map_file, 'r') as file:
            map_data = yaml.safe_load(file)
        
        map_path = Path(map_file).parent / map_data['image']

        # 상대 경로를 절대 경로로 변경
        map_data['image'] = str(map_path)

        return map_data

    # 로봇 위치 맵 업데이트
    def update_robot_position(self, x, y, robot='tb2'):
        # 원점 중앙 설정, 해상도 고려 맵 상의 위치 계산
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

    # 카메라 이미지 Tkinter 라벨 업데이트
    def update_tb1_camera_image(self, pil_image):
        """하는 함수"""
        pil_image = pil_image.resize((448, 336))
        camera_photo = ImageTk.PhotoImage(pil_image)
        self.camera_label_tb1.config(image=camera_photo)
        # 이미지 라벨에 할당하여 계속 갱신
        self.camera_label_tb1.image = camera_photo

    # 카메라 이미지 Tkinter 라벨 업데이트
    def update_tb2_camera_image(self, pil_image):
        """카메라 이미지를 Tkinter 라벨에 업데이트하는 함수"""
        pil_image = pil_image.resize((448, 336))
        camera_photo = ImageTk.PhotoImage(pil_image)
        self.camera_label_tb2.config(image=camera_photo)
        # 이미지 라벨에 할당하여 계속 갱신
        self.camera_label_tb2.image = camera_photo



def main(args=None):
    rclpy.init(args=args)

    # Tkinter 애플리케이션 생성
    root = Tk()
    root.title("Control Tower")

    # map.yaml 파일 경로
    map_file = '/home/rokey/Desktop/map.yaml'

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
        
        rclpy.spin_once(odom_node)

        root.after(100, update_gui)  # 100ms마다 갱신

    # Tkinter GUI 실행
    update_gui()
    root.mainloop()


if __name__ == '__main__':
    main()