# ============================================================
# assembly_controller.py
# ============================================================
# [EN]
# Main controller for managing the overall assembly sequence.
# Receives target information from the Vision System and
# controls the assembly process.
#
# Main Responsibilities:
# - Receive target position
# - Approach the target
# - Align with the target
# - Perform insertion
# - Manage assembly completion
#
# [KR]
# 전체 조립 작업 순서를 관리하는 메인 Controller.
# Vision System에서 Target 정보를 받아 조립 작업을 수행한다.
#
# 주요 역할:
# - Target 위치 수신
# - Target으로 접근
# - Target 위치 정렬
# - 도형 삽입
# - 작업 완료 처리
# ============================================================

import rclpy
from rclpy.node import Node
from assembly_interfaces.msg import DetectedObject
from dynamic_assembly_robot_control.robot_init import RobotInit
from .motion_utils import MotionUtils
from std_msgs.msg import Float64MultiArray

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

class AssemblyController(Node):

    def __init__(self):
        super().__init__('assembly_controller',namespace=ROBOT_ID)
        #self.node = node
        # 1. 최신 비전 좌표를 저장할 변수
        self.latest_x = 0.0
        self.latest_y = 0.0
        self.latest_z = 0.0
        self.type=""
        self.shape=""
        self.is_object_detected = False
        self.robot_init = RobotInit(self)
        self.mu = MotionUtils(self.robot_init)


        self.subscription = self.create_subscription(
            DetectedObject,
            '/vision/detected_object',
            self.listener_callback,
            10
        )
        self.get_logger().info('DetectedObject 구독 시작')

        self.robot_pose_pub = self.create_publisher(Float64MultiArray, '/robot/current_pose', 10)
        self.robot_pose_timer = self.create_timer(0.1, self.publish_robot_pose)

    def publish_robot_pose(self):
        robot_pose = self.robot_init.get_current_pose()

        if robot_pose is None:
            return

        msg = Float64MultiArray()
        msg.data = robot_pose

        self.robot_pose_pub.publish(msg)
        
    def listener_callback(self, msg: DetectedObject):
        """데이터가 들어올 때 호출되는 콜백 함수"""
        self.latest_x = msg.x
        self.latest_y = msg.y
        self.latest_z = msg.z
        self.type = msg.type
        self.shape = msg.shape
        self.is_object_detected = True
        self.get_logger().info(f'Detected Object - X: {self.latest_x}, Y: {self.latest_y}, Z: {self.latest_z}')

    def test_run(self,is_object_detected,x,y,z):
        self.mu.pick_up([x,y,z,100.08, 179.98, 100.9])

        # if is_object_detected:
        #     self.get_logger().info(f'Detected Object - X: {self.latest_x}, Y: {self.latest_y}, Z: {self.latest_z}')
        #     self.mu.pick_up([x,y,z,100.08, 179.98, 100.9])
        # else:
        #     self.get_logger().info('오브젝트를 감지하지 못했다...')


def main(args=None):
    rclpy.init(args=args)
    import DR_init
        

    node = AssemblyController()
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = node  
    # node.test_run(node.is_object_detected,node.latest_x,node.latest_y,node.latest_z,node.type,node.shape)
    node.test_run(True, 367, 6, 215)  # 테스트용 임시 값
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":

    main()