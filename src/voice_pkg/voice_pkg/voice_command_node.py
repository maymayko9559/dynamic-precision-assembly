import time

import rclpy
from rclpy.node import Node

from assembly_interfaces.srv import VoiceCommand

from voice_pkg.STT import STT, openai_api_key
from voice_pkg.wakeup_word import WakeupWord
from voice_pkg.shape_extractor import ShapeExtractor

WAKEWORD_TIMEOUT = 30.0


class VoiceCommandNode(Node):

    def __init__(self):
        super().__init__("voice_command_node")

        self.stt = STT(openai_api_key=openai_api_key)
        self.extractor = ShapeExtractor()
        self.wakeup_word = WakeupWord()

        # 절대 이름(/voice_command)으로 만들어야 controller의
        # dsr01 namespace와 무관하게 매칭된다.
        self.srv = self.create_service(
            VoiceCommand, "/voice_command", self.handle_voice_command
        )
        self.get_logger().info("voice_command 서비스 대기 중...")

    def handle_voice_command(self, request, response):
        response.success = False
        response.command = ""
        response.shape = ""

        # 1) 웨이크워드
        try:
            self.wakeup_word.open()
        except Exception as e:
            self.get_logger().error(f"오디오 스트림 실패: {e}")
            return response

        detected = False
        try:
            t0 = time.monotonic()
            while time.monotonic() - t0 < WAKEWORD_TIMEOUT:
                if self.wakeup_word.is_wakeup():
                    detected = True
                    break
        finally:
            self.wakeup_word.close()

        if not detected:
            self.get_logger().warn("웨이크워드 타임아웃")
            return response

        # 2) STT + 3) 도형 추출 (예외로 노드가 죽지 않게)
        try:
            text = self.stt.speech2text()
            shape, command = self.extractor.extract(text)
        except Exception as e:
            self.get_logger().error(f"STT/추출 실패: {type(e).__name__}: {e}")
            return response

        if shape is None and command != "stop":
            self.get_logger().warn(f"도형 인식 실패: '{text}'")
            return response

        response.success = True
        response.command = command
        response.shape = shape or ""
        self.get_logger().info(f"인식 결과 → shape={response.shape}, command={command}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()