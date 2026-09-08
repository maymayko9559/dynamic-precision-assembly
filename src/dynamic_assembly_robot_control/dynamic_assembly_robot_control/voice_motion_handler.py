import rclpy
from assembly_interfaces.srv import VoiceCommand



class VoiceMotionHandler:
    def __init__(self, node):
        self.node = node
        self.keyword = None

        self.cli = node.create_client(VoiceCommand, "/voice_command")
    


    def handle_voice_motion(self, res):
        if res is not None and res.success and res.shape:
            self.keyword = res.shape
            self.node.get_logger().info(f"{res.shape} 음성 인식 성공")

        return res
    
    def request_shape(self, timeout_sec=60.0):
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error("voice_command 서비스를 찾을 수 없음")
            return None

        self.node.get_logger().info("음성 대기 중... ('헬로 로키' 후 도형 말하기)")

        future = self.cli.call_async(VoiceCommand.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)

        return self.handle_voice_motion(future.result())

        
    def run(self):
        self.request_shape()
        self.node.get_logger().info(f"보이스 모션 핸들러 실행 완료, keyword: {self.keyword}")
        
def main():
    rclpy.init()
    node = rclpy.create_node("voice_motion_handler")
    voice_handler = VoiceMotionHandler(node)
    
    
    voice_handler.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


    
        
