import time

import rclpy

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

HOME = [0, 0, 90, 0, 90, 0]
TARGET = [363.80, -12.77, 396.74, 15.18, 179.83, 15.33]

VEL = [100, 100]
ACC = [200, 300]


def main():
    rclpy.init()
    node = rclpy.create_node("test_pkg", namespace=ROBOT_ID)

    # DSR_ROBOT2 는 import 시점에 이 값들로 서비스 클라이언트를 만든다.
    import DR_init
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import (
        movej,
        servol,
        posj,
        posx,
        set_velj,
        set_accj,
        set_velx,
        set_accx,
    )

    set_velj(30)
    set_accj(60)
    set_velx(50)
    set_accx(100)

    node.get_logger().info("홈으로 이동")
    movej(posj(HOME))

    node.get_logger().info("목표로 이동")
    servol(posx(TARGET), v=VEL, a=ACC)

    # servol 은 비동기라 명령 직후 바로 반환한다.
    time.sleep(3.0)

    node.get_logger().info("이동 완료")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
