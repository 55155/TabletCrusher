#!/usr/bin/env python3
"""
IK 기반 task-space 이동 테스트 (펜던트 없이 ROS2에서 직접 제어).

사전 조건:
  - 터미널 A에서 real-mode bringup이 이미 실행 중이어야 함:
      ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
        mode:=real host:=192.168.137.100 port:=12345 model:=m1013
  - 로봇이 Auto 모드 + Servo On 상태

동작:
  1. 현재 task-space 위치(x,y,z,rx,ry,rz)를 읽어서 출력
  2. 그 자리에서 Z축으로만 +30mm 들어올림 (movel)
  3. 2초 대기 후 원래 위치로 복귀

속도/가속도를 일부러 낮게(10, 10) 잡았으니 첫 실행은 반드시 비상정지 버튼에
손을 올려둔 채로 실행할 것.
"""
import rclpy
from rclpy.logging import get_logger

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m1013"

import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

logger = get_logger("test_movel_safe")


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("test_movel_safe", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import (
        movel, posx, get_current_posx,
        set_velx, set_accx, set_robot_mode,
        ROBOT_MODE_AUTONOMOUS,
    )

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    # 이동 속도/가속도를 낮게: 30mm/sec, 30mm/sec^2
    set_velx(30, 30)
    set_accx(30, 30)

    current, sol = get_current_posx()
    if current is None:
        logger.info("get_current_posx 실패 - bringup이 real mode로 떠 있는지 확인하세요")
        rclpy.shutdown()
        return

    logger.info(f"현재 위치: x={current[0]:.1f} y={current[1]:.1f} z={current[2]:.1f} "
                f"rx={current[3]:.1f} ry={current[4]:.1f} rz={current[5]:.1f} (sol={sol})")

    up_target = posx(current[0], current[1], current[2] + 30.0,
                      current[3], current[4], current[5])

    logger.info("Z축으로 +30mm 이동 시도...")
    movel(up_target, vel=30, acc=30)

    logger.info("2초 대기...")
    rclpy.spin_once(node, timeout_sec=2.0)

    logger.info("원래 위치로 복귀...")
    movel(current, vel=30, acc=30)

    logger.info("완료")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
