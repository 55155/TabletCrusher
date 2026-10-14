#!/usr/bin/env python3
"""M1013 데모 궤적: 1번 축 좌우 회전 → 손끝 직선 이동(IK).

기본은 MuJoCo 시뮬(sim01). 실물 로봇(dsr01)은 --real 을 명시해야만 명령하며, 속도를 낮춘다.
  python3 sim_demo.py           # 시뮬 (sim_mujoco.sh 먼저 실행)
  python3 sim_demo.py --real    # 실물 (real_mujoco.sh 먼저 실행, 비상정지 버튼 준비)
"""
import sys

import rclpy

REAL = "--real" in sys.argv
ROBOT_ID = "dsr01" if REAL else "sim01"
ROBOT_MODEL = "m1013"
JVEL = 30 if REAL else 60    # deg/s
LVEL = 100 if REAL else 150  # mm/s

import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main():
    rclpy.init()
    node = rclpy.create_node("demo_traj", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import movej, movel, posj, posx, get_current_posx, get_current_posj

    print(f"대상: /{ROBOT_ID} ({'실물' if REAL else '시뮬'}), 관절 {JVEL}deg/s, 직선 {LVEL}mm/s")
    print(f"시작 관절 각도: {[round(v, 1) for v in get_current_posj()]}")

    print("1) home 자세로 이동 (관절)")
    movej(posj(0, 0, 90, 0, 90, 0), vel=JVEL, acc=JVEL)

    print("2) 1번 축 좌우 회전")
    movej(posj(45, 0, 90, 0, 90, 0), vel=JVEL, acc=JVEL)
    movej(posj(-45, 0, 90, 0, 90, 0), vel=JVEL, acc=JVEL)
    movej(posj(0, 0, 90, 0, 90, 0), vel=JVEL, acc=JVEL)

    current, _ = get_current_posx()
    print(f"3) 현재 손끝 좌표: {[round(v, 1) for v in current]}")

    print("4) 손끝 직선 이동 (IK): 아래로 200mm → 옆으로 200mm → 복귀")
    x, y, z, rx, ry, rz = current
    movel(posx(x, y, z - 200, rx, ry, rz), vel=LVEL, acc=LVEL)
    movel(posx(x, y + 200, z - 200, rx, ry, rz), vel=LVEL, acc=LVEL)
    movel(posx(x, y, z, rx, ry, rz), vel=LVEL, acc=LVEL)

    print("완료")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
