"""
기본 운전 테스트: 저속/저전류로 짧게 정회전 후 정지.
리미트 스위치 미사용 상태이므로 run_seconds를 짧게 유지할 것.
"""

import time

from sbd14u_driver import SBD14U

PORT = "/dev/cu.usbserial-A04AC9IK"
RUN_SECONDS = 1.0

if __name__ == "__main__":
    with SBD14U(port=PORT, baudrate=9600, slave_id=1) as motor:
        print("장치 주소:", hex(motor.get_device_address()))

        motor.set_current(run=23, hold=8)   # (24/32)*2A=1.5A 운전(모터 정격 LSM-NK174218 1.5A/phase), (9/32)*2A≈0.56A 구속
        motor.set_init_speed(1500)          # 51200 -> 1500, 탈조 방지 위한 완만한 시작속도
        motor.set_run_speed(3000)           # 3000 pulse/s, 저속

        print("상태(운전 전):", motor.read_status())

        print(f"정회전 시작 ({RUN_SECONDS}초)...")
        motor.jog(forward=True)  # 리미트 미사용
        time.sleep(RUN_SECONDS)

        print("정지...")
        motor.stop(lock=True)

        print("상태(정지 후):", motor.read_status())
