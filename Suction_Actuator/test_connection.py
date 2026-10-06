"""
배선 후 통신 확인용 스모크 테스트.
USB-RS485 어댑터를 연결하고 PORT를 실제 장치 경로로 바꾼 뒤 실행.
(Mac: `ls /dev/tty.*`로 어댑터 포트 확인, 보통 /dev/tty.usbserial-XXXX)
"""

from sbd14u_driver import SBD14U

PORT = "/dev/cu.usbserial-A04AC9IK"

if __name__ == "__main__":
    with SBD14U(port=PORT, baudrate=9600, slave_id=1) as motor:
        addr = motor.get_device_address()
        print(f"장치 주소 읽기 성공: 0x{addr:04X}")

        status = motor.read_status()
        print("상태:", status)
