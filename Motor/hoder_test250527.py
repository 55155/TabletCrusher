from pymodbus.client import ModbusSerialClient as ModbusClient
import time

# 설정값
PORT = '/dev/ttyUSB2'   # 환경에 맞게 수정
BAUDRATE = 9600
SLAVE_ID = 1                       # 모터 드라이버 주소 (기본 1)

client = ModbusClient(
    port=PORT,
    baudrate=BAUDRATE,
    parity='N',
    
    stopbits=1,
    bytesize=8,
    timeout=3
)

try:
    if not client.connect():
        print("모터 드라이버에 연결 실패")
        exit(1)
    print("연결 성공")

    # 1. RS-485 통신 모드 기동 (필수)
    result = client.write_register(0x0023, 0x0000, slave=SLAVE_ID)
    print(f"통신모드 기동: {result}")
    time.sleep(0.1)  # 모드 전환 대기

    # 2. 브레이크 해제
    result = client.write_register(0x0026, 0x0000, slave=SLAVE_ID)
    print(f"브레이크 해제: {result}")
    time.sleep(0.05)

    # 3. 방향 설정 (CW: 0x0001, CCW: 0x0002)
    result = client.write_register(0x0025, 0x0001, slave=SLAVE_ID)  # CW
    print(f"방향(CW) 설정: {result}")
    time.sleep(0.05)

    # 4. 속도(PWM) 설정 (예: 40%)
    result = client.write_register(0x0024, 40, slave=SLAVE_ID)
    print(f"PWM 40% 설정: {result}")
    time.sleep(2)

    # 5. 방향 전환 (CCW)
    result = client.write_register(0x0025, 0x0002, slave=SLAVE_ID)  # CCW
    print(f"방향(CCW) 전환: {result}")
    time.sleep(2)

    # 6. 속도 변경 (예: 70%)
    result = client.write_register(0x0024, 70, slave=SLAVE_ID)
    print(f"PWM 70% 설정: {result}")
    time.sleep(2)

    # 7. 브레이크 ON (정지)
    result = client.write_register(0x0026, 0x0001, slave=SLAVE_ID)
    print(f"브레이크 ON: {result}")

except Exception as e:
    print(f"에러 발생: {e}")

finally:
    client.close()
    print("프로그램 종료")
