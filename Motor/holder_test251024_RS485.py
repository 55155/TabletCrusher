from pymodbus.client import ModbusSerialClient as ModbusClient
import time

# 설정값
PORT = '/dev/ttyUSB0'   # 환경에 맞게 수정
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



if not client.connect():
    print("모터 드라이버에 연결 실패")
    exit(1)

print("연결 성공")

device_id = 1
client.write_register(address=0x0020, value=device_id, )
print("device id : ", device_id)

# 쓰기 (0x06): 결과는 요청 에코
import time

rq = client.write_register(0x0023, value = 0x0000, )
print(rq)  # 응답 객체. 정상/에러 여부 등 확인e

rq = client.write_register(address=0x0024, value = 0x0064, )
print(rq)

