from pymodbus.client import ModbusSerialClient
import time
import threading
from queue import Queue
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Matplotlib 백엔드 설정
matplotlib.use('TkAgg')
class MotorController:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, device_id=100, timeout = 0.1):
        self.client = ModbusSerialClient(
            port=port,
            baudrate=baudrate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=timeout,
        )
        
        self.device_id = device_id
        self.lock = threading.Lock()

    def connect(self):
        if not self.client.connect():
            return False
        time.sleep(0.5)  # 모터 안정화
        
        # Remote Mode 강제 설정 (PDF 기준)
        self.set_enable(0)      # 안전 정지
        self.set_cw_ccw(0)      # CW
        self.set_speed(100)     # 테스트 속도
        self.set_enable(1)      # 시작!
        
        test_rpm = self.read_register(0x0015, 1)
        print(f"RPM: {test_rpm}")  # 값 출력되면 성공!
        return test_rpm is not None

    def close(self):
        self.set_speed(0)
        self.client.close()

    def read_register(self, address, count=1):
        try:
            with self.lock:
                response = self.client.read_holding_registers(
                    address=address, count=count, slave=self.device_id
                )
            if response and not response.isError():
                return response.registers
            return None
        except:
            return None

    def write_register(self, address, value):
        try:
            with self.lock:
                response = self.client.write_register(
                    address=address, value=value, slave=self.device_id
                )
            if response and not response.isError():
                return True
            return False
        except:
            return False

    def set_speed(self, speed):
        if 0 <= speed <= 300:
            return self.write_register(0x0001, speed)
        return False

    def set_cw_ccw(self, direction):
        if direction in [0, 1]:
            return self.write_register(0x0002, direction)
        return False

    def set_enable(self, enable):
        if enable in [0, 1]:
            return self.write_register(0x0003, enable)
        return False

    def set_brake(self, brake):
        if brake in [0, 1]:
            return self.write_register(0x0004, brake)
        return False

    def get_current_RPM(self):
        return self.read_register(0x0015)


def user_input_handler(motor, run_event, input_queue): 
    # queue 로 신호를 맞출수 있구나.. 이생각을 못했네.. 
    while run_event.is_set():
        try:
            user_input = input()
            if user_input.isdigit(): # 여기서 except 발생
                user_speed = int(user_input)
                if 0 <= user_speed <= 150:
                    input_queue.put(user_speed)
            else:
                run_event.clear() # 
                break
        except:
            run_event.clear() # clear 시에 스레드 종료
            break

from collections import deque

def motor_control(motor, run_event, direction_lock, current_direction, input_queue, timeout = 0.1):
    rpm_buffer = deque(maxlen=5)  # 최근 5개의 RPM 값을 저장
    while run_event.is_set():
        if not input_queue.empty():
            new_speed = input_queue.get()
            motor.set_speed(new_speed)

        current_rpm = motor.get_current_RPM()
        if current_rpm:
            rpm_value = current_rpm[0]
            rpm_buffer.append(rpm_value)  # 최근 RPM 값을 버퍼에 추가

            if sum(rpm_buffer) == 0:  # 최근 값의 평균이 0일 경우
                motor.set_enable(0)
                with direction_lock:
                    current_direction[0] = 1 - current_direction[0]
                    motor.set_cw_ccw(current_direction[0])
                motor.set_enable(1)
                rpm_buffer.clear()  # 방향 전환 후 버퍼 초기화
                time.sleep(0.5)  # 안정화 시간 추가
        time.sleep(timeout) # 모터 컨트롤 신호 주기 

def plot_rpm_and_direction(motor, run_event, run_time=20):
    x_data = []
    rpm_data = []
    direction_data = []
    start_time = time.time()

    # 두 개의 서브플롯 생성
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    line_rpm, = ax1.plot([], [], 'b-', label="RPM")
    line_direction, = ax2.plot([], [], 'r-', label="Direction")

    # 서브플롯 1: RPM
    ax1.set_xlim(0, run_time)
    ax1.set_ylim(-10, 3000)
    ax1.set_xlabel("Time (sec)")
    ax1.set_ylabel("RPM")
    ax1.set_title("Real-time Motor RPM")
    ax1.legend(loc="upper right")

    # 서브플롯 2: Direction
    ax2.set_xlim(0, run_time)
    ax2.set_ylim(-0.5, 1.5)  # 0과 1 사이
    ax2.set_yticks([0, 1])   # y축에 0과 1 라벨
    ax2.set_xlabel("Time (sec)")
    ax2.set_ylabel("Direction")
    ax2.set_title("Real-time Motor Direction")
    ax2.legend(loc="upper right")

    def update(frame):
        if not run_event.is_set():
            plt.close()
            return line_rpm, line_direction

        current_time = time.time() - start_time
        if current_time > run_time:
            ax1.set_xlim(current_time - run_time, current_time)
            ax2.set_xlim(current_time - run_time, current_time)
        else:
            ax1.set_xlim(0, run_time)
            ax2.set_xlim(0, run_time)

        # RPM 데이터 업데이트
        current_register_rpm = motor.get_current_RPM()
        if current_register_rpm:
            rpm_value = current_register_rpm[0]
            x_data.append(current_time)
            rpm_data.append(rpm_value)
            line_rpm.set_data(x_data, rpm_data)

        # Direction 데이터 업데이트
        current_register_direction = motor.read_register(0x0002)  # 0x0002는 방향 레지스터
        if current_register_direction:
            direction_value = current_register_direction[0]
            direction_data.append(direction_value)
            line_direction.set_data(x_data, direction_data)

        return line_rpm, line_direction

    ani = FuncAnimation(fig, update, interval=1000, blit=False, save_count=100)
    try:
        plt.tight_layout()
        plt.show()
    except:
        pass

    run_event.clear()

if __name__ == "__main__":

    global_timeout = [0.03, 0.05, 0.1, 0.2, 0.5, 1.0]
    motor = MotorController(timeout=global_timeout[0])
    if not motor.connect():
        print("[오류] Modbus 연결 실패")
        exit()
    else:
        print("모터 연결 성공")

    if not motor.set_speed(100):
        print("[오류] 속도 설정 실패")
    if not motor.set_brake(0):
        print("[오류] 브레이크 해제 실패")

    run_event = threading.Event()
    run_event.set()
    direction_lock = threading.Lock()
    current_direction = [0]
    input_queue = Queue()

    motor_thread = threading.Thread(
        target=motor_control, args=(motor, run_event, direction_lock, current_direction, input_queue, global_timeout[0])
    )
    input_thread = threading.Thread(
        target=user_input_handler, args=(motor, run_event, input_queue)
    )

    motor_thread.start()
    input_thread.start()

    try:
        # 두 개의 플롯 표시 함수 호출
        plot_rpm_and_direction(motor, run_event, run_time=10)
    except Exception as e:
        print(f"[오류] 플로팅 실행 중 오류 발생: {e}")
        run_event.clear()

    motor_thread.join()
    input_thread.join()
    motor.close()

    print("프로세스 종료")