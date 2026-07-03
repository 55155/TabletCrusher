"""Keyborad_control_v3 — 단일방향 타격 + 각도기반 후퇴 FSM (실HW)

v2(대칭 stall-반전, Crusher.md §12-1)의 문제점(§12-2: 주기 비제어, CW/CCW 토크
비대칭)을 §12-3 알고리즘으로 교체한 버전.

    STRIKE(CCW, 강방향):  정제 타격. 짧은 윈도우 stall = 접촉 → RETRACT.
    RETRACT(CW, 약방향, 무부하):  후퇴각 Δ_retract 도달(1차) 또는
                                  T_RETRACT 만료(안전) → STRIKE.

핵심(§12-3): 부하 극복(타격)은 항상 강한 CCW가 전담하고, 약한 CW는 무부하 후퇴에만
쓰므로 모터 토크 비대칭이 알고리즘에 영향을 주지 않는다.

실HW 제약(v2 레지스터 맵 기준):
  - 전류·엔코더 레지스터가 없다 → 접촉은 stall로만 판정(§12-4 전류 임계 미구현).
  - 크랭크 각도는 RPM(0x0015)을 감속비로 나눠 적분 추정한다(N_f·Δ_retract용).
방향 규약(v2 connect(): set_cw_ccw(0)=CW): CCW=1(타격/전진), CW=0(후퇴). §1
"""

from pymodbus.client import ModbusSerialClient
import time
import threading
from queue import Queue
from collections import deque
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


# ── FSM 파라미터 (Crusher.md §12-5) ──────────────────────────────────────────
DIR_CCW      = 1      # set_cw_ccw(1) → CCW → 슬라이더 전진(정제 타격) §1
DIR_CW       = 0      # set_cw_ccw(0) → CW  → 후퇴
STRIKE_SPEED = 100    # CCW 타격 속도 [모터 명령 단위] — v2 기본값
RETRACT_SPEED= 100    # CW 후퇴 속도  [모터 명령 단위] — 무부하라 자유(§12-5)
GEAR_RATIO   = 212    # 모터→크랭크 감속비 (§1 GEAR 1:212) — 크랭크각 적분 추정용
STALL_SAMPLES= 5      # 접촉 판정 짧은 윈도우 (§12-4)
STRIKE_GRACE_S = 0.6  # STRIKE 진입 직후 스핀업/방향전환 구간 stall 무시
D_BREAK_DEG  = 8.0    # 파쇄 판정 진전각 (§12-5, 5~10°)
RETRACT_TURNS= 0.5    # 후퇴 회전수 — 크랭크 "반바퀴"(TDC→BDC 완전 개방)
D_RETRACT_DEG= RETRACT_TURNS * 360.0  # =180° 크랭크. 감속비는 crank_deg 적분(÷GEAR)에 반영
T_RETRACT_MAX= 8.0    # 후퇴 안전 타임아웃 [s]: 반바퀴 @크랭크8RPM≈3.75s → 여유2× (§12-6 안전용)
T_STRIKE_MAX = 5.0    # 타격 안전 타임아웃 [s] (§12-5)


def _drive(motor, direction, speed):
    """방향 전환 안전 시퀀스 (v2 반전 관행): enable off → dir/speed → enable on."""
    motor.set_enable(0)
    motor.set_cw_ccw(direction)
    motor.set_speed(speed)
    motor.set_enable(1)


def motor_control(motor, run_event, input_queue, timeout=0.05):
    """Crusher.md §12-3 단일방향 타격 + 각도기반 후퇴 FSM.

    §12-1 대칭 stall-반전을 대체. §12-7 의사코드를 실HW에 이식하되, 후퇴 종료는
    §12-6 권고대로 후퇴각 Δ_retract 도달을 1차 기준, 시간을 안전 타임아웃으로 둔다.
    """
    strike_speed = STRIKE_SPEED
    rpm_buffer   = deque(maxlen=STALL_SAMPLES)

    state   = "STRIKE"
    t0      = time.time()
    t_state = t0
    t_prev  = t0

    crank_deg        = 0.0    # 적분 추정 크랭크각 [deg]
    contact_deg      = None   # 직전 STRIKE 접촉각
    retract_start_deg= 0.0    # RETRACT 진입 시 크랭크각(후퇴 진행량 기준)
    n_f              = 0      # 파쇄(fracture) 카운트 (§12-4)

    _drive(motor, DIR_CCW, strike_speed)   # STRIKE 진입: CCW 강방향
    print("[FSM] STRIKE 시작 (CCW)")

    while run_event.is_set():
        # 사용자 속도 오버라이드(선택): 타격 속도 갱신
        if not input_queue.empty():
            strike_speed = input_queue.get()
            if state == "STRIKE":
                motor.set_speed(strike_speed)

        now = time.time()
        dt  = now - t_prev
        t_prev = now

        rpm = motor.get_current_RPM()
        rpm_val = rpm[0] if rpm else 0
        rpm_buffer.append(rpm_val)

        # 크랭크각 적분: STRIKE(CCW,+) / RETRACT(CW,−). rpm→deg/s = rpm*6, ÷감속비.
        dir_sign = 1.0 if state == "STRIKE" else -1.0
        crank_deg += dir_sign * (rpm_val / GEAR_RATIO) * 6.0 * dt

        if state == "STRIKE":
            # 파쇄: 직전 접촉각 + Δ_break 를 stall 없이 통과 → 균열 진전(§12-4)
            if contact_deg is not None and rpm_val > 0 \
                    and crank_deg > contact_deg + D_BREAK_DEG:
                n_f += 1
                print(f"[FRACTURE] N_f={n_f}  crank≈{crank_deg:6.1f}°  t={now-t0:5.1f}s")
                contact_deg = None            # 이번 stroke 재계수 방지

            # 접촉: 짧은 윈도우 stall (grace 이후) → RETRACT (§12-4)
            stalled = len(rpm_buffer) == STALL_SAMPLES and sum(rpm_buffer) == 0
            if stalled and (now - t_state) > STRIKE_GRACE_S:
                contact_deg = crank_deg
                print(f"[CONTACT] crank≈{crank_deg:6.1f}°  → RETRACT")
                state, t_state = "RETRACT", now
                retract_start_deg = crank_deg
                _drive(motor, DIR_CW, RETRACT_SPEED)
                rpm_buffer.clear()
            elif (now - t_state) > T_STRIKE_MAX:      # 안전: 무접촉(정제 소진/이탈)
                print("[WARN] STRIKE 타임아웃(무접촉) → RETRACT")
                state, t_state = "RETRACT", now
                retract_start_deg = crank_deg
                contact_deg = None
                _drive(motor, DIR_CW, RETRACT_SPEED)
                rpm_buffer.clear()

        else:  # RETRACT — 무부하 후퇴, stall 판정 안 함 (§12-3)
            progress = retract_start_deg - crank_deg   # 후퇴 진행량(양수)
            if progress >= D_RETRACT_DEG:              # 1차: 후퇴 회전수 도달 (§12-6)
                print(f"[RETRACT done] Δ={progress:5.1f}° ({progress/360:.2f}rev) → STRIKE")
                state, t_state = "STRIKE", now
                _drive(motor, DIR_CCW, strike_speed)
                rpm_buffer.clear()
            elif (now - t_state) >= T_RETRACT_MAX:     # 안전: 시간 타임아웃 (§12-6)
                print(f"[RETRACT timeout] Δ={progress:5.1f}° ({progress/360:.2f}rev, 안전) → STRIKE")
                state, t_state = "STRIKE", now
                _drive(motor, DIR_CCW, strike_speed)
                rpm_buffer.clear()

        time.sleep(timeout)   # 제어 신호 주기


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
    input_queue = Queue()

    motor_thread = threading.Thread(
        target=motor_control, args=(motor, run_event, input_queue, global_timeout[0])
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
