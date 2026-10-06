"""
Bongshin BS-105 인디케이터 (CBFS-200k 로드셀 연결) - RS-485 실시간 반력(하중) 플랏

실제 하드웨어로 확인한 프레임 포맷 (연속출력/auto-print 모드):
  인디케이터가 별도 요청 없이 계속 아래 포맷으로 스트리밍함:
      STX(0x02) + [ID] + "<sign><space><value>" + ETX(0x03)
      예) ID=0, 표시값 -155.0 -> b'\\x020- 155.0\\x03'

RS-485는 전기적 신호 방식만 다르고 프레임 포맷은 RS-232C와 동일하므로,
USB-RS485 변환기를 통해 일반 시리얼 포트처럼 열어서 그대로 수신하면 된다.
"""

import time
import threading
import serial
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

matplotlib.use("TkAgg")

# ====== 사용자가 직접 맞춰야 할 변수들 ======
PORT = "/dev/tty.usbserial-A17QA5CA"    # 로드셀 인디케이터가 연결된 포트 (실측 확인됨)
BAUD_RATE = 9600                        # 실측 확인됨
DEVICE_ID = 0                            # 참고용 (현재 연속출력 모드라 요청에는 사용되지 않음)
REFRESH_INTERVAL_MS = 100               # 그래프 갱신 주기 (ms)
WINDOW_SEC = 20                         # 그래프에 보여줄 최근 구간 (초)

STX = 0x02
ETX = 0x03


class BS105Indicator:
    def __init__(self, port=PORT, baudrate=BAUD_RATE, device_id=DEVICE_ID, timeout=0.2):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
        )
        self.device_id = device_id
        self.id_byte = 0x30 + device_id  # ID를 ASCII 숫자로 인코딩
        self.lock = threading.Lock()
        self.tare_offset = 0.0

    def close(self):
        self.ser.close()

    def _read_raw(self):
        """버퍼에 쌓인 프레임을 모두 비우고 가장 최신 프레임만 사용 (지연 누적 방지). 실패 시 None."""
        with self.lock:
            waiting = self.ser.in_waiting
            data = self.ser.read(waiting) if waiting else self.ser.read_until(bytes([ETX]))

        end = data.rfind(bytes([ETX]))
        if end == -1:
            return None
        start = data.rfind(bytes([STX]), 0, end)
        if start == -1:
            return None

        body = data[start + 1:end]  # ID + "<sign><space><value>"
        if len(body) < 2:
            return None

        value_str = body[1:].decode("ascii", errors="ignore").replace(" ", "")
        try:
            return float(value_str)
        except ValueError:
            return None

    def read_weight(self):
        """tare_offset이 반영된 보정값 반환. 실패 시 None."""
        raw = self._read_raw()
        return None if raw is None else raw - self.tare_offset

    def tare(self, samples=10):
        """현재 raw 값 평균을 0점 기준으로 저장 (소프트웨어 영점)."""
        vals = []
        while len(vals) < samples:
            raw = self._read_raw()
            if raw is not None:
                vals.append(raw)
        self.tare_offset = sum(vals) / len(vals)
        return self.tare_offset


def plot_realtime(indicator, window_sec=WINDOW_SEC, interval_ms=REFRESH_INTERVAL_MS):
    x_data, y_data = [], []
    start_time = time.time()

    fig, ax = plt.subplots()
    line, = ax.plot([], [], "b-")
    ax.set_xlabel("Time (sec)")
    ax.set_ylabel("Force / Weight")
    ax.set_title("BS-105 Real-time Reading (RS-485)")
    ax.grid(True)

    def update(frame):
        value = indicator.read_weight()
        current_time = time.time() - start_time

        if value is not None:
            x_data.append(current_time)
            y_data.append(value)

        if current_time > window_sec:
            ax.set_xlim(current_time - window_sec, current_time)
        else:
            ax.set_xlim(0, window_sec)

        if y_data:
            line.set_data(x_data, y_data)
            ax.set_ylim(min(y_data) - 1, max(y_data) + 1)
        return (line,)

    ani = FuncAnimation(fig, update, interval=interval_ms, blit=False, save_count=100)
    plt.show()
    return ani


if __name__ == "__main__":
    indicator = BS105Indicator()
    print(f"[연결] {PORT} @ {BAUD_RATE}bps, ID={DEVICE_ID}")

    test_value = indicator.read_weight()
    if test_value is None:
        print("[오류] 인디케이터 응답 없음. PORT/BAUD_RATE/DEVICE_ID 를 확인하세요.")
    else:
        print(f"[확인] 현재 값(영점 전): {test_value}")

    offset = indicator.tare()
    print(f"[영점] 소프트웨어 tare 적용, offset={offset:.2f} -> 현재 값: {indicator.read_weight():.2f}")

    try:
        plot_realtime(indicator)
    finally:
        indicator.close()
        print("종료")
