"""BLC-400R4E + PG42BL4279 (엔코더 1000CPR, 감속비 1/212) 제어 모듈

PC + USB-RS485 컨버터로 누리로봇 MC-RS485 프로토콜을 쓴다.
(판매처 아두이노 예제와 프레임·체크섬이 바이트 단위로 동일하다)

--------------------------------------------------------------------------
검증된 설정 — 이 값으로 90도 왕복 10회 오차 ±0.08도, 누적 오차 없음
--------------------------------------------------------------------------
  위치·속도 제어기 : Kp 10 / Ki 1 / Kd 0 / 정격전류 1800mA
  분해능           : 1000
  감속비           : 2120 (212.0 : 1)
  위치제어 모드     : 절대
  제어 방향        : CW
  통신             : 9600bps / 8 Data / No Parity / 1 Stop, ID 0x00

  출고값 Kp200 은 이 모터에 과해서 제자리 진동이 났다. 판매처(모터뱅크)가
  지정한 Kp10 으로 해결됐다 — "모터 성능이 높을수록 kp가 높아지면 불안정".

--------------------------------------------------------------------------
반드시 알아야 할 동작 특성 (실측으로 확인)
--------------------------------------------------------------------------
1. **위치 지령을 보내는 순간 드라이버가 좌표를 0 으로 리셋한다.**
   설정은 "절대 위치제어"지만 동작은 상대다.
   -> 지령값을 그대로 "이동량"으로 준다. 현재 위치를 더하면 안 된다.
      180도 돌리려면 180, 90도면 90.
      (현재 위치를 더해 목표를 만들면 수백 도씩 어긋난다)

2. **0x0F(위치 초기화)를 쓰지 말 것.** 보내면 이후 위치 지령이 전부 무시된다.

3. **전원 투입 = 위치 0.** 전원을 껐다 켜면 피드백이 0.00deg 에서 시작한다.
   설정값은 EEPROM 에 유지된다.

4. **지령 상한 655.33도.** 위치 필드가 2바이트(0.01deg 단위)다.
   모터 자체는 계속 돌 수 있고 좌표만 655.36 에서 0 으로 되감긴다.

5. **원점 저장 레지스터가 없다.** 드라이버에 위치 오프셋을 기록하는 명령이
   프로토콜에 없으므로 누적 각도는 PC 쪽에서 관리해야 한다.

6. **피드백은 0xD1 의 위치 필드(0.01deg)를 쓴다.** 0xD2 의 위치 필드는
   기준이 달라 값이 다르게 나온다.

7. 속도제어로 회전 중 위치 지령을 보내면 **정지 과정 없이 전환**된다.
   크랭크 전진(속도제어) / 후퇴(위치제어) 알고리즘이 성립한다.

8. **원점(홈) 복귀는 PC가 추적한 누적 변위를 되돌리는 방식이다.** 드라이버에
   원점 저장 레지스터가 없으므로(위 5), 프로그램을 시작한 바로 이 위치를
   "홈"으로 보고 move() 가 feedback() 의 실측 방향(좌표 증감이 아니라
   CW/CCW 바이트)으로 누적 변위를 갱신한다. 프로그램이 정상 종료·예외·
   Ctrl+C 중 무엇으로 끝나도 close()(with 블록의 __exit__)가 누적 변위를
   역방향으로 상쇄해 홈으로 복귀한 뒤 제어를 끈다.

9. **위치제어(move)·속도제어(run) 를 한 프로세스 안에서 섞어 써도 추적된다.**
   run() 중에는 poll_speed() 를 주기적으로 불러 RPM×경과시간을 적분하고,
   stop_run() 또는 뒤이은 move() 호출 시점에 그 구간이 _home_offset 에
   합산된다(position 필드가 아니라 RPM 적분을 쓰는 이유는 아래 10번). 이
   추적은 프로세스(= 객체) 생명주기 안에서만 유효하다 — CLI 의
   speed/stop 처럼 서로 다른 프로세스 실행으로 나누면 추적이 끊긴다
   (디바이스에도 저장 수단이 없음, 위 5).

10. **run() 중 position 필드는 믿지 말 것(실측 확인).** 연속 회전 중에는
   0xD1 의 위치 필드가 간헐적으로 수십 도씩 튀는 값을 보여줄 때가 있다
   (0.1초 간격 polling 중 52도→2.5도로 점프하는 사례 실측). move() 로
   정지 상태에서 읽는 position(위 1, 6)은 이 문제가 없다. 그래서 위 9의
   회전량 추적은 RPM 적분을 쓴다.

--------------------------------------------------------------------------
미확정 — 실사용 전 확인할 것
--------------------------------------------------------------------------
  · 좌표 증감과 CW/CCW 의 대응 (관측이 엇갈렸다)
  · 좌표 1도와 출력축 1도의 배율이 1:1 인지

--------------------------------------------------------------------------
안전
--------------------------------------------------------------------------
  정격전류는 기어박스가 정한다. PG42(1/212) 연속 2.45Nm / 순시 7.36Nm.
  토크상수 0.0385Nm/A x 212 x 효율 0.5 로 환산하면
      1.8A = 약 7.3Nm (순시 상한)   0.6A = 약 2.4Nm (연속 정격)
  공장 초기화 시 25.4A 로 돌아가는데, 그대로 두면 이론상 100Nm 이 넘어
  기어가 먼저 깨진다. 초기화 후에는 반드시 다시 낮출 것.

사용법
  python BLDC_Encoder_control.py --port COM5 setup       설정 1회 등록
  python BLDC_Encoder_control.py --port COM5 status      현재 설정·위치
  python BLDC_Encoder_control.py --port COM5 move 90     90도 이동
  python BLDC_Encoder_control.py --port COM5 move 90 --ccw
  python BLDC_Encoder_control.py --port COM5 speed 300   30.0RPM 회전
  python BLDC_Encoder_control.py --port COM5 stop
  python BLDC_Encoder_control.py --port COM5 watch 5     5초간 피드백 출력

  프로그램이 어떤 식으로 끝나든(정상/예외/Ctrl+C) 종료 직전에 시작 위치
  ("홈")로 자동 복귀한 뒤 제어를 끈다. 홈 기준은 이 프로그램을 시작한
  시점의 위치이며, 전원을 끄지 않는 한 그 위치가 계속 홈으로 유지된다.
"""

import argparse
import time

import serial

# ---- 프로토콜 상수 --------------------------------------------------------
HEADER = b"\xFF\xFE"
CW, CCW = 0x01, 0x00

M_POS_SPEED   = 0x01   # 위치·속도제어
M_POS         = 0x02   # 가감속 위치제어
M_SPEED       = 0x03   # 가감속 속도제어
M_POS_GAIN    = 0x04   # 위치제어기 설정
M_SPD_GAIN    = 0x05   # 속도제어기 설정
M_RATED_SPEED = 0x09   # 모터 정격속도
M_RESOLUTION  = 0x0A   # 분해능
M_GEAR        = 0x0B   # 감속비
M_ONOFF       = 0x0C   # 제어 On/Off
M_POS_MODE    = 0x0D   # 위치제어 모드 (0=절대, 1=상대)
M_DIRECTION   = 0x0E   # 제어 방향 (0=CCW, 1=CW)
# M_POS_ZERO  = 0x0F   # 위치 초기화 — 쓰지 말 것 (이후 위치 지령이 무시됨)

Q_PING, R_PING = 0xA0, 0xD0
Q_POS,  R_POS  = 0xA1, 0xD1
Q_SPD,  R_SPD  = 0xA2, 0xD2
Q_PGAIN, R_PGAIN = 0xA3, 0xD3
Q_SGAIN, R_SGAIN = 0xA4, 0xD4
Q_RATED, R_RATED = 0xA6, 0xD6
Q_RES,  R_RES  = 0xA7, 0xD7
Q_GEAR, R_GEAR = 0xA8, 0xD8
Q_ONOFF, R_ONOFF = 0xA9, 0xD9
Q_PMODE, R_PMODE = 0xAA, 0xDA
Q_DIR,  R_DIR  = 0xAB, 0xDB
Q_FW,   R_FW   = 0xCD, 0xFD

POS_MAX_DEG = 655.33    # 2바이트 x 0.01deg
POS_WRAP    = 655.36

# ---- 검증된 설정값 --------------------------------------------------------
VENDOR_SETTINGS = {
    "resolution": 1000,          # 분해능
    "gear_x10":   2120,          # 감속비 212.0 : 1
    "kp": 10, "ki": 1, "kd": 0,  # 판매처 지정 게인
    "current_100ma": 18,         # 1800mA — 기어박스 순시 7.36Nm 상한
    "direction": CW,
    "absolute": True,
}


def _u16(v):
    return bytes([(v >> 8) & 0xFF, v & 0xFF])


class BLC400R4E:
    def __init__(self, port, baud=9600, dev_id=0, timeout=0.05, verbose=False):
        self.id = dev_id
        self.verbose = verbose
        self.ser = serial.Serial(port, baud, bytesize=8, parity="N",
                                 stopbits=1, timeout=timeout)
        self._home_offset = 0.0   # 홈(시작 위치) 기준 누적 변위 [deg]. CW(+)/CCW(-)
        self._speed_last_t = None # 속도제어 추적 중 마지막 poll 시각(monotonic). None=추적 안 함
        self._speed_last_rpm = 0.0 # 같은 추적의 마지막 RPM 샘플(사다리꼴 적분용)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False

    # -- 프레임 ------------------------------------------------------------
    def frame(self, mode, data=b""):
        body = bytes([mode]) + data
        size = len(body) + 1                      # Mode + Data + Checksum
        chk = (~(self.id + size + sum(body))) & 0xFF
        return HEADER + bytes([self.id, size, chk]) + body

    def _send(self, mode, data=b"", deadline=0.4):
        """송신 후 프레임이 다 찰 때까지 기다린다. 컨버터 에코는 걸러낸다."""
        pkt = self.frame(mode, data)
        self.ser.reset_input_buffer()
        self.ser.write(pkt)
        self.ser.flush()
        buf = bytearray()
        t0 = time.monotonic()
        while time.monotonic() - t0 < deadline:
            n = self.ser.in_waiting
            if n:
                buf += self.ser.read(n)
            b = bytes(buf)
            if b.startswith(pkt):
                b = b[len(pkt):]
            if len(b) >= 4 and b[0] == 0xFF and len(b) >= 4 + b[3]:
                break
            time.sleep(0.001)
        if self.verbose:
            print(f"TX: {pkt.hex(' ').upper()}")
            print(f"RX: {bytes(buf).hex(' ').upper() or '(없음)'}")
        return pkt, bytes(buf)

    def query(self, req, resp, tries=3):
        """요청을 보내고 기대한 Mode 의 응답 payload 를 돌려준다. 실패하면 None.

        Ping 처럼 payload 가 없는 응답은 빈 bytes 를 돌려주므로
        `is not None` 으로 판정할 것.
        """
        for _ in range(tries):
            pkt, raw = self._send(req)
            b = raw[len(pkt):] if raw.startswith(pkt) else raw
            if len(b) < 6 or b[0] != 0xFF or b[1] != 0xFE or len(b) < 4 + b[3]:
                continue
            f = b[:4 + b[3]]
            if f[5] != resp:
                continue
            if ((~(f[2] + f[3] + sum(f[5:]))) & 0xFF) != f[4]:
                continue
            return f[6:]
        return None

    def command(self, mode, data=b"", settle=0.3):
        self._send(mode, data)
        time.sleep(settle)

    # -- 상태 --------------------------------------------------------------
    def alive(self):
        return self.query(Q_PING, R_PING) is not None

    def position(self):
        """현재 위치 [deg]. 0xD1 기준."""
        d = self.query(Q_POS, R_POS)
        return None if d is None else ((d[1] << 8) | d[2]) / 100.0

    def speed(self):
        """현재 속도 [RPM]. 0xD1 기준."""
        d = self.query(Q_POS, R_POS)
        return None if d is None else ((d[3] << 8) | d[4]) / 10.0

    def feedback(self):
        """(방향, 위치[deg], 속도[RPM], 전류[A]) — 0xD1."""
        d = self.query(Q_POS, R_POS)
        if d is None:
            return None
        return ("CW" if d[0] else "CCW",
                ((d[1] << 8) | d[2]) / 100.0,
                ((d[3] << 8) | d[4]) / 10.0,
                d[5] * 0.1)

    def settings(self):
        out = {}
        for key, req, resp, fn in (
            ("분해능",      Q_RES,   R_RES,   lambda d: (d[0] << 8) | d[1]),
            ("감속비",      Q_GEAR,  R_GEAR,  lambda d: ((d[0] << 8) | d[1]) / 10.0),
            ("정격속도",    Q_RATED, R_RATED, lambda d: (d[0] << 8) | d[1]),
            ("위치제어기",  Q_PGAIN, R_PGAIN, lambda d: (d[0], d[1], d[2], d[3] * 100)),
            ("속도제어기",  Q_SGAIN, R_SGAIN, lambda d: (d[0], d[1], d[2], d[3] * 100)),
            ("제어방향",    Q_DIR,   R_DIR,   lambda d: "CW" if d[0] else "CCW"),
            ("위치모드",    Q_PMODE, R_PMODE, lambda d: "상대" if d[0] else "절대"),
            ("제어On/Off",  Q_ONOFF, R_ONOFF, lambda d: "Off" if d[0] else "On"),
            ("펌웨어",      Q_FW,    R_FW,    lambda d: d[0]),
        ):
            d = self.query(req, resp)
            out[key] = fn(d) if d else None
        return out

    # -- 제어 --------------------------------------------------------------
    def enable(self):
        self.command(M_ONOFF, bytes([0x00]))

    def disable(self):
        """제어 Off. 이 드라이버는 감속 정지 지령이 듣지 않으므로
        확실한 정지 수단은 이것뿐이다."""
        self.command(M_ONOFF, bytes([0x01]))

    def apply_settings(self, cfg=None):
        """설정 등록. 반드시 제어 Off 상태에서, 명령 간 0.3초 이상 간격.
        EEPROM 에 기록되므로 최초 1회만 하면 된다."""
        cfg = cfg or VENDOR_SETTINGS
        self.disable()
        self.command(M_POS_MODE, bytes([0x00 if cfg["absolute"] else 0x01]), settle=0.45)
        self.command(M_RESOLUTION, _u16(cfg["resolution"]), settle=0.45)
        self.command(M_GEAR, _u16(cfg["gear_x10"]), settle=0.45)
        gains = bytes([cfg["kp"], cfg["ki"], cfg["kd"], cfg["current_100ma"]])
        self.command(M_POS_GAIN, gains, settle=0.45)
        self.command(M_SPD_GAIN, gains, settle=0.45)
        self.command(M_DIRECTION, bytes([cfg["direction"]]), settle=0.45)

    def move(self, degrees, direction=CW, reach_s=1.0):
        """지정한 각도만큼 이동.

        위치 지령 순간 드라이버가 좌표를 0 으로 리셋하므로, 여기 넣는 값이
        그대로 이동량이 된다. 현재 위치를 더하지 않는다.

        직전에 run() 으로 속도제어 중이었다면(§7: 정지 과정 없이 전환),
        명령을 보내기 전에 그 구간 회전량을 먼저 _home_offset 에 반영한다
        — stop_run() 을 안 불러도 전환 시점에 자동으로 마무리된다.
        """
        if self._speed_last_t is not None:
            self.poll_speed()
            self._speed_last_t = None
        if not 0 < degrees <= POS_MAX_DEG:
            raise ValueError(f"이동량은 0 초과 {POS_MAX_DEG}도 이하여야 합니다")
        t01 = max(1, min(255, int(round(reach_s * 10))))
        self.command(M_POS,
                     bytes([direction]) + _u16(int(round(degrees * 100))) + bytes([t01]),
                     settle=0.02)

    def run(self, rpm_x10, direction=CW, reach_s=1.0):
        """속도제어로 연속 회전 시작.

        정지는 disable() 이 아니라 stop_run() 으로 할 것 — disable() 은
        회전량을 _home_offset 에 반영하지 않고 바로 끈다. 회전 중 주기적으로
        poll_speed() 를 불러줘야 추적이 된다(아래 참고).

        추적 시작 시각은 명령 전송 *뒤*에 찍는다 — 전송 전에 찍으면
        _send() 의 응답 대기(최대 0.4초, 이 명령은 응답이 없어 항상 다
        채운다)까지 "이미 회전 중"으로 적분해 과대 추정하게 된다(실측
        확인: 1.4초짜리 회전을 ~66도로, 실제 ~52도보다 크게 잘못 계산).
        """
        t01 = max(1, min(255, int(round(reach_s * 10))))
        self.command(M_SPEED,
                     bytes([direction]) + _u16(int(rpm_x10)) + bytes([t01]),
                     settle=0.02)
        self._speed_last_t = time.monotonic()
        self._speed_last_rpm = 0.0   # 정지 상태에서 시작한다고 가정(사다리꼴 적분용)

    def poll_speed(self):
        """속도제어(run()) 중 회전량을 누적한다.

        position 필드가 아니라 RPM×경과시간 적분(사다리꼴: 이전·현재 RPM
        평균)을 쓴다 — 실측해보니 연속 회전 중 position 필드가 간헐적으로
        튀는 값을 보여(수십 도가 한 틱 사이 점프), 그대로 적분하면 틀린
        값이 쌓인다. RPM은 같은 구간에서 안정적이었고, position 필드가
        멈춰서(move()) 보여주는 값과도 6×RPM×dt 가 잘 맞는다(실측 확인).
        run() 을 부르지 않은 상태(추적 비활성)면 아무 것도 안 하고 None.
        """
        if self._speed_last_t is None:
            return None
        now = time.monotonic()
        dt = now - self._speed_last_t
        self._speed_last_t = now
        fb = self.feedback()
        if fb is None:
            return None
        direction, _, rpm, _ = fb
        avg_rpm = (self._speed_last_rpm + rpm) / 2.0
        self._speed_last_rpm = rpm
        delta = avg_rpm * 6.0 * dt   # RPM -> deg/s
        self._home_offset += delta if direction == "CW" else -delta
        return fb

    def stop_run(self):
        """속도제어 종료: 남은 회전량을 한 번 더 반영한 뒤 제어를 끈다."""
        self.poll_speed()
        self._speed_last_t = None
        self.disable()

    def wait_stop(self, timeout=10.0, poll=0.04, track=True):
        """이동이 끝날 때까지 기다린다. 도달 위치를 돌려준다.

        track=True 면 정지 후 feedback() 의 실측 방향으로 홈 기준 누적
        변위(self._home_offset)를 갱신한다. 위치 지령 1회(move())의 결과를
        반영하는 용도이므로, 연속 속도제어(run()) 뒤에는 track=False 로
        호출할 것 — 끝나지 않은 회전을 "도달 변위"로 잘못 누적하게 된다.

        추적(finally)은 대기 루프가 어떻게 끝나든(정상 완료 / Ctrl+C) 실행된다
        — 이동 "중" 끊겨도 그 순간의 실측 위치를 홈 복귀 계산에 반영하기 위함.
        """
        t0 = time.monotonic()
        moved = False
        try:
            while time.monotonic() - t0 < timeout:
                fb = self.feedback()
                if fb:
                    if fb[2] > 0.5:
                        moved = True
                    elif moved:
                        break
                time.sleep(poll)
            time.sleep(0.4)
        finally:
            if track:
                fb = self.feedback()
                if fb is not None:
                    direction, deg = fb[0], fb[1]
                    self._home_offset += deg if direction == "CW" else -deg
        return self.position()

    def home(self, tolerance_deg=0.15, max_iter=20):
        """홈(프로그램 시작 위치)으로 복귀. 누적 변위가 없으면 아무 것도 안 한다.

        원점 저장 레지스터가 없으므로(docstring §5) PC가 추적한
        self._home_offset 을 역방향으로 상쇄하는 방식이다. 지령 상한
        655.33도(§4)를 넘는 변위는 여러 번에 나눠 이동한다.

        위치 유지 중에도 RPM이 완전히 0으로 안 떨어지는 경우가 있어
        wait_stop() 의 RPM 임계치 감지를 쓰면 매번 타임아웃을 다 채운다.
        대신 move() 에 넘긴 도달시간만큼만 고정 대기한다(검증된 90도/1.0초
        기준과 동일한 비율, §검증된 설정값).
        """
        if abs(self._home_offset) <= tolerance_deg:
            return None
        if not self.alive():
            return None
        for _ in range(max_iter):
            if abs(self._home_offset) <= tolerance_deg:
                break
            step = min(abs(self._home_offset), POS_MAX_DEG - 1.0)
            direction = CCW if self._home_offset > 0 else CW
            reach = max(1.0, step / 90.0)
            self.enable()
            self.move(step, direction, reach_s=reach)
            try:
                time.sleep(reach + 0.4)
            finally:
                fb = self.feedback()
                self.disable()
                if fb is not None:
                    fb_dir, deg = fb[0], fb[1]
                    self._home_offset += deg if fb_dir == "CW" else -deg
        return self.position()

    def close(self):
        try:
            self.home()
        finally:
            try:
                self.disable()
            finally:
                self.ser.close()


# ---- CLI -----------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="BLC-400R4E 모터 제어")
    ap.add_argument("--port", default="COM5")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--id", type=int, default=0)
    ap.add_argument("--ccw", action="store_true", help="CCW 방향")
    ap.add_argument("--reach", type=float, default=1.0, help="도달시간 [s]")
    ap.add_argument("-v", "--verbose", action="store_true", help="TX/RX 로그")
    ap.add_argument("cmd", choices=["setup", "status", "move", "speed", "stop", "watch"])
    ap.add_argument("value", nargs="?", type=float)
    a = ap.parse_args()

    direction = CCW if a.ccw else CW
    with BLC400R4E(a.port, a.baud, a.id, verbose=a.verbose) as m:
        if not m.alive():
            print("드라이버 무응답 — 24V 전원과 RS-485 결선을 확인하십시오")
            return

        if a.cmd == "setup":
            print("설정 등록 (제어 Off 상태, 0.45초 간격)")
            m.apply_settings()
            for k, v in m.settings().items():
                print(f"  {k:10s} {v}")

        elif a.cmd == "status":
            for k, v in m.settings().items():
                print(f"  {k:10s} {v}")
            fb = m.feedback()
            if fb:
                print(f"  {'현재':10s} {fb[0]} {fb[1]:.2f}deg  {fb[2]:.1f}RPM  {fb[3]:.1f}A")

        elif a.cmd == "move":
            if a.value is None:
                print("이동할 각도를 입력하십시오 (예: move 90)"); return
            m.enable()
            before = m.position()
            m.move(a.value, direction, a.reach)
            end = m.wait_stop()
            m.disable()
            print(f"  지령 {a.value:.2f}도 ({'CCW' if a.ccw else 'CW'}) "
                  f"-> 도달 {end:.2f}deg  오차 {end - a.value:+.2f}deg")

        elif a.cmd == "speed":
            if a.value is None:
                print("속도를 0.1RPM 단위로 입력하십시오 (예: speed 300 = 30.0RPM)"); return
            m.enable()
            m.run(a.value, direction, a.reach)
            print(f"  {a.value/10:.1f}RPM 회전 중 — 정지는 stop")

        elif a.cmd == "stop":
            m.disable()
            print("  제어 Off")

        elif a.cmd == "watch":
            secs = a.value or 5.0
            t0 = time.monotonic()
            while time.monotonic() - t0 < secs:
                fb = m.feedback()
                if fb:
                    print(f"  t={time.monotonic()-t0:5.2f}s  {fb[0]:3s} "
                          f"{fb[1]:8.2f}deg  {fb[2]:6.1f}RPM  {fb[3]:4.1f}A")
                time.sleep(0.1)


if __name__ == "__main__":
    main()
