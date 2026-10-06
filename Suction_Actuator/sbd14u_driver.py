"""
SBD-14U (모터뱅크) 2상 바이폴러 스텝모터 드라이버 - Modbus RTU 제어 드라이버.

프로토콜: Modbus RTU, 9600bps, parity none, stop bit 1 (출하 기본값)
지원 Function code: 0x03(레지스터 읽기), 0x06(단일 쓰기), 0x10(다중 쓰기)
레지스터맵 출처: SBD-14U_protocol.pdf
"""

import struct
import time

import serial

# 레지스터 주소
REG_ADDRESS = 0x0020       # 16bit, R/W, FC03/06 - 모듈 주소
REG_BAUDRATE = 0x0021      # 32bit, R/W, FC03/0x10 - 통신속도
REG_CURRENT = 0x0023       # 16bit, R/W, FC03/06 - 운전전류(AH)/구속전류(AL), 0-31
REG_HOME_SPEED = 0x0024    # 32bit, R/W, FC03/0x10 - 원점복귀 속도 (pulse/s)
REG_INIT_SPEED = 0x0026    # 32bit, R/W, FC03/0x10 - 초기속도 (pulse/s)
REG_RUN_SPEED = 0x0028     # 32bit, R/W, FC03/0x10 - 운전속도 (pulse/s)
REG_DIR_MODE = 0x002A      # 16bit, W, FC06 - 정역회전 모드 운전
REG_COORD = 0x002B         # 32bit, R/W, FC03/0x10 - 좌표모드 운전 (목표 스텝수)
REG_STOP = 0x002D          # 16bit, W, FC06 - 운전정지
REG_STATUS = 0x002E        # 16bit, R, FC03 - 상태
REG_BRAKE = 0x002F         # 16bit, R/W, FC06 - 브레이크/솔레노이드 (구매시 옵션)
REG_MICROSTEP = 0x0030     # 16bit, R/W, FC06 - 분주비

ALLOWED_MICROSTEPS = (4, 8, 16, 32, 64, 125, 256)


def crc16_modbus(data: bytes) -> bytes:
    """표준 Modbus RTU CRC16. 프레임에 붙일 때는 low byte, high byte 순서."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return struct.pack("<H", crc)


class ModbusError(Exception):
    pass


class SBD14U:
    def __init__(self, port: str, baudrate: int = 9600, slave_id: int = 1, timeout: float = 0.5):
        self.slave_id = slave_id
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
        )

    def close(self):
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # --- 저수준 Modbus 프레임 ---

    def _transact(self, pdu: bytes, expected_len: int) -> bytes:
        frame = bytes([self.slave_id]) + pdu
        frame += crc16_modbus(frame)
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        resp = self.ser.read(expected_len)
        if len(resp) < expected_len:
            raise ModbusError(f"응답 길이 부족: {resp.hex()} (기대 {expected_len}바이트)")
        if crc16_modbus(resp[:-2]) != resp[-2:]:
            raise ModbusError(f"CRC 불일치: {resp.hex()}")
        if resp[1] & 0x80:
            raise ModbusError(f"드라이버 예외 응답: {resp.hex()}")
        return resp

    def read_registers(self, address: int, count: int = 1) -> list[int]:
        pdu = struct.pack(">BHH", 0x03, address, count)
        resp = self._transact(pdu, expected_len=3 + count * 2 + 2)
        data = resp[3:3 + count * 2]
        return list(struct.unpack(f">{count}H", data))

    def write_register(self, address: int, value: int):
        pdu = struct.pack(">BHH", 0x06, address, value & 0xFFFF)
        self._transact(pdu, expected_len=8)

    def write_registers(self, address: int, values: list[int]):
        count = len(values)
        payload = struct.pack(">BHHB", 0x10, address, count, count * 2)
        for v in values:
            payload += struct.pack(">H", v & 0xFFFF)
        self._transact(payload, expected_len=8)

    @staticmethod
    def _pack32(value: int) -> list[int]:
        value &= 0xFFFFFFFF
        return [(value >> 16) & 0xFFFF, value & 0xFFFF]

    @staticmethod
    def _unpack32(words: list[int]) -> int:
        return (words[0] << 16) | words[1]

    # --- 기본 설정 ---

    def get_device_address(self) -> int:
        return self.read_registers(REG_ADDRESS, 1)[0]

    def set_device_address(self, new_address: int):
        if not 0x0000 <= new_address <= 0x007F:
            raise ValueError("주소 범위는 0x0000-0x007F")
        self.write_register(REG_ADDRESS, new_address)

    def set_baudrate(self, baud: int):
        self.write_registers(REG_BAUDRATE, self._pack32(baud))

    def set_current(self, run: int, hold: int):
        """run/hold: 0-31. 실제 전류 = (설정값+1)/32 * Imax(SBD-14U는 2A)."""
        if not (0 <= run <= 31 and 0 <= hold <= 31):
            raise ValueError("run/hold 전류값은 0-31 사이")
        self.write_register(REG_CURRENT, (run << 8) | hold)

    def set_home_speed(self, pulses_per_sec: int):
        self.write_registers(REG_HOME_SPEED, self._pack32(pulses_per_sec))

    def set_init_speed(self, pulses_per_sec: int):
        self.write_registers(REG_INIT_SPEED, self._pack32(pulses_per_sec))

    def set_run_speed(self, pulses_per_sec: int):
        self.write_registers(REG_RUN_SPEED, self._pack32(pulses_per_sec))

    def set_microstep(self, microstep: int):
        if microstep not in ALLOWED_MICROSTEPS:
            raise ValueError(f"분주비는 {ALLOWED_MICROSTEPS} 중 하나여야 함")
        self.write_register(REG_MICROSTEP, microstep)

    # --- 운전 ---

    def jog(
        self,
        forward: bool,
        use_right_limit: bool = False,
        right_limit_stop_on_low: bool = True,
        use_left_limit: bool = False,
        left_limit_stop_on_low: bool = True,
        home_no_sensor: bool = False,
        home_with_sensor: bool = False,
    ):
        """정역회전 모드 운전 (0x002A). 리미트/원점복귀 센서를 쓰지 않으면 전부 False로 둔다."""
        value = 0
        if home_no_sensor:
            value |= 1 << 13
        if home_with_sensor:
            value |= 1 << 12
        if forward:
            value |= 1 << 4
        if right_limit_stop_on_low:
            value |= 1 << 3
        if left_limit_stop_on_low:
            value |= 1 << 2
        if use_right_limit:
            value |= 1 << 1
        if use_left_limit:
            value |= 1 << 0
        self.write_register(REG_DIR_MODE, value)

    def move_to(
        self,
        steps: int,
        negative_direction: bool = False,
        use_right_limit: bool = False,
        right_limit_stop_on_low: bool = True,
        use_left_limit: bool = False,
        left_limit_stop_on_low: bool = True,
    ):
        """좌표모드 운전 (0x002B). steps: 원점 기준 목표 스텝 (0 ~ 0x00FFFFFF)."""
        if not 0 <= steps <= 0x00FFFFFF:
            raise ValueError("steps는 0 ~ 0x00FFFFFF 범위")
        value = steps & 0x00FFFFFF
        if negative_direction:
            value |= 1 << 31
        if right_limit_stop_on_low:
            value |= 1 << 27
        if left_limit_stop_on_low:
            value |= 1 << 26
        if use_right_limit:
            value |= 1 << 25
        if use_left_limit:
            value |= 1 << 24
        self.write_registers(REG_COORD, self._pack32(value))

    def stop(self, lock: bool = True):
        """lock=True: 모터 잠금(EN 활성화). lock=False: 잠금 해제(외력으로 회전 가능)."""
        self.write_register(REG_STOP, 0 if lock else 1)
        time.sleep(2.0)  # 매뉴얼: 운전정지 후 2초간 다른 지령 송신 금지

    def read_status(self) -> dict:
        raw = self.read_registers(REG_STATUS, 1)[0]
        return {
            "raw": raw,
            "stopped": bool(raw & (1 << 15)),
            "overheat_warning": bool(raw & (1 << 14)),
            "overheat_critical": bool(raw & (1 << 13)),
            "actual_current": (raw >> 4) & 0x1F,
            "right_switch": bool(raw & (1 << 1)),
            "left_switch": bool(raw & (1 << 0)),
        }
