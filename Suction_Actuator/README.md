# Suction Actuator (SBD-14U 스텝모터 드라이버)

## 전원
- VCC 입력: DC 7V ~ 24V (24V 권장)
- 정상 운전전류: 0.5A/phase, 최대 2A/phase
- **24V / 150W 파워서플라이 사용 가능.** 150W@24V ≈ 6.25A로, 드라이버 최대 소모전류 대비 충분한 여유.
- 극성 주의: VM(+)/GND 반대연결 시 드라이버 손상 가능.

## 결선 (매뉴얼 기준)
- 상측: RS-485 A/B, Motor VM(+)
- 좌측: 모터 A+/A-/B+/B- (2상 바이폴러)
- 하측: 좌/우 리미트 광커플러(L/R, NPN 타입만 호환), GND, V(VCC 12-24V)
- USB-RS485 컨버터: A-A, B-B

## 통신
- Modbus RTU, 9600bps, parity none, stop bit 1 (출하 기본값), 슬레이브 주소 기본 1
- Function code: 0x03(읽기) / 0x06(단일쓰기) / 0x10(다중쓰기)

## 파일
- `sbd14u_driver.py`: Modbus RTU 드라이버 클래스 (pyserial만 사용, 외부 의존성 없음)
- `test_connection.py`: 배선/통신 확인용 스모크 테스트 (장치 주소 레지스터 읽기)

## 사용 순서
1. 결선 완료 후 USB-RS485 어댑터를 PC에 연결
2. `ls /dev/tty.*`로 포트 확인, `test_connection.py`의 `PORT` 수정
3. `python test_connection.py` 로 통신 확인 (장치 주소/상태 정상 출력되면 결선 OK)
4. 이후 `SBD14U` 클래스로 속도/전류/운전모드 설정 후 `jog()` 또는 `move_to()`로 구동
