# M1013 컨트롤 박스 — 제어권/모드 전환 & I/O 배선

## 1. 제어권(Access Control) — PC(ROS2)로 가져오기

**실측 결론: Task Builder/Writer Play 탭으로 미리 Auto 모드에 들어갈 필요 없음.**
Manual 모드 + Servo Off 상태에서 그냥 `ros2 launch`만 실행해도 연결되고 제어권이 넘어왔다
(2026-10-15 M1013 실기 테스트 확인). 아래 2번 항목의 "Task Play로 먼저 Auto 모드 진입" 절차는
User Manual에 적힌 **사람이 펜던트를 수동 조작하는 절차**를 ROS 연결에도 적용될 거라고 추측한
것이었는데, 틀린 추측이었음 — 실제로는 막혀있던 원인이 전혀 다른 곳(IP 주소, 3번 항목 참고)에
있었고, 그게 풀리자 Auto 모드 수동 전환 없이도 바로 연결됐다.

**연결 IP는 `192.168.137.1`이 아니라 `192.168.137.100`.** 같은 서브넷에 MAC 주소가 다른
두 장비가 있었음 — `.1`은 컨트롤러가 아닌 다른 장비(아마 USB-이더넷 동글/게이트웨이), 실제
컨트롤러는 `.100`. `.1`은 ping만 되고 12345 포트는 항상 refused, `.100`은 포트까지 열려있었음.

이유는 `dsr_hw_interface2.cpp`(`src/doosan-robot2/dsr_hardware2/src/dsr_hw_interface2.cpp:158-197`)의
연결 로직이 TCP 연결 성공 후 제어권 요청과 Servo On까지 **ROS2 노드가 API로 직접 처리**하기 때문:

```cpp
if(!get_control_access) {
    Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);  // 제어권 강제로 요청
    continue;
}
if(!is_standby) {
    Drfl.set_robot_control(CONTROL_SERVO_ON);  // Servo On도 ROS가 직접 켬
    continue;
}
```

즉 사람이 펜던트에서 Servo On을 누르거나 Task를 Play할 필요 없이, **TCP 연결만 되면 나머지
(제어권 획득, Servo On, Standby 진입)는 ROS2 노드가 최대 10초(10회) 재시도하며 자동 처리**한다.
연결되는 순간 펜던트에 "Transfer Control" 팝업이 뜬다.

## 2. (참고) Manual ↔ Auto 모드 — 사람이 펜던트에서 직접 전환할 때만 해당

ROS2 제어에는 불필요하지만, 사람이 펜던트에서 직접 Auto 모드로 전환하고 싶을 때는 이 절차가 맞음
(M-series는 A-series Smart Pendant와 달리 물리적인 Auto 버튼이 없어서 소프트웨어로만 전환):

> User Manual 8.8절: "Moving to the Play tab disengages manual mode and engages Auto mode."

1. 펜던트에서 **Task Builder**(또는 Task Writer) 열기
2. 아무 task나 열어서 **Play 탭으로 이동** → 이 순간 Manual → Auto 전환
3. Auto 모드가 되면 Status/Power 버튼 외 나머지 메인 메뉴는 비활성화됨 (6.1절)
4. task를 stop 하면 다시 모든 버튼이 활성화됨 (Auto 모드에서 벗어남)

## 3. 컨트롤 박스 24V I/O 배선

컨트롤 박스의 **Configurable Digital I/O**(`TBCI1-4` 입력 16개, `TBCO1-4` 출력 16개)가
외부 장비와 연결하는 통로. (Installation Manual 4.2.3 / 4.2.4절)

| 항목 | 스펙 |
|---|---|
| 내부 공급(공장 기본값, TBPWR 경유) | VIO/GIO 통해 24V, **총 2A까지** |
| 2A 초과 시 | TBPWR에서 내부(VCC/GND) 연결 끊고 **외부 24V PSU**를 VIO/GIO에 직접 연결 |
| Digital Output (Oxx) | 0-24V, **채널당 최대 1A** |
| Digital Input (Ixx) | 0-30V, ON 범위 11-30V, 2-15mA |

그 외: `TBSFT`(안전 접점 입력, E-stop/protective stop 전용), `TBSTO`(안전 접점 출력,
전원 스위치용), `TBAIO`(아날로그 I/O), `TBEN1/2`(엔코더 입력) 단자도 있음.

### 전원/채널 개수 정리

| 위치 | 상시 24V 핀 | 스위칭 출력 채널 | 입력 채널 | 비고 |
|---|---|---|---|---|
| 컨트롤 박스 (TBCI/TBCO) | VIO/GIO 1개 라인 | 16개 (`O01~O16`) 채널당 최대 1A | 16개 (`I01~I16`) | 내부 소스 총 2A, 초과 시 외부 24V PSU 필요 |
| 로봇 손목 플랜지 (X1, X2) | 커넥터당 1개 → 총 2개 | 커넥터당 3채널 → 총 6채널 | 커넥터당 3채널 → 총 6채널 | 전체 공급 3A 한도 공유 |

**결론 — TabletCrusher 장비 연결 방침:**
- 컨트롤 박스 I/O는 **신호(trigger) 교환용**으로만 쓸 것 (예: "흡착 액추에이터 동작 시작" 출력,
  "액추에이터 완료 센서" 입력).
- BLDC 모터 / SBD-14U 흡착 액추에이터 / LSM-NK174218 리니어 액추에이터처럼 전류를 많이
  먹는 장비는 **컨트롤 박스에서 직접 전원 공급하지 말 것** — 채널당 1A, 합계 2A(내부 기준) 한도로는
  부족함. 기존 `TabletCrusher/Motor/`, `Suction_Actuator/`의 자체 24V 드라이버 보드를 그대로 쓰고,
  컨트롤 박스와는 디지털 I/O(start/done 신호)만 주고받는 구조로 간다.
