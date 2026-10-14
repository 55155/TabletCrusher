# Doosan M1013 구동 노트

TabletCrusher 프로젝트에서 사용할 Doosan M1013 (6축 협동로봇, 가반 10kg, 리치 1300mm) 구동 과정과
시뮬레이터(Genesis / MuJoCo) 연동 가능성을 정리하는 디렉토리.

## 0. 환경
- ROS2 Humble (이 머신에 이미 설치됨: `/opt/ros/humble`)
- 공식 패키지: [DoosanRobotics/doosan-robot2](https://github.com/DoosanRobotics/doosan-robot2)

## 1. doosan-robot2 설치

```bash
cd ~/robotis_ws/src   # 이 워크스페이스의 ROS2 src에 설치
git clone https://github.com/DoosanRobotics/doosan-robot2.git -b humble
cd ~/robotis_ws
rosdep install --from-paths src/doosan-robot2 --ignore-src -r -y

# 가상모드(에뮬레이터) 사용 시 Docker 필요
cd src/doosan-robot2
sudo ./install_emulator.sh

cd ~/robotis_ws
colcon build --symlink-install
# 컨트롤러 버전이 3.x인 실물이면: colcon build --cmake-args -DDRCF_VER=3
```

## 2. 구동 (Bringup)

> 실제 구동 명령어 모음은 [`commands.md`](commands.md), 제어권/모드/I/O 배선은 [`controller.md`](controller.md).

**가상모드 — 실물 로봇 없이 테스트**
```bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
  mode:=virtual host:=127.0.0.1 port:=12345 model:=m1013
```

**실물모드 — 컨트롤러 IP로 연결**
```bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
  mode:=real host:=192.168.137.100 port:=12345 model:=m1013
```

기본 동작 명령은 `dsr_msgs2`의 서비스/액션(`move_joint`, `move_line` 등) 또는
`dsr_example2` 예제 스크립트로 확인.

## 3. 시뮬레이터 연동

### MuJoCo — 공식 지원 ✅
`doosan-robot2`에 `dsr_mujoco` 패키지가 포함되어 있어 바로 연동 가능.

```bash
# 사전 준비
sudo apt install libglfw3-dev
cd ~/robotis_ws/src
git clone https://github.com/moveit/mujoco_ros2_control.git
cd ~/robotis_ws && colcon build

# 실행 (이 디렉토리 스크립트 — 업스트림 launch의 시작 타이밍 문제 우회, 네임스페이스 sim01 고정)
./scripts/sim_mujoco.sh
```
- MuJoCo 3.2.7, `mujoco_ros2_control` 커밋 `bd2d576` (doosan-robot2 README 지정) 사용.
- 실물 로봇을 MuJoCo에 실시간 미러링: `./scripts/real_mujoco.sh` — 업스트림 설정(PID 추종)은 약 0.7초 지연이라
  PID를 뺀 미러 전용 설명 파일(`description/`)을 쓴다. 자세한 내용은 [`commands.md`](commands.md).
- `mujoco_ros2_control`이 `ros2_control` 컨트롤러 매니저와 MuJoCo 물리엔진을 브릿지함.
- 즉, 실물에 보내는 것과 동일한 `ros2_control` 트래젝토리/조인트 명령이 그대로 MuJoCo 시뮬레이션에 적용됨 → 실물 구동 전 검증용으로 바로 쓸 수 있음.

### Genesis — 공식 지원 없음, 직접 연동 필요 ⚠️
Doosan 쪽에 Genesis용 공식 패키지는 없지만 아래 경로로 연동 가능:
1. `dsr_description2`가 제공하는 M1013 URDF를 Genesis의 `gs.morphs.URDF(file=...)`로 바로 로드.
2. 참고용 오픈소스: [`genesis_ros`](https://github.com/vybhav-ibr/genesis_ros) — Genesis ↔ ROS2 토픽 브릿지 패키지. 이 구조를 참고해 `dsr_control2`가 퍼블리시하는 `/dsr01/joint_states`를 Genesis 씬에 미러링하고, 반대로 Genesis에서 계획한 궤적을 `move_joint` 액션으로 실물/에뮬레이터에 전달.
3. **주의**: `genesis_ros`는 토픽 기반이라 업데이트 주기가 실시간 컨트롤 수준은 아님 — 하드 리얼타임 서보잉이 아니라 "계획 검증 + 비주얼 디지털 트윈" 용도로는 적합, 고속 폐루프 제어엔 부적합.

### 권장 워크플로
1. 가상모드로 기본 동작(`move_joint` 등) 확인
2. MuJoCo로 궤적/충돌 검증 (공식 지원이라 가장 안정적)
3. 필요 시 Genesis는 별도 브릿지 노드로 비주얼/오프라인 검증 용도로만 추가

## 4. 배선 (컨트롤러 박스)

M1013을 구동하는 "큰 박스"는 Doosan 컨트롤러(DRCF 컨트롤 박스)입니다.
설치/사용자 매뉴얼 (저작권 문서라 저장소에는 올리지 않음 — 로컬 `docs/`에 따로 받아둘 것):
- `docs/Doosan_Installation_Manual_M-series_v1.6.pdf` — M-series(M1013 포함) 전용 설치 매뉴얼 (스캔본, 116p)
- `docs/Doosan_Installation_Manual_Vention_v2.7.1.pdf` — 범용 설치 매뉴얼 (텍스트 포함, 190p) — **배선 핵심 내용은 여기**
- `docs/Doosan_User_Manual_v2.7.1.pdf` — 사용자 매뉴얼 (펜던트 조작, 242p)
- 한국어 온라인 매뉴얼: https://manual.doosanrobotics.com/ko/user-manual/3.2.2/1-m-h-series/part-3

핵심 배선 순서 (매뉴얼 3.3절 / 4.2절 기준):

1. **매니퓰레이터 ↔ 컨트롤러**: 로봇 케이블을 컨트롤러 커넥터에 꽂고 고정 링 체결. "딸깍" 소리가 날 때까지 밀어넣기. 전원 켜진 상태에서 분리하면 로봇 손상 위험.
2. **컨트롤러 ↔ 티치 펜던트**: 티치 펜던트 케이블을 컨트롤러 커넥터에 "딸깍" 소리 날 때까지 삽입.
3. **케이블 라우팅**: 매니퓰레이터/티치펜던트 케이블 최소 곡률 반경 120mm 이상 유지.
4. **컨트롤러 전원**: 표준 IEC C13/C14 케이블로 연결. 입력 전압 100–240VAC, 퓨즈 15A, 주파수 47–63Hz. 접지 필수, 서킷 브레이커로 보호.
5. **컨트롤러 I/O 단자** (내부 터미널 블록 6종):
   - `TBSFT` — 안전 접점 입력 (비상정지 스위치, 라이트커튼, 안전매트)
   - `TBSTO` — 안전 접점 출력 (파워 스위치용, STO 상태에서 오픈)
   - `TBPWR` — 디지털 I/O 전원 (기본 내부 24V/2A, 2A 넘으면 외부 24V를 VIO/GIO에 별도 연결)
   - `TBCI1-4 / TBCO1-4` — 설정 가능 디지털 입력 16개 / 출력 16개 (PLC, 릴레이, 솔레노이드밸브, 컨베이어 엔코더 등 연결)
   - `TBAIO` — 아날로그 I/O
   - `TBEN1/2` — 엔코더 입력
6. **엔드플랜지 I/O (X1/X2, M8 8핀)**: 그리퍼/센서용. 핀맵 — 1:DI, 2-4:DO, 5:+24V, 6-7:DI, 8:GND (X1/X2 각각 DI 3개 + DO 3개). 내부 24V/3A 공급.

> 상세 핀맵/배선도 그림은 텍스트로 옮기기 어려워서 원본 PDF(`docs/Doosan_Installation_Manual_Vention_v2.7.1.pdf`) 3.3~4.2절 참고.

## 5. 진행 상황
- [x] `doosan-robot2` clone & build (ROS2 Humble, 29 패키지)
- [x] 실물 연결 — 컨트롤러 IP는 `192.168.137.100` (`.1` 은 다른 장비), USB-이더넷 동글로 별도 망
- [x] 실물 구동 (`move_joint`, `movel` IK) — 펜던트 Auto 모드 전환 불필요
- [x] MuJoCo 시뮬 연동 + 실물 실시간 미러 (지연 측정 약 55ms, 녹화 시 MuJoCo 표시 약 23Hz)
- [ ] Genesis 브릿지 필요성 재검토 (MuJoCo로 충분한지 먼저 판단)
- [ ] TabletCrusher 장비(흡착 그리퍼 등)를 컨트롤러 디지털 I/O로 연동
