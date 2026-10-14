# M1013 구동 명령어

| 대상 | 네임스페이스 | 컨트롤러 |
|---|---|---|
| 실물 로봇 | `dsr01` | `192.168.137.100:12345` |
| MuJoCo 시뮬 | `sim01` | 가상 컨트롤러(Docker `dsr_emulator:3.0.1`) `127.0.0.1:12345` |

**실물과 시뮬은 반드시 다른 네임스페이스로 띄울 것.** 같은 네임스페이스면 시뮬에 보낸 명령이
실물 로봇에도 전달된다 (ROS2 서비스는 같은 이름의 서버 모두에게 요청이 간다) — 2026-10-15 실제로 발생.

모든 터미널 공통 준비 (conda가 켜져 있으면 빌드/실행이 꼬이므로 PATH에서 제거):
```bash
export PATH=$(echo "$PATH" | sed -e 's|[^:]*anaconda3[^:]*:||g')
source /opt/ros/humble/setup.bash && source ~/robotis_ws/install/setup.bash
```

## 실물 로봇

터미널 A — bringup (펜던트에서 Auto 모드 전환 불필요, 연결되면 ROS가 제어권·Servo On 자동 처리):
```bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.137.100 port:=12345 model:=m1013
```

터미널 B — 동작:
```bash
# 현재 관절 각도 / 손끝 좌표 읽기
ros2 service call /dsr01/aux_control/get_current_posj dsr_msgs2/srv/GetCurrentPosj
ros2 service call /dsr01/aux_control/get_current_posx dsr_msgs2/srv/GetCurrentPosx "{ref: 0}"

# 관절 이동 (현재 자세와 차이가 크면 크게 움직이니 주의)
ros2 service call /dsr01/motion/move_joint dsr_msgs2/srv/MoveJoint "{pos: [0, 0, 90, 0, 90, 0], vel: 20, acc: 20}"

# Python: 현재 위치에서 Z +30mm 올렸다 복귀
python3 ~/robotis_ws/TabletCrusher/M1013/scripts/test_movel_safe.py
```
`/dsr01/joint_states` 의 관절 순서는 `joint_1, 2, 4, 5, 3, 6` 이다.

## MuJoCo 시뮬레이션

터미널 A:
```bash
~/robotis_ws/TabletCrusher/M1013/scripts/sim_mujoco.sh
```
- `launch/m1013_mujoco.launch.py` 를 쓴다 — 업스트림 `dsr_bringup2_mujoco.launch.py` 복사본에
  control node(4초)·spawner(12초) 시작 지연만 추가한 것. 업스트림 그대로 쓰면 에뮬레이터 컨테이너보다
  control node가 먼저 접속해서 실패하는 경우가 많다.
- Ctrl+C로 종료하면 에뮬레이터 컨테이너까지 정리된다.

터미널 B:
```bash
python3 ~/robotis_ws/TabletCrusher/M1013/scripts/sim_demo.py
```
1번 축 좌우 회전 → 손끝 직선 이동(IK) 데모. `sim01` 네임스페이스에만 명령한다.

시뮬도 미러 방식으로 보고 싶으면 `MIRROR=1 ~/robotis_ws/TabletCrusher/M1013/scripts/sim_mujoco.sh`.

## 실물 로봇 + MuJoCo 실시간 미러 (영상용)

터미널 A — 실물 연결 + MuJoCo 미러 (연결 시 제어권 획득·Servo On, 움직이지는 않음):
```bash
~/robotis_ws/TabletCrusher/M1013/scripts/real_mujoco.sh
```
터미널 B — 화면 녹화 + 실물 데모 (`--real` 없으면 실물에 명령 안 함, 실물은 속도 낮춤):
```bash
/usr/bin/python3 ~/robotis_ws/TabletCrusher/M1013/scripts/screen_record.py ~/Videos/m1013.webm &
python3 ~/robotis_ws/TabletCrusher/M1013/scripts/sim_demo.py --real
kill %1      # 녹화 종료
```

**미러 방식**: 업스트림 MuJoCo 설정은 실물 각도를 목표로 PID + 큰 관절 감쇠(1200)로 쫓아가서
약 0.7초 늦고 이동 중 최대 39° 차이가 났다. `description/m1013_mirror.*.xacro` 는 PID를 빼서
mujoco_ros2_control이 관절 각도(qpos)를 그대로 써넣게 한 것 — 측정 결과 지연 0ms, 오차 0.00°.
대신 MuJoCo 속 로봇에는 중력·충돌 같은 물리가 적용되지 않는다 (보여주기용 디지털 트윈).

동기화 측정 (명령은 안 보냄): `python3 ~/robotis_ws/TabletCrusher/M1013/scripts/measure_sync.py <sim01|dsr01> <초>`

**주의**: `real_mujoco.sh` 를 끄면 펜던트에 "실시간 외부제어 중 오류" 팝업이 뜬다 — 연결이 끊긴 것뿐이고
로봇은 움직이지 않는다. 팝업 확인 후 계속 사용.

## 설치된 것 (참고)
- MuJoCo 3.2.7: `~/mujoco/mujoco-3.2.7` (`MUJOCO_DIR` 은 `~/.bashrc` 에 설정)
- `mujoco_ros2_control`: `~/robotis_ws/src/mujoco_ros2_control` (커밋 `bd2d576`, doosan-robot2 README 지정)
- 에뮬레이터 이미지: `docker pull doosanrobot/dsr_emulator:3.0.1`
