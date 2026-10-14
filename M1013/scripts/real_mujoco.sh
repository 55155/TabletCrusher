#!/bin/bash
# 실물 M1013 + MuJoCo 미러(디지털 트윈). MuJoCo 속 로봇이 실물의 관절 상태를 실시간으로 따라간다.
# 이 launch 를 켜면 ROS가 제어권을 가져가고 Servo On 한다 (움직이지는 않음).
# 사용법: ./real_mujoco.sh [추가 launch 인자...]

NS=dsr01
HOST=192.168.137.100
WS=~/robotis_ws
LAUNCH_FILE=$WS/TabletCrusher/M1013/launch/m1013_mujoco.launch.py
# PID 없이 실물 관절 각도를 MuJoCo에 그대로 써넣는 설명 파일 (지연 없는 미러)
MIRROR_XACRO=$WS/TabletCrusher/M1013/description/m1013_mirror.urdf.xacro
LOG=/tmp/real_mujoco_${NS}.log

export PATH=$(echo "$PATH" | sed -e 's|[^:]*anaconda3[^:]*:||g')
export MUJOCO_DIR=${MUJOCO_DIR:-$HOME/mujoco/mujoco-3.2.7}
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash

if pgrep -f "__ns:=/[d]sr01" > /dev/null; then
  echo "이미 /${NS} 가 실행 중입니다 (다른 bringup). 먼저 종료하세요."
  exit 1
fi

cleanup() {
  echo "종료 중..."
  kill -INT $LAUNCH_PID 2>/dev/null
  sleep 3
  pkill -f "__ns:=/[d]sr01"
}
trap cleanup EXIT INT TERM

ros2 launch $LAUNCH_FILE mode:=real host:=${HOST} model:=m1013 name:=${NS} gui:=false mj_xacro:=${MIRROR_XACRO} "$@" > $LOG 2>&1 &
LAUNCH_PID=$!
echo "launch 시작 (로그: $LOG)"

for i in $(seq 1 40); do
  grep -q "INITIAL STATE_STANDBY" $LOG && break
  if grep -q "process has died.*ros2_control_node" $LOG; then
    echo "control node가 컨트롤러($HOST) 접속에 실패했습니다. 로그 확인: $LOG"
    exit 1
  fi
  sleep 1
done
if ! grep -q "INITIAL STATE_STANDBY" $LOG; then
  echo "40초 안에 Standby 상태가 안 됐습니다. 펜던트 상태와 로그 확인: $LOG"
  exit 1
fi

sleep 12
if ! ros2 control list_controllers -c /${NS}/controller_manager 2>/dev/null | grep "dsr_controller2" | grep -q active; then
  echo "컨트롤러 재시작..."
  ros2 run controller_manager spawner joint_state_broadcaster dsr_controller2 -c /${NS}/controller_manager
fi

echo ""
echo "준비 완료 (실물 연결 + MuJoCo 미러). 다른 터미널에서:  python3 $WS/TabletCrusher/M1013/scripts/sim_demo.py --real"
echo "종료: Ctrl+C"
wait $LAUNCH_PID
