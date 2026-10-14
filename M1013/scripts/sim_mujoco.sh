#!/bin/bash
# M1013 MuJoCo 시뮬레이션 실행 (가상 컨트롤러 + MuJoCo)
# 네임스페이스를 sim01 로 고정해서 실물 로봇(dsr01)과 명령이 섞이지 않게 한다.
# 사용법: ./sim_mujoco.sh [추가 launch 인자...]   예) ./sim_mujoco.sh gripper:=2f85

NS=sim01
WS=~/robotis_ws
LAUNCH_FILE=$WS/TabletCrusher/M1013/launch/m1013_mujoco.launch.py
LOG=/tmp/sim_mujoco_${NS}.log

export PATH=$(echo "$PATH" | sed -e 's|[^:]*anaconda3[^:]*:||g')
export MUJOCO_DIR=${MUJOCO_DIR:-$HOME/mujoco/mujoco-3.2.7}
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash

if pgrep -f "__ns:=/[s]im01" > /dev/null; then
  echo "이미 /${NS} 시뮬레이션이 실행 중입니다. 먼저 종료하세요."
  exit 1
fi

cleanup() {
  echo "종료 중..."
  kill -INT $LAUNCH_PID 2>/dev/null
  sleep 3
  pkill -f "__ns:=/[s]im01"
  docker rm -f ${NS}_emulator > /dev/null 2>&1
}
trap cleanup EXIT INT TERM

EXTRA=""
[ "$MIRROR" = "1" ] && EXTRA="mj_xacro:=$WS/TabletCrusher/M1013/description/m1013_mirror.urdf.xacro"
ros2 launch $LAUNCH_FILE model:=m1013 name:=${NS} $EXTRA "$@" > $LOG 2>&1 &
LAUNCH_PID=$!
echo "launch 시작 (로그: $LOG)"

for i in $(seq 1 40); do
  grep -q "INITIAL STATE_STANDBY" $LOG && break
  if grep -q "process has died.*ros2_control_node" $LOG; then
    echo "control node가 에뮬레이터 접속에 실패했습니다. 로그 확인: $LOG"
    exit 1
  fi
  sleep 1
done
if ! grep -q "INITIAL STATE_STANDBY" $LOG; then
  echo "40초 안에 Standby 상태가 안 됐습니다. 로그 확인: $LOG"
  exit 1
fi

# launch 안의 spawner(12초 지연)가 끝나길 기다렸다가, 실패했으면 다시 띄운다.
sleep 12
if ! ros2 control list_controllers -c /${NS}/controller_manager 2>/dev/null | grep "dsr_controller2" | grep -q active; then
  echo "컨트롤러 재시작..."
  ros2 run controller_manager spawner joint_state_broadcaster dsr_controller2 -c /${NS}/controller_manager
fi

echo ""
echo "준비 완료. 다른 터미널에서:  python3 $WS/TabletCrusher/M1013/scripts/sim_demo.py"
echo "종료: Ctrl+C"
wait $LAUNCH_PID
