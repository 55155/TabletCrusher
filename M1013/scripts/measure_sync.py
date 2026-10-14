#!/usr/bin/env python3
"""컨트롤러 관절 상태와 MuJoCo 관절 상태의 지연(lag)·오차를 측정한다. 로봇에 명령은 보내지 않는다.

사용법: python3 measure_sync.py <네임스페이스> <측정초>    예) python3 measure_sync.py sim01 25
"""
import sys
import time

import numpy as np
import rclpy
from sensor_msgs.msg import JointState

ns = sys.argv[1]
duration = float(sys.argv[2])
NAMES = [f"joint_{i}" for i in range(1, 7)]

rclpy.init()
node = rclpy.create_node("measure_sync")
data = {"ctrl": [], "mj": []}


def make_cb(key):
    def cb(msg):
        idx = [msg.name.index(n) for n in NAMES]
        data[key].append((time.monotonic(), [msg.position[i] for i in idx]))
    return cb


node.create_subscription(JointState, f"/{ns}/joint_states", make_cb("ctrl"), 100)
node.create_subscription(JointState, f"/{ns}/mj/joint_states", make_cb("mj"), 100)
end = time.monotonic() + duration
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.01)

t_c = np.array([d[0] for d in data["ctrl"]]); q_c = np.degrees([d[1] for d in data["ctrl"]])
t_m = np.array([d[0] for d in data["mj"]]); q_m = np.degrees([d[1] for d in data["mj"]])
t0, t1 = max(t_c[0], t_m[0]), min(t_c[-1], t_m[-1])
grid = np.arange(t0, t1, 0.005)
qc = np.stack([np.interp(grid, t_c, q_c[:, j]) for j in range(6)], 1)
qm = np.stack([np.interp(grid, t_m, q_m[:, j]) for j in range(6)], 1)

err = np.abs(qc - qm)
print(f"샘플: 컨트롤러 {len(t_c)}개 / MuJoCo {len(t_m)}개, {t1 - t0:.1f}s")
print("관절별 최대 오차(deg): " + " ".join(f"J{j+1}={err[:, j].max():.2f}" for j in range(6)))

# 가장 많이 움직인 관절로 지연 추정: MuJoCo 신호를 과거로 몇 ms 당겨야 컨트롤러와 가장 잘 맞는지
j = int(np.argmax(qc.max(0) - qc.min(0)))
best = min(range(0, 200), key=lambda k: np.mean((qc[: len(grid) - k, j] - qm[k:, j]) ** 2))
print(f"지연 추정 (J{j+1} 기준): {best * 5} ms")
