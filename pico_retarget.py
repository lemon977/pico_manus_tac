#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pico_retarget.py — pico_tracking_retarget_node.cpp 的纯 Python 移植

以 C++ 节点为准的完整后处理管线（采集配置: 头显 + 双手柄, 无 Motion Tracker）:
  parseTrackingJson -> convertLHToRH(左手系->右手系, z 取反)
    -> applyPicoToRobotAxes(Pico X右Y上Z前 -> 机器人 X前Y左Z上)
    -> applyLocalAxisRotation(手柄本地系绕 (1,0,1) 转 180° -> 手腕系 X前Y左Z上)
    -> computeRelativePose(全部转为相对头位姿; 头=identity)
    -> torso 无 tracker 走兜底: 首帧头位姿锁定 + 头后 0.1m/头下 0.45m 偏移
    -> ensureQuaternionContinuity(四元数符号连续)
    -> smoothPose(位置线性 + 姿态 SLERP, alpha=0.5)

坐标约定（同 cpp）: 输出在机器人头系 neck_yaw_link, X前 Y左 Z上, 头为原点。
发布条件（同 cpp）: 头 + 左右手柄三者位姿都有效才输出;
手柄未握持上报零位置+单位四元数, 按位置模长判活跃 (controllerActive)。

可独立使用: python3 pico_retarget.py logs/pico_xxx.jsonl  (离线回放 JSONL)
也可被 pico_receiver.py import (Retargeter 类)。
"""

import json
import math
import sys

# ---------------------------------------------------------------- 四元数/位姿数学
# quat 一律 [x, y, z, w]; pos 一律 [x, y, z]

def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


def quat_normalize(q):
    n = math.sqrt(sum(v * v for v in q))
    if n < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    return [v / n for v in q]


def quat_conj(q):
    return [-q[0], -q[1], -q[2], q[3]]


def quat_rotate(q, p):
    """q * [p,0] * q^-1 (q 需为单位四元数)"""
    qv = [p[0], p[1], p[2], 0.0]
    r = quat_mul(quat_mul(q, qv), quat_conj(q))
    return r[:3]


def quat_from_axis_angle(axis, angle):
    n = math.sqrt(sum(v * v for v in axis))
    ax, ay, az = (v / n for v in axis)
    s = math.sin(angle / 2.0)
    return [ax * s, ay * s, az * s, math.cos(angle / 2.0)]


def quat_to_mat(q):
    x, y, z, w = quat_normalize(q)
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def mat_to_quat(m):
    """3x3 旋转矩阵 -> [x,y,z,w]（Shepperd 法）"""
    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return quat_normalize([
            (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s,
            (m[1][0] - m[0][1]) / s, 0.25 * s])
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        return quat_normalize([
            0.25 * s, (m[0][1] + m[1][0]) / s,
            (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s])
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        return quat_normalize([
            (m[0][1] + m[1][0]) / s, 0.25 * s,
            (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s])
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
    return quat_normalize([
        (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s,
        0.25 * s, (m[1][0] - m[0][1]) / s])


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_transpose(m):
    return [[m[j][i] for j in range(3)] for i in range(3)]


def mat_vec(m, v):
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


def slerp(qa, qb, t):
    qa = quat_normalize(qa)
    qb = quat_normalize(qb)
    dot = sum(a * b for a, b in zip(qa, qb))
    if dot < 0.0:
        qb = [-v for v in qb]
        dot = -dot
    if dot > 0.9995:
        return quat_normalize([a + t * (b - a) for a, b in zip(qa, qb)])
    theta = math.acos(max(-1.0, min(1.0, dot)))
    sa = math.sin((1 - t) * theta) / math.sin(theta)
    sb = math.sin(t * theta) / math.sin(theta)
    return [sa * a + sb * b for a, b in zip(qa, qb)]


def compose_pose(parent, child):
    """parent * child, 位姿为 (pos, quat) 元组"""
    pp, qp = parent
    pc, qc = child
    rp = quat_rotate(qp, pc)
    return ([pp[0] + rp[0], pp[1] + rp[1], pp[2] + rp[2]],
            quat_normalize(quat_mul(qp, qc)))


def relative_pose(reference, target):
    """reference^-1 * target"""
    pr, qr = reference
    pt, qt = target
    qi = quat_conj(quat_normalize(qr))
    dp = [pt[0] - pr[0], pt[1] - pr[1], pt[2] - pr[2]]
    rp = quat_rotate(qi, dp)
    return (rp, quat_normalize(quat_mul(qi, qt)))


# ---------------------------------------------------------------- cpp 原样移植

# applyPicoToRobotAxes 的固定矩阵 (cpp: kR)
R_PICO_TO_ROBOT = [
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
]
R_PICO_TO_ROBOT_T = mat_transpose(R_PICO_TO_ROBOT)

IDENTITY_POSE = ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])

# 默认参数 (与 cpp declare_parameter 默认值一致)
TORSO_BEHIND_HEAD_M = 0.1
TORSO_BELOW_HEAD_M = 0.45
POSE_FILTER_ALPHA = 0.5
TIMEOUT_S = 0.5

# 手柄本地系 -> 手腕系: 绕 (1,0,1) 转 180° (cpp: controller_to_wrist_axis)
Q_CTRL_TO_WRIST = quat_from_axis_angle((1.0, 0.0, 1.0), math.pi)


def convert_lh_to_rh(pose):
    """Pico 跟踪(含手柄)左手系 -> 右手系 (cpp: convertLHToRH)"""
    (p, q) = pose
    return ([p[0], p[1], -p[2]], [q[0], q[1], -q[2], -q[3]])


def apply_pico_to_robot_axes(pose):
    """Pico 轴 -> 机器人轴 (cpp: applyPicoToRobotAxes)"""
    (p, q) = pose
    p2 = mat_vec(R_PICO_TO_ROBOT, p)
    m = mat_mul(mat_mul(R_PICO_TO_ROBOT, quat_to_mat(q)), R_PICO_TO_ROBOT_T)
    return (p2, mat_to_quat(m))


def controller_active(pose):
    """未握持时位置全零 (cpp: controllerActive)"""
    p = pose[0]
    return p[0] * p[0] + p[1] * p[1] + p[2] * p[2] > 1e-8


def ensure_quat_continuity(q, prev):
    """q 与 -q 同旋转, 保持帧间符号连续 (cpp: ensureQuaternionContinuity)"""
    if sum(v * v for v in q) < 1e-8:
        return q, prev
    if prev is not None and sum(a * b for a, b in zip(q, prev)) < 0.0:
        q = [-v for v in q]
    return q, q


def smooth_pose(prev, target, alpha):
    """位置线性 + 姿态 SLERP (cpp: smoothPose)"""
    alpha = max(0.0, min(1.0, alpha))
    p = [(1 - alpha) * a + alpha * b for a, b in zip(prev[0], target[0])]
    q = slerp(prev[1], target[1], alpha)
    return (p, q)


class Retargeter:
    """无 tracker 配置: 头 + 双手柄, torso 走首帧锁定兜底。
    用法: 每收到一帧解析后的 Tracking value (pico_receiver 的 data 字段)
          调 process(value, recv_ts) -> dict 或 None"""

    def __init__(self, alpha=POSE_FILTER_ALPHA, timeout_s=TIMEOUT_S,
                 torso_behind_head_m=TORSO_BEHIND_HEAD_M,
                 torso_below_head_m=TORSO_BELOW_HEAD_M):
        self.alpha = alpha
        self.timeout_s = timeout_s
        self.torso_behind_head_m = torso_behind_head_m
        self.torso_below_head_m = torso_below_head_m
        self._reset()

    def _reset(self):
        self.torso_locked = False
        self.torso_fixed = None
        self.prev_quat = {"torso": None, "left": None, "right": None}
        self.filtered = {"torso": None, "left": None, "right": None}
        self.last_ts = None

    @staticmethod
    def _extract_pose(obj):
        """从 pico_receiver 解析后的 dict 取 (pos, quat); 兼容 pos/quat 数组或 pose 字符串"""
        if not isinstance(obj, dict):
            return None
        pos, quat = obj.get("pos"), obj.get("quat")
        if isinstance(pos, list) and isinstance(quat, list) and len(pos) == 3 and len(quat) == 4:
            return ([float(v) for v in pos], [float(v) for v in quat])
        s = obj.get("pose")
        if isinstance(s, str):
            try:
                v = [float(x) for x in s.split(",")]
            except ValueError:
                return None
            if len(v) >= 7:
                return (v[:3], v[3:7])
        return None

    def process(self, value, recv_ts=None):
        """输入一帧 Tracking value, 输出 retarget 结果 dict; 不满足发布条件返回 None"""
        if not isinstance(value, dict):
            return None

        head = self._extract_pose(value.get("Head"))
        ctrl = value.get("Controller") if isinstance(value.get("Controller"), dict) else {}
        left = self._extract_pose(ctrl.get("left"))
        right = self._extract_pose(ctrl.get("right"))

        # cpp: 发布成功条件 head_valid && left_valid && right_valid (按位置判活跃)
        if head is None or left is None or right is None:
            return None
        head_t = apply_pico_to_robot_axes(convert_lh_to_rh(head))
        left_t = apply_pico_to_robot_axes(convert_lh_to_rh(left))
        right_t = apply_pico_to_robot_axes(convert_lh_to_rh(right))
        if not (controller_active(left_t) and controller_active(right_t)):
            return None

        # 超时重置 (cpp: timeout_duration 0.5s -> resetTrackingState)
        if self.last_ts is not None and recv_ts is not None \
                and recv_ts - self.last_ts > self.timeout_s:
            self._reset()
        self.last_ts = recv_ts

        # 手柄本地系 -> 手腕系 (cpp: applyLocalAxisRotation axis(1,0,1) pi)
        left_t = compose_pose(left_t, (IDENTITY_POSE[0], Q_CTRL_TO_WRIST))
        right_t = compose_pose(right_t, (IDENTITY_POSE[0], Q_CTRL_TO_WRIST))

        # torso: 无 tracker -> 首帧锁定 (cpp: else 分支 torso_locked_)
        if not self.torso_locked:
            offset = ([-self.torso_behind_head_m, 0.0, -self.torso_below_head_m],
                      IDENTITY_POSE[1])
            self.torso_fixed = compose_pose(head_t, offset)
            self.torso_locked = True
        torso_rel = relative_pose(head_t, self.torso_fixed)
        left_rel = relative_pose(head_t, left_t)
        right_rel = relative_pose(head_t, right_t)

        # 四元数符号连续
        out = {}
        for key, rel in (("torso", torso_rel), ("left", left_rel), ("right", right_rel)):
            q, self.prev_quat[key] = ensure_quat_continuity(rel[1], self.prev_quat[key])
            out[key] = (rel[0], q)

        # 滤波 (cpp: alpha < 1 时 smoothPose)
        if self.alpha < 1.0 - 1e-6:
            for key in ("torso", "left", "right"):
                if self.filtered[key] is None:
                    self.filtered[key] = out[key]
                else:
                    self.filtered[key] = smooth_pose(self.filtered[key], out[key], self.alpha)
                out[key] = self.filtered[key]

        return {
            "frame_id": "neck_yaw_link",
            "head_rel": {"pos": list(IDENTITY_POSE[0]), "quat": list(IDENTITY_POSE[1])},
            "torso_rel": {"pos": list(out["torso"][0]), "quat": list(out["torso"][1])},
            "left_wrist_rel": {"pos": list(out["left"][0]), "quat": list(out["left"][1])},
            "right_wrist_rel": {"pos": list(out["right"][0]), "quat": list(out["right"][1])},
            # cpp 槽位映射: [8]=waist_upper_yaw_link=torso_rel, [9]=neck_yaw_link=head_rel(identity),
            # [13]=left_wrist_roll_link=left_rel, [17]=right_wrist_roll_link=right_rel
        }


def replay_jsonl(path):
    """离线回放 pico_receiver 的 JSONL 日志, 打印每帧 retarget 结果"""
    rt = Retargeter()
    n_in = n_out = 0
    with open(path, encoding="utf-8") as f:
        for line in f:
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            if rec.get("functionName") != "Tracking" or not isinstance(rec.get("data"), dict):
                continue
            n_in += 1
            r = rt.process(rec["data"], rec.get("recv_ts"))
            if r is None:
                continue
            n_out += 1
            lw = r["left_wrist_rel"]["pos"]
            rw = r["right_wrist_rel"]["pos"]
            print(f"ts={rec.get('recv_ts', 0):.3f} "
                  f"L_wrist[{lw[0]:+.3f},{lw[1]:+.3f},{lw[2]:+.3f}] "
                  f"R_wrist[{rw[0]:+.3f},{rw[1]:+.3f},{rw[2]:+.3f}]")
    print(f"[replay] 输入 {n_in} 帧, 有效输出 {n_out} 帧", file=sys.stderr)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("用法: python3 pico_retarget.py <pico_xxx.jsonl>", file=sys.stderr)
        sys.exit(1)
    replay_jsonl(sys.argv[1])
