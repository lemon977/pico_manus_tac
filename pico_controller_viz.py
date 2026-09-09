#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pico_controller_viz.py — PICO 双手柄坐标系 + 后续 MANUS 对齐 的 MeshCat 3D 可视化

两个阶段（同一份代码/同一场景）:

阶段1  只看 PICO:
  - retarget 模式(默认): 画 pico_retarget 输出的机器人头系腕位姿 (head=原点)
  - raw 模式: 画原始手柄经 LH->RH + Pico->Robot 轴变换 + 本地(1,0,1)180° 后的绝对腕系
  每只手一组 RGB 轴 (红=X 绿=Y 蓝=Z), 世界原点也画一组参考轴。

阶段2  叠加 MANUS 手 (双硬件坐标对齐验收):
  MANUS raw skeleton 约定为腕根系位置。HandMotion_None 时 node0 四元数≈I；
  HandMotion_IMU/Auto 时 node0 四元数会转、位置轴随腕动——绘制前必须 hand_local()
  收到真腕局部(与 calibrate_wrist / export_dataset 一致), 再用 T_calib:

      T_world_manus_node = T_world_pico_ref @ T_calib @ T_manus_local

  对齐成功时: 同一只手 PICO 参考轴(粗) 与 MANUS 腕轴(细) 原点重合、朝向一致
  (或只差一个已知的 T_calib)。HUD 打印腕原点距离 + 轴向夹角, 便于调 T_calib。

统一数据包 (viz 只吃这个, 不绑死某一硬件):

    bundle = {
      "frame_id": "neck_yaw_link" | "pico_world",
      "head":  (pos, quat) | None,
      "left":  {"pico_ref": (pos,quat)|None,
                "manus_nodes": (N,7) np | None,
                "manus_parents": (N,) np | None,
                "manus_count": int},
      "right": {...同上...},
    }
  外参 T_calib_left / T_calib_right 由 ControllerVisualizer 持有 (calib 文件或默认 I)。

用法:
  # 离线回放 pico_receiver 落盘的 JSONL (无需头显)
  python3 pico_controller_viz.py logs/pico_xxx.jsonl                 # retarget 模式
  python3 pico_controller_viz.py logs/pico_xxx.jsonl --mode raw      # 原始手柄系
  python3 pico_controller_viz.py logs/pico_xxx.jsonl --manus-demo    # 叠一只合成 MANUS 手(验证对齐管线)
  python3 pico_controller_viz.py logs/pico_xxx.jsonl --mode raw --hands left  # 只画左手

  # 被 pico_receiver.py --viz / --viz-raw import 使用 (实时)
"""

import argparse
import json
import math
import sys
import threading
import time

import numpy as np

from record_control import parse_hands, hands_label, HANDS_CHOICES

# 复用 retarget 里的位姿数学与轴变换, 保证与实际下发机器人的坐标约定完全一致
from pico_retarget import (
    Q_CTRL_TO_WRIST,
    IDENTITY_POSE,
    apply_pico_to_robot_axes,
    compose_pose,
    convert_lh_to_rh,
    quat_normalize,
    quat_to_mat,
)

try:
    import meshcat
    import meshcat.geometry as g
    _HAS_MESHCAT = True
except Exception:  # pragma: no cover - 环境未装 meshcat 时降级
    meshcat = None
    g = None
    _HAS_MESHCAT = False


# ------------------------------------------------------------------ 4x4 变换工具

def pose_to_T(pos, quat):
    """(pos[3], quat[x,y,z,w]) -> 4x4 numpy 齐次变换。"""
    T = np.eye(4)
    T[:3, :3] = np.array(quat_to_mat(list(quat)))
    T[:3, 3] = np.array(pos, dtype=float)
    return T


def T_to_pose(T):
    """4x4 -> (pos, quat[x,y,z,w])。"""
    from pico_retarget import mat_to_quat
    R = [[float(T[i][j]) for j in range(3)] for i in range(3)]
    return [float(T[i][3]) for i in range(3)], mat_to_quat(R)


def calib_entry_to_T(entry):
    """calib JSON 的一条 -> 4x4。支持 {"matrix":4x4} 或 {"pos":[3],"quat":[4]}。"""
    if entry is None:
        return np.eye(4)
    if isinstance(entry, dict):
        if "matrix" in entry:
            return np.array(entry["matrix"], dtype=float).reshape(4, 4)
        pos = entry.get("pos", [0.0, 0.0, 0.0])
        quat = entry.get("quat", [0.0, 0.0, 0.0, 1.0])
        return pose_to_T(pos, quat)
    arr = np.array(entry, dtype=float)
    if arr.size == 16:
        return arr.reshape(4, 4)
    if arr.size == 7:
        return pose_to_T(arr[:3], arr[3:7])
    return np.eye(4)


def load_calib(path):
    """读取外参文件, 返回 (T_calib_left, T_calib_right)。缺省单位阵。"""
    if not path:
        return np.eye(4), np.eye(4)
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    left = calib_entry_to_T(data.get("left"))
    right = calib_entry_to_T(data.get("right"))
    return left, right


def frame_geodesic_deg(Ta, Tb):
    """两个位姿旋转之间的测地夹角(度), 用于对齐质检。"""
    Ra = Ta[:3, :3]
    Rb = Tb[:3, :3]
    dR = Ra.T @ Rb
    c = (np.trace(dR) - 1.0) / 2.0
    c = max(-1.0, min(1.0, float(c)))
    return math.degrees(math.acos(c))


# ------------------------------------------------------------------ 骨架几何
# 从 PICO 的 头 + 双腕(世界系) 估计一个上半身火柴人:
# 头→颈→胸(躯干原点), 胸→肩→肘→腕。肘用 2 连杆平面 IK 估计(向下弯), 使手臂
# 有自然弯折, 观感接近 RViz teleop_cartesian。真实值仅头/腕, 其余为解剖学估计。
BODY_DIMS = {
    "neck": 0.12,          # 头心 -> 颈根
    "spine": 0.30,         # 颈根 -> 胸(躯干原点)
    "shoulder_half": 0.18,  # 胸 -> 肩(左右各)
    "upper_arm": 0.28,     # 肩 -> 肘
    "forearm": 0.26,       # 肘 -> 腕
}


def _rot_y_to(direction):
    """把局部 +Y 轴旋到 direction 的 3x3 旋转(用于把圆柱摆成骨骼)。"""
    d = np.asarray(direction, dtype=float)
    n = np.linalg.norm(d)
    if n < 1e-9:
        return np.eye(3)
    d = d / n
    y = np.array([0.0, 1.0, 0.0])
    v = np.cross(y, d)
    s = float(np.linalg.norm(v))
    c = float(np.dot(y, d))
    if s < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
    return np.eye(3) + vx + vx @ vx * ((1.0 - c) / (s * s))


def _horiz(v, up):
    v = np.asarray(v, dtype=float)
    return v - float(np.dot(v, up)) * up


def _elbow_point(shoulder, wrist, l1, l2, up):
    """2 连杆平面 IK: 给定肩/腕与上臂/前臂长度, 返回向下弯的肘点。"""
    S = np.asarray(shoulder, dtype=float)
    W = np.asarray(wrist, dtype=float)
    d = W - S
    dn = float(np.linalg.norm(d))
    if dn < 1e-6:
        return (S + W) / 2.0
    u = d / dn
    if dn >= l1 + l2 - 1e-4:  # 够不着 -> 直臂
        return S + u * l1
    a = (l1 * l1 - l2 * l2 + dn * dn) / (2.0 * dn)
    h = math.sqrt(max(0.0, l1 * l1 - a * a))
    M = S + a * u
    bend = -np.asarray(up, dtype=float)  # 向下弯
    n = bend - float(np.dot(bend, u)) * u
    if np.linalg.norm(n) < 1e-6:
        n = np.array([1.0, 0.0, 0.0]) - u * u[0]
    nn = np.linalg.norm(n)
    if nn < 1e-6:
        return M
    return M + h * (n / nn)


def build_body_points(head_pos, R_head, refs):
    """由 头位姿 + 双腕(refs 里的 4x4) 估计躯干/肩点。返回 dict。"""
    up = np.array([0.0, 0.0, 1.0])
    cols = [R_head[:, i] for i in range(3)]
    # 头三轴里最竖直的当作头的"上", 其余两轴里靠 wrist 连线的当作"右"
    vert_idx = int(np.argmax([abs(np.dot(c, up)) for c in cols]))
    rem = [i for i in range(3) if i != vert_idx]
    span = None
    if "left" in refs and "right" in refs:
        span = _horiz(refs["right"][:3, 3] - refs["left"][:3, 3], up)
        if np.linalg.norm(span) < 1e-6:
            span = None
    if span is not None:
        dots = [abs(np.dot(_horiz(cols[i], up), span)) for i in rem]
        right_idx = rem[int(np.argmax(dots))]
    else:
        right_idx = rem[0]
    right = _horiz(cols[right_idx], up)
    if np.linalg.norm(right) < 1e-6:
        right = np.array([1.0, 0.0, 0.0])
    right = right / np.linalg.norm(right)
    if span is not None and np.dot(right, span) < 0:
        right = -right
    fwd = np.cross(up, right)
    if np.linalg.norm(fwd) < 1e-6:
        fwd = np.array([0.0, 1.0, 0.0])
    fwd = fwd / np.linalg.norm(fwd)

    head_pos = np.asarray(head_pos, dtype=float)
    neck = head_pos - up * BODY_DIMS["neck"]
    chest = neck - up * BODY_DIMS["spine"]
    return {
        "head": head_pos, "neck": neck, "chest": chest,
        "shoulderL": chest - right * BODY_DIMS["shoulder_half"],
        "shoulderR": chest + right * BODY_DIMS["shoulder_half"],
        "right": right, "fwd": fwd, "up": up,
    }


# ------------------------------------------------------------------ bundle 构建

def _extract_pose(obj):
    """从解析后的 dict 取 (pos, quat); 兼容 pos/quat 数组或 pose 字符串。"""
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


def _pico_to_world(pose):
    """头/手柄统一: 原始 PICO 左手系 -> 世界右手系 X前 Y左 Z上(不做腕系 180°)。"""
    return apply_pico_to_robot_axes(convert_lh_to_rh(pose))


def _ctrl_to_teleop_wrist(pose):
    """可选: 遥操腕系 = 世界系手柄再叠本地 (1,0,1)180°。默认 ego 可视化不用。"""
    return compose_pose(_pico_to_world(pose), (IDENTITY_POSE[0], list(Q_CTRL_TO_WRIST)))


def _axis_dirs_str(T):
    """把一个坐标系的三根轴在世界系里的指向打成一行, 便于对着实物核对朝向。
    世界系为 X前 Y左 Z上, 所以 X→[+1,0,0] 表示该轴指向正前方。"""
    R = np.asarray(T, dtype=float)[:3, :3]
    return "  ".join(f"{n}→[{R[0, i]:+.2f},{R[1, i]:+.2f},{R[2, i]:+.2f}]"
                     for i, n in enumerate("XYZ"))


def _blank_side():
    return {"pico_ref": None, "pico_wrist": None,
            "manus_nodes": None, "manus_parents": None, "manus_count": 0}


def build_bundle_raw(value):
    """ego 默认: 头与手柄同一世界系(LH→RH + Pico→Robot 轴)。
    pico_ref = 转换后手柄原点系; pico_wrist 仅在需要对比遥操腕系时填充。"""
    bundle = {"frame_id": "pico_world", "head": None,
              "left": _blank_side(), "right": _blank_side()}
    head = _extract_pose(value.get("Head"))
    if head is not None:
        bundle["head"] = _pico_to_world(head)
    ctrl = value.get("Controller") if isinstance(value.get("Controller"), dict) else {}
    for side in ("left", "right"):
        p = _extract_pose(ctrl.get(side))
        if p is not None and (p[0][0] ** 2 + p[0][1] ** 2 + p[0][2] ** 2) > 1e-8:
            bundle[side]["pico_ref"] = _pico_to_world(p)
            bundle[side]["pico_wrist"] = _ctrl_to_teleop_wrist(p)
    return bundle


def build_bundle_retarget(retargeted):
    """retarget 模式: 机器人头系(neck_yaw_link), head=原点。"""
    if not isinstance(retargeted, dict):
        return None
    bundle = {"frame_id": retargeted.get("frame_id", "neck_yaw_link"),
              "head": (list(IDENTITY_POSE[0]), list(IDENTITY_POSE[1])),
              "left": _blank_side(), "right": _blank_side()}
    for side, key in (("left", "left_wrist_rel"), ("right", "right_wrist_rel")):
        d = retargeted.get(key)
        if isinstance(d, dict) and "pos" in d and "quat" in d:
            bundle[side]["pico_ref"] = (list(d["pos"]), list(d["quat"]))
    return bundle


# ------------------------------------------------------------------ 可视化器

_AXIS_COLORS = (0xff3333, 0x33dd33, 0x3366ff)  # X 红 / Y 绿 / Z 蓝
# 用线段拼出 X/Y/Z 字母(单位方格 [0,1]x[0,1] 内的笔画), 贴在各轴尖端做标注。
_GLYPHS = {
    "X": (((0, 0), (1, 1)), ((0, 1), (1, 0))),
    "Y": (((0.5, 0), (0.5, 0.5)), ((0.5, 0.5), (0, 1)), ((0.5, 0.5), (1, 1))),
    "Z": (((0, 1), (1, 1)), ((1, 1), (0, 0)), ((0, 0), (1, 0))),
}


class ControllerVisualizer:
    """MeshCat 3D 可视化: 世界参考轴 + 头 + 双手柄轴 + 可选 MANUS 手。

    实时: 每帧调 update_from_pico(parsed_value, retargeted); 内部按 hz 限频。
    MANUS: 由外部源(ROS/离线)调 set_manus(side, nodes, parents, count)。
    """

    def __init__(self, mode="retarget", axis_len=0.12, manus_axis_len=0.08,
                 hz=30.0, calib_path=None, zmq_url=None, overlap=False,
                 skeleton=True, bone_radius=0.02, draw_points=False,
                 show_origins=False, show_wrist=False, hands="both"):
        if not _HAS_MESHCAT:
            raise RuntimeError("未安装 meshcat, 请先: pip install meshcat")
        self.mode = mode
        self.hands = parse_hands(hands)
        self.axis_len = float(axis_len)
        # show_origins: 手柄原点/MANUS腕原点各画一个小球+连线, 便于手动调 --calib 平移
        self.show_origins = bool(show_origins)
        # show_wrist: 额外画遥操腕系细轴(含 (1,0,1)180°); 默认 ego 只画统一世界系手柄轴
        self.show_wrist = bool(show_wrist)
        self.show_ctrl_raw = self.show_wrist  # 兼容旧属性名
        # 开 show_origins 时把 MANUS 腕轴画长些, 免得被粗手柄轴盖住
        self.manus_axis_len = float(manus_axis_len) * (1.6 if show_origins else 1.0)
        # skeleton: 把 头/躯干/双臂/腕 连成带色胶囊的火柴人(类 RViz teleop_cartesian)
        self.skeleton = bool(skeleton)
        self.bone_radius = float(bone_radius)
        self.period = 1.0 / hz if hz > 0 else 0.0
        # 性能: 缓存材质避免每帧重建; MANUS 点云默认关(骨架线已够, 少一半消息)
        self.draw_points = bool(draw_points)
        self._mat_cache = {}
        # overlap: 把左右手柄两组轴都画到世界原点(去掉平移), 只比朝向 ->
        #          "举向正前方时两坐标系应重合、转向一致" 的直接验收视图
        self.overlap = bool(overlap)
        self._last_hud = 0.0
        self.calib_path = calib_path
        self.T_calib = {"left": np.eye(4), "right": np.eye(4)}
        if calib_path:
            l, r = load_calib(calib_path)
            self.T_calib["left"], self.T_calib["right"] = l, r
        self.vis = meshcat.Visualizer() if zmq_url is None else meshcat.Visualizer(zmq_url)
        self._last = 0.0
        self._lock = threading.Lock()
        self._manus = {"left": None, "right": None}  # side -> (nodes, parents, count)
        self._objects_ready = set()
        self._scene_static()
        print(f"[viz] MeshCat: {self.vis.url()}")
        print("[viz] 远程查看: 在本机开 SSH 隧道后浏览器打开该地址, 例如")
        print("[viz]   ssh -L 7000:127.0.0.1:7000 lemon@<本机IP>  再访问 http://127.0.0.1:7000/static/")
        print("[viz] 坐标轴: X=红 Y=绿 Z=蓝(仅世界原点 world 标了 X/Y/Z 字母做图例, "
              "其余坐标系同一配色)。world 始终在原点不动, 不是头。")
        if self.mode == "raw":
            print("[viz] 模式=world(ego简约定): 头/双手柄同一世界系 X前Y左Z上"
                  "(仅 LH→RH + 轴映射, 不做遥操腕系180°)。")
        else:
            print("[viz] 模式=retarget: head 固定在原点, 画相对头系腕位姿(遥操视图)。")
        if self.show_wrist:
            print("[viz] --viz-wrist: 额外画细轴=遥操腕系(含180°), 粗轴仍是统一世界系手柄。")
        if self.overlap:
            print("[viz] ⚠ 已开 --viz-overlap: 左右手柄平移被清零画到同一原点, 只比朝向。"
                  "这会人为让两者重合, 不代表真实空间位置; 看真实位置请去掉该开关。")
        if self.hands != frozenset({"left", "right"}):
            print(f"[viz] --hands={hands_label(self.hands)}: 只画所选侧手柄/手套。")

    # -------------------------------------------------- 场景搭建
    def _scene_static(self):
        # 世界原点参考系(长一点, 便于判方向: X前 Y左 Z上), 只有它标 X/Y/Z 字母做图例
        self._draw_triad("world", self.axis_len * 1.5, thick=True, label=True)

    def _draw_triad(self, path, length, thick=False, label=False):
        """在 path 下建 3 段有色线作为坐标轴; 之后只需 set_transform 移动整组。
        label=True 时在轴尖端拼出 X/Y/Z 字母(仅世界原点用, 避免画面太乱)。"""
        segs = ((length, 0, 0), (0, length, 0), (0, 0, length))
        for name, color, end in zip(("x", "y", "z"), _AXIS_COLORS, segs):
            verts = np.array([[0.0, end[0]], [0.0, end[1]], [0.0, end[2]]], dtype=np.float32)
            mat = g.LineBasicMaterial(color=color, linewidth=(6 if thick else 2))
            self.vis[path][name].set_object(g.LineSegments(g.PointsGeometry(verts), mat))
        if label:
            self._draw_axis_labels(path, length)
        self._objects_ready.add(path)

    def _draw_axis_labels(self, path, length):
        """在每根轴尖端用线段拼出 X/Y/Z 字母(颜色与该轴一致), 便于把 MANUS xyz 对齐到该系。
        字母作为 path 的子物体, 随 set_transform 一起移动, 无需每帧重建。"""
        s = 0.4 * length          # 字母大小
        off = 1.12                # 沿轴外推, 避免和轴线重叠
        # (子路径, 字母, 颜色, 轴单位向量, 字母平面的横轴 u, 纵轴 v)
        specs = (
            ("lx", "X", _AXIS_COLORS[0], (1, 0, 0), (0, 1, 0), (0, 0, 1)),
            ("ly", "Y", _AXIS_COLORS[1], (0, 1, 0), (1, 0, 0), (0, 0, 1)),
            ("lz", "Z", _AXIS_COLORS[2], (0, 0, 1), (1, 0, 0), (0, 1, 0)),
        )
        for cpath, letter, color, ax, u, v in specs:
            ax = np.asarray(ax, float); u = np.asarray(u, float); v = np.asarray(v, float)
            base = ax * length * off
            pts = []
            for (a0, b0), (a1, b1) in _GLYPHS[letter]:
                pts.append(base + s * (a0 - 0.5) * u + s * b0 * v)
                pts.append(base + s * (a1 - 0.5) * u + s * b1 * v)
            verts = np.asarray(pts, dtype=np.float32).T
            self.vis[path][cpath].set_object(
                g.LineSegments(g.PointsGeometry(verts),
                               g.LineBasicMaterial(color=color, linewidth=3)))

    def set_calib(self, side, T):
        with self._lock:
            self.T_calib[side] = np.array(T, dtype=float).reshape(4, 4)

    def reload_calib(self):
        """重新读取 --calib 文件(边改 json 边看)。文件缺失/损坏时保持当前外参。"""
        if not self.calib_path:
            return
        try:
            l, r = load_calib(self.calib_path)
        except Exception as e:  # noqa: BLE001
            print(f"[viz] 重载外参失败, 沿用旧值: {e}", file=sys.stderr)
            return
        with self._lock:
            self.T_calib["left"], self.T_calib["right"] = l, r
        print(f"[viz] 已重载外参 {self.calib_path}", flush=True)

    def set_manus(self, side, nodes, parents=None, count=None):
        """线程安全地喂入一帧 MANUS (腕根系) 数据。nodes: (N,7)。"""
        side = "left" if str(side).lower().startswith("l") else "right"
        if side not in self.hands:
            return
        nodes = np.asarray(nodes, dtype=float)
        if count is None:
            count = nodes.shape[0]
        with self._lock:
            self._manus[side] = (nodes, None if parents is None else np.asarray(parents),
                                 int(count))

    # -------------------------------------------------- 每帧更新
    def update_from_pico(self, parsed_value, retargeted=None, force=False):
        now = time.time()
        if not force and self.period and (now - self._last) < self.period:
            return None
        self._last = now
        if self.mode == "raw":
            bundle = build_bundle_raw(parsed_value)
        else:
            bundle = build_bundle_retarget(retargeted)
            if bundle is None:
                return None
        return self.update(bundle)

    def update(self, bundle):
        """按 bundle 画 head/左右 pico_ref/MANUS, 返回对齐质检 metrics。"""
        metrics = {}
        # overlap 视图: 只关心朝向, 隐藏 head 参考(避免遮挡)
        if bundle.get("head") is not None and not self.overlap:
            self._set_triad("head", pose_to_T(*bundle["head"]), self.axis_len)
        else:
            self._hide("head")

        with self._lock:
            manus_snapshot = dict(self._manus)
            calib = dict(self.T_calib)

        refs = {}
        wrist_refs = {}
        for side in ("left", "right"):
            if side not in self.hands:
                self._hide(f"pico_{side}")
                self._hide(f"wrist_{side}")
                self._hide(f"manus_{side}")
                self._hide(f"manus_{side}_bones")
                self._hide(f"manus_{side}_pts")
                if self.show_origins:
                    self._hide(f"orig_ctrl_{side}")
                    self._hide(f"orig_wrist_{side}")
                    self._hide(f"orig_link_{side}")
                continue
            sd = bundle.get(side, _blank_side())
            ref = sd.get("pico_ref")
            if ref is None:
                self._hide(f"pico_{side}")
                self._hide(f"wrist_{side}")
                self._hide(f"manus_{side}")
                self._hide(f"manus_{side}_bones")
                continue
            T_ref = pose_to_T(*ref)
            refs[side] = T_ref
            # 可选: 遥操腕系细轴(不参与骨架/MANUS, 只对比)
            if self.show_wrist:
                wpose = sd.get("pico_wrist")
                if wpose is None:
                    self._hide(f"wrist_{side}")
                else:
                    T_w = pose_to_T(*wpose)
                    if self.overlap:
                        T_w[:3, 3] = 0.0
                    wrist_refs[side] = T_w
                    self._set_triad(f"wrist_{side}", T_w, self.axis_len * 0.7)
            # overlap: 画在原点(仅保留旋转), 便于直接比对左右朝向是否重合
            T_draw = T_ref.copy()
            if self.overlap:
                T_draw[:3, 3] = 0.0
            # 左手轴略长, 便于在重合视图里区分谁是谁
            self._set_triad(f"pico_{side}", T_draw,
                            self.axis_len * (1.25 if side == "left" else 1.0))

            # MANUS 叠加: 优先 bundle 内, 否则用最近一次 set_manus 的快照
            nodes = sd.get("manus_nodes")
            parents = sd.get("manus_parents")
            count = sd.get("manus_count") or 0
            if nodes is None and manus_snapshot.get(side) is not None:
                nodes, parents, count = manus_snapshot[side]
            if nodes is not None and count > 0:
                T_world = T_draw @ calib[side]
                self._draw_manus(side, T_world, np.asarray(nodes), parents, int(count))
                # 质检: MANUS 腕(node0) 世界系 vs PICO 参考系
                d = float(np.linalg.norm(T_world[:3, 3] - T_draw[:3, 3]))
                metrics[side] = {"wrist_dist_m": d,
                                 "wrist_angle_deg": frame_geodesic_deg(T_draw, T_world)}
                if self.show_origins:
                    # 手柄原点(球) & MANUS 手套腕原点(球) + 连线, 便于手动对齐
                    p_ctrl = T_draw[:3, 3]
                    p_wrist = T_world[:3, 3]
                    self._set_marker(f"orig_ctrl_{side}", p_ctrl, 0x22ff22, 0.014)
                    self._set_marker(f"orig_wrist_{side}",
                                     p_wrist, 0x00e5ff if side == "left" else 0xffa500, 0.010)
                    self._set_link(f"orig_link_{side}", p_ctrl, p_wrist, 0xffff00)
                    metrics[side]["delta_local_cm"] = (
                        (T_draw[:3, :3].T @ (p_wrist - p_ctrl)) * 100.0).tolist()
            else:
                self._hide(f"manus_{side}")
                self._hide(f"manus_{side}_bones")
                if self.show_origins:
                    self._hide(f"orig_ctrl_{side}")
                    self._hide(f"orig_wrist_{side}")
                    self._hide(f"orig_link_{side}")

        # 连线骨架(类 RViz teleop_cartesian): 头/躯干/双臂/腕 用带色胶囊连起来。
        # overlap 只比朝向, 不画骨架。
        if self.skeleton and not self.overlap and bundle.get("head") is not None:
            self._draw_skeleton(pose_to_T(*bundle["head"]), refs)
        else:
            self._hide_skeleton()

        # 头↔手柄距离 HUD (world 模式): 便于当场发现手柄冻结/未握持
        # (真实握持时 |H-L|,|H-R| 约 0.3~0.8m; 若某值恒定不变=该手柄卡死/未跟踪)。
        head_pose = bundle.get("head")
        if not self.overlap and head_pose is not None:
            head_pos = np.asarray(head_pose[0], dtype=float)
            dist = {}
            for side in ("left", "right"):
                if side in refs:
                    dist[side] = float(np.linalg.norm(refs[side][:3, 3] - head_pos))
            if dist:
                metrics["head_dist_m"] = dist

        # 左右手柄朝向/位置一致性
        if "left" in refs and "right" in refs:
            lr = frame_geodesic_deg(refs["left"], refs["right"])
            pos_gap = float(np.linalg.norm(refs["left"][:3, 3] - refs["right"][:3, 3]))
            metrics["lr"] = {"orient_diff_deg": lr, "pos_gap_m": pos_gap}
            now = time.time()
            if now - self._last_hud > 0.5:
                self._last_hud = now
                if self.overlap:
                    print(f"[重合视图] 左右手柄朝向差={lr:5.1f}°  原点间距={pos_gap*100:5.1f}cm"
                          f"  ({'朝向一致√' if lr < 8 else '朝向不一致'})", flush=True)
                else:
                    hd = metrics.get("head_dist_m", {})
                    hl = hd.get("left")
                    hr = hd.get("right")
                    hl_s = f"{hl*100:5.1f}" if hl is not None else "  n/a"
                    hr_s = f"{hr*100:5.1f}" if hr is not None else "  n/a"
                    line = (f"[world] |H-L|={hl_s}cm  |H-R|={hr_s}cm  |L-R|={pos_gap*100:5.1f}cm"
                            f"  左右朝向差={lr:5.1f}°")
                    mseg = []
                    for side, tag in (("left", "左"), ("right", "右")):
                        m = metrics.get(side)
                        if m:
                            seg = (f"{tag}手套腕↔手柄原点 d={m['wrist_dist_m']*100:.1f}cm "
                                   f"a={m['wrist_angle_deg']:.1f}°")
                            dl = m.get("delta_local_cm")
                            if dl is not None:
                                seg += (f" 偏差(手柄系 前/左/上)="
                                        f"[{dl[0]:+.1f},{dl[1]:+.1f},{dl[2]:+.1f}]cm")
                            mseg.append(seg)
                    if mseg:
                        line += "  | MANUS " + "  ".join(mseg)
                    print(line, flush=True)
                    if self.show_wrist:
                        for side, tag in (("left", "左"), ("right", "右")):
                            if side not in refs:
                                continue
                            print(f"  [{tag}] 手柄系(粗) {_axis_dirs_str(refs[side])}",
                                  flush=True)
                            if side in wrist_refs:
                                print(f"  [{tag}] 遥操腕系(细) "
                                      f"{_axis_dirs_str(wrist_refs[side])}", flush=True)
        return metrics

    # -------------------------------------------------- 绘制原语
    def _material(self, kind, color, linewidth=2, size=0.008):
        """材质缓存: 同色/同类型只建一次, 每帧重用(减少对象分配与序列化)。"""
        key = (kind, color, linewidth, size)
        m = self._mat_cache.get(key)
        if m is None:
            if kind == "line":
                m = g.LineBasicMaterial(color=color, linewidth=linewidth)
            elif kind == "pts":
                m = g.PointsMaterial(size=size, color=color)
            else:
                m = g.MeshLambertMaterial(color=color)
            self._mat_cache[key] = m
        return m

    def _set_triad(self, path, T, length):
        if path not in self._objects_ready:
            self._draw_triad(path, length, thick=(path.startswith("pico") or path == "head"))
        self.vis[path].set_transform(np.asarray(T, dtype=float))

    def _set_marker(self, path, pos, color, radius=0.012):
        """在世界坐标 pos 处画一个小球(标记原点位置)。"""
        if path not in self._objects_ready:
            self.vis[path].set_object(g.Sphere(radius), self._material("mesh", color))
            self._objects_ready.add(path)
        T = np.eye(4)
        T[:3, 3] = np.asarray(pos, dtype=float)
        self.vis[path].set_transform(T)

    def _set_link(self, path, p0, p1, color):
        """在 p0->p1 之间画一根直线(连接两个原点, 直观看物理间距)。"""
        verts = np.array([[p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]]], dtype=np.float32)
        self.vis[path].set_object(g.LineSegments(g.PointsGeometry(verts),
                                                 self._material("line", color, linewidth=2)))
        self._objects_ready.add(path)

    def _hide(self, path):
        if path in self._objects_ready:
            self.vis[path].delete()
            self._objects_ready.discard(path)

    def _draw_manus(self, side, T_world, nodes, parents, count):
        n = min(count, nodes.shape[0])
        nodes = np.asarray(nodes, dtype=float)
        # 与 export/calib 一致: 先腕局部再乘 T_world(=pico∘T_calib)
        if nodes.shape[1] >= 7:
            from export_dataset import hand_local
            local = hand_local(nodes[:n])
        else:
            local = nodes[:n, :3]
        R = T_world[:3, :3]
        t = T_world[:3, 3]
        world = (R @ local.T).T + t  # (n,3)

        # 腕轴= T_world(已含手柄∘T_calib); 不再叠 node0 IMU 四元数
        T_wrist = T_world
        self._set_triad(f"manus_{side}", T_wrist, self.manus_axis_len)

        col = 0x00e5ff if side == "left" else 0xffa500
        # 节点点云(默认关, 少一半 set_object 消息; --viz-points 打开)
        if self.draw_points:
            pts = world.T.astype(np.float32)  # (3,n)
            self.vis[f"manus_{side}_pts"].set_object(
                g.Points(g.PointsGeometry(pts),
                         self._material("pts", col, size=0.008)))
            self._objects_ready.add(f"manus_{side}_pts")

        # 骨架线: 每个节点连到 parent (材质缓存, 只重建几何顶点)
        if parents is not None:
            segs = []
            for i in range(n):
                p = int(parents[i]) if i < len(parents) else -1
                if 0 <= p < n:
                    segs.append(world[i]); segs.append(world[p])
            if segs:
                verts = np.array(segs, dtype=np.float32).T  # (3, 2E)
                self.vis[f"manus_{side}_bones"].set_object(
                    g.LineSegments(g.PointsGeometry(verts),
                                   self._material("line", col, linewidth=2)))
                self._objects_ready.add(f"manus_{side}_bones")
            else:
                self._hide(f"manus_{side}_bones")

    # -------------------------------------------------- 连线骨架
    _SKELETON_PATHS = (
        "skel/head", "skel/spine_up", "skel/spine", "torso",
        "skel/clav_left", "skel/upper_left", "skel/fore_left",
        "skel/clav_right", "skel/upper_right", "skel/fore_right",
    )

    def _set_bone(self, path, p0, p1, radius, color):
        """在 p0->p1 之间画一根胶囊(圆柱)。用单位圆柱 + 非均匀缩放, 避免每帧重建几何。"""
        p0 = np.asarray(p0, dtype=float)
        p1 = np.asarray(p1, dtype=float)
        L = float(np.linalg.norm(p1 - p0))
        if L < 1e-6:
            self._hide(path)
            return
        if path not in self._objects_ready:
            self.vis[path].set_object(g.Cylinder(1.0, 1.0),
                                      g.MeshLambertMaterial(color=color))
            self._objects_ready.add(path)
        T = np.eye(4)
        T[:3, :3] = _rot_y_to(p1 - p0) @ np.diag([radius, L, radius])
        T[:3, 3] = (p0 + p1) / 2.0
        self.vis[path].set_transform(T)

    def _draw_skeleton(self, head_T, refs):
        head_pos = head_T[:3, 3]
        b = build_body_points(head_pos, head_T[:3, :3], refs)
        r = self.bone_radius

        # 头(红胶囊, 沿前向) + 脊柱 头->颈->胸
        self._set_bone("skel/head", head_pos - b["fwd"] * 0.08,
                       head_pos + b["fwd"] * 0.08, 0.05, 0xff3b30)
        self._set_bone("skel/spine_up", b["head"], b["neck"], r, 0xff3b30)
        self._set_bone("skel/spine", b["neck"], b["chest"], r, 0xff3b30)

        # 躯干原点三轴(以身体系摆放)
        T_torso = np.eye(4)
        T_torso[:3, 0] = b["right"]
        T_torso[:3, 1] = b["fwd"]
        T_torso[:3, 2] = b["up"]
        T_torso[:3, 3] = b["chest"]
        self._set_triad("torso", T_torso, self.axis_len)

        for side, sh in (("left", b["shoulderL"]), ("right", b["shoulderR"])):
            if side not in refs:
                for p in (f"skel/clav_{side}", f"skel/upper_{side}", f"skel/fore_{side}"):
                    self._hide(p)
                continue
            wrist = refs[side][:3, 3]
            elbow = _elbow_point(sh, wrist, BODY_DIMS["upper_arm"],
                                 BODY_DIMS["forearm"], b["up"])
            self._set_bone(f"skel/clav_{side}", b["chest"], sh, r * 0.8, 0x00e5ff)
            self._set_bone(f"skel/upper_{side}", sh, elbow, r, 0x33dd33)
            self._set_bone(f"skel/fore_{side}", elbow, wrist, r, 0x3366ff)

    def _hide_skeleton(self):
        for p in self._SKELETON_PATHS:
            self._hide(p)


# ------------------------------------------------------------------ 合成 MANUS 手(自测/演示)

def synthetic_manus_hand(side="left"):
    """造一只极简 MANUS 手(腕根系): node0=手腕, 每指 4 节沿 +X 展开。
    仅用于验证 T_calib @ 变换 @ 绘制管线, 不代表真实解剖。"""
    nodes = [[0, 0, 0, 0, 0, 0, 1]]          # 手腕
    parents = [-1]
    finger_y = [-0.03, -0.015, 0.0, 0.015, 0.03]
    idx = 0
    for fy in finger_y:
        prev = 0
        for j in range(1, 5):
            nodes.append([0.02 * j, fy, 0.0, 0, 0, 0, 1])
            parents.append(prev)
            idx += 1
            prev = len(nodes) - 1
    nodes = np.array(nodes, dtype=float)
    parents = np.array(parents, dtype=np.int32)
    return nodes, parents, nodes.shape[0]


# ------------------------------------------------------------------ 离线回放

def replay_jsonl(path, mode="retarget", hz=30.0, calib_path=None,
                 manus_demo=False, loop=False, overlap=False, skeleton=True,
                 show_origins=False, show_wrist=False, hands="both"):
    from pico_retarget import Retargeter
    viz = ControllerVisualizer(mode=mode, hz=hz, calib_path=calib_path,
                               overlap=overlap, skeleton=skeleton,
                               show_origins=show_origins,
                               show_wrist=show_wrist, hands=hands)
    if manus_demo:
        for side in ("left", "right"):
            nodes, parents, cnt = synthetic_manus_hand(side)
            viz.set_manus(side, nodes, parents, cnt)
        print("[viz] 已注入合成 MANUS 手用于对齐管线演示 (--manus-demo)")

    rt = Retargeter()
    n_in = n_out = 0
    while True:
        with open(path, encoding="utf-8") as f:
            for line in f:
                try:
                    rec = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if rec.get("functionName") != "Tracking" or not isinstance(rec.get("data"), dict):
                    continue
                n_in += 1
                data = rec["data"]
                retargeted = rec.get("retarget")
                if retargeted is None and mode != "raw":
                    retargeted = rt.process(data, rec.get("recv_ts"))
                m = viz.update_from_pico(data, retargeted, force=True)
                if m is not None:
                    n_out += 1
                    if m:
                        # "lr" 朝向差已在 update_from_pico 内降频打印, 这里只打 MANUS 对齐项
                        for side, v in m.items():
                            if side == "lr" or "wrist_dist_m" not in v:
                                continue
                            print(f"[align] {side}: 腕距={v['wrist_dist_m']*100:.1f}cm "
                                  f"轴向夹角={v['wrist_angle_deg']:.1f}°")
                if hz > 0:
                    time.sleep(1.0 / hz)
        print(f"[replay] 输入 {n_in} 帧, 绘制 {n_out} 帧", file=sys.stderr)
        if not loop:
            break
        n_in = n_out = 0


def _aligned_to_bundle(rec):
    """把 align_pico_manus.py 的一条对齐记录 -> viz bundle(世界系)。
    需要对齐时用 --full 保留 MANUS 全节点; 否则只画 PICO。"""
    def mk(p):
        # p 形如 [[x,y,z],[qx,qy,qz,qw]]
        return {"pos": p[0], "quat": p[1]} if p else None
    val = {}
    if rec.get("head"):
        val["Head"] = mk(rec["head"])
    ctrl = {}
    if rec.get("left_ctrl"):
        ctrl["left"] = mk(rec["left_ctrl"])
    if rec.get("right_ctrl"):
        ctrl["right"] = mk(rec["right_ctrl"])
    val["Controller"] = ctrl
    bundle = build_bundle_raw(val)  # 头/手柄统一世界系(LH→RH + Pico→Robot 轴)
    manus = rec.get("manus") or {}
    for side in ("left", "right"):
        m = manus.get(side)
        if not m:
            continue
        nodes = m.get("nodes")
        if not nodes:  # 没跑 --full: 退化为只用腕(node0)画一根细轴
            wp = m.get("wrist_pose")
            if wp:
                nodes = [wp]
        if not nodes:
            continue
        arr = np.asarray(nodes, dtype=float)
        # IMU/Auto: 去掉 node0 腕朝向, 与标定/导出同一腕局部系; None 下为幂等。
        if arr.ndim == 2 and arr.shape[0] > 0 and arr.shape[1] >= 7:
            from export_dataset import hand_local
            xyz = hand_local(arr)
            arr = arr.copy()
            arr[: xyz.shape[0], :3] = xyz
            arr[0, 3:7] = (0.0, 0.0, 0.0, 1.0)
        parents = m.get("parent_ids")
        bundle[side]["manus_nodes"] = arr
        bundle[side]["manus_parents"] = None if parents is None else np.asarray(parents)
        bundle[side]["manus_count"] = int(m.get("node_count") or arr.shape[0])
    return bundle


def replay_aligned(path, hz=30.0, calib_path=None, loop=False,
                   overlap=False, skeleton=True, realtime=False,
                   draw_points=False, max_fps=30.0, show_origins=False,
                   calib_reload=False, hands="both"):
    """回放 align_pico_manus.py 的对齐 JSONL: PICO 世界系 + 真实 MANUS 手同场景。

    性能: --realtime 时按录制 wall 还原速度, 但绘制上限 max_fps(默认30), 录制帧率
    高于此就跳帧只按真实时间推进, 避免刷爆 meshcat 造成卡顿。
    """
    viz = ControllerVisualizer(mode="raw", hz=hz, calib_path=calib_path,
                               overlap=overlap, skeleton=skeleton,
                               draw_points=draw_points, show_origins=show_origins,
                               hands=hands)
    print("[viz] 回放对齐文件: 头/手柄(世界系) + 真实 MANUS 手骨架。")
    print("[viz] MANUS 手套腕默认贴在手柄原点(T_calib=I); 标定 T_calib 后可精确重合。")
    if show_origins:
        print("[viz] --show-origins: 绿球=手柄原点, 青/橙球=MANUS手套腕原点, 黄线=二者连线; "
              "HUD 打印手柄局部系偏差[前,左,上]cm(即 --calib 的 pos 该给多少)。")
    if calib_reload:
        print("[viz] --calib-reload: 每轮循环重读外参; 改完 config/calib_wrist.json 存盘即生效(配合 --loop)。")
    print(f"[viz] 性能: 绘制上限 {max_fps:.0f}fps, MANUS 点云={'开' if draw_points else '关(仅骨架线)'}。")
    min_period = 1.0 / max_fps if max_fps > 0 else 0.0
    while True:
        if calib_reload:
            viz.reload_calib()
        prev_wall = None
        last_draw = 0.0
        n = drawn = 0
        with open(path, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    rec = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if "pico_wall_ns" not in rec:
                    continue
                n += 1
                now = time.time()
                # 跳帧: 距上次绘制不足 min_period 就不画(仍按 wall 推进时间)
                skip = min_period and (now - last_draw) < min_period
                if not skip:
                    viz.update(_aligned_to_bundle(rec))
                    last_draw = now
                    drawn += 1
                if realtime and rec.get("pico_wall_ns"):
                    if prev_wall is not None:
                        dt = (rec["pico_wall_ns"] - prev_wall) / 1e9
                        if 0.0 < dt < 0.5:
                            time.sleep(dt)
                    prev_wall = rec["pico_wall_ns"]
                elif hz > 0:
                    time.sleep(1.0 / hz)
        print(f"[replay-aligned] 读取 {n} 帧, 绘制 {drawn} 帧", file=sys.stderr)
        if not loop:
            break


def _selftest():
    """无头自测: 不依赖头显/日志, 构造合成数据跑通 bundle+变换+(可选)绘制。"""
    val = {"Head": {"pos": [0, 1.5, 0], "quat": [0, 0, 0, 1]},
           "Controller": {"left": {"pos": [0.3, 1.4, -0.2], "quat": [0, 0, 0, 1]},
                          "right": {"pos": [-0.3, 1.4, -0.2], "quat": [0, 0, 0, 1]}}}
    b = build_bundle_raw(val)
    assert b["left"]["pico_ref"] is not None and b["right"]["pico_ref"] is not None
    T = pose_to_T(*b["left"]["pico_ref"])
    assert T.shape == (4, 4)
    nodes, parents, cnt = synthetic_manus_hand("left")
    Tw = T @ np.eye(4)
    world = (Tw[:3, :3] @ nodes[:cnt, :3].T).T + Tw[:3, 3]
    assert world.shape == (cnt, 3)
    assert frame_geodesic_deg(np.eye(4), np.eye(4)) < 1e-6

    # 骨架数学: 旋转对齐 / 肘 IK / 身体点
    R = _rot_y_to([0, 0, 1])
    assert np.allclose(R @ np.array([0, 1, 0]), [0, 0, 1], atol=1e-6)
    sh = np.array([0.0, 0.0, 0.0]); wr = np.array([0.4, 0.0, 0.0])
    e = _elbow_point(sh, wr, 0.28, 0.26, np.array([0, 0, 1.0]))
    assert e[2] <= 1e-6 and abs(np.linalg.norm(e - sh) - 0.28) < 1e-2, "肘在上臂长度上、向下弯"
    refs = {"left": pose_to_T([0.3, 0.2, 1.0], [0, 0, 0, 1]),
            "right": pose_to_T([-0.3, 0.2, 1.0], [0, 0, 0, 1])}
    b = build_body_points([0.0, 0.0, 1.6], np.eye(3), refs)
    assert b["chest"][2] < b["head"][2], "胸应在头下方"
    assert np.linalg.norm(b["shoulderL"] - b["shoulderR"]) > 0.2, "左右肩应分开"

    # 轴标注字母: 每个字母至少 2 笔, 顶点有限且非退化
    for letter in ("X", "Y", "Z"):
        strokes = _GLYPHS[letter]
        assert len(strokes) >= 2
        for (a0, b0), (a1, b1) in strokes:
            assert (a0, b0) != (a1, b1)
    # 对齐记录 -> bundle: 真实 MANUS 节点应被挂到对应侧
    arec = {"pico_wall_ns": 1, "head": [[0, 0, 1.6], [0, 0, 0, 1]],
            "left_ctrl": [[0.3, 0.2, 1.0], [0, 0, 0, 1]],
            "right_ctrl": [[-0.3, 0.2, 1.0], [0, 0, 0, 1]],
            "manus": {"left": {"nodes": nodes.tolist(), "parent_ids": parents.tolist(),
                               "node_count": int(cnt)}}}
    ab = _aligned_to_bundle(arec)
    assert ab["left"]["manus_nodes"] is not None and ab["left"]["manus_count"] == cnt
    assert ab["left"]["pico_ref"] is not None
    print("[selftest] OK: bundle/变换/合成MANUS/骨架/轴标注/对齐回放 管线正常")


def main():
    ap = argparse.ArgumentParser(description="PICO 手柄坐标系 + MANUS 对齐 MeshCat 可视化")
    ap.add_argument("jsonl", nargs="?", help="pico_receiver 落盘的 JSONL (离线回放)")
    ap.add_argument("--mode", choices=["retarget", "raw"], default="retarget",
                    help="retarget=机器人头系相对位姿(默认); raw=手柄原点绝对世界位姿")
    ap.add_argument("--hz", type=float, default=30.0, help="回放/刷新帧率")
    ap.add_argument("--calib", default=None, help="外参 JSON (PICO参考系->MANUS腕系)")
    ap.add_argument("--aligned", action="store_true",
                    help="输入是 align_pico_manus.py 的对齐 JSONL(叠真实 MANUS 手); 建议对齐时用 --full")
    ap.add_argument("--realtime", action="store_true",
                    help="--aligned 回放时按录制时间戳还原真实速度(否则按 --hz)")
    ap.add_argument("--max-fps", type=float, default=30.0,
                    help="--aligned 回放绘制上限帧率(跳帧防卡顿), 默认30")
    ap.add_argument("--viz-points", action="store_true",
                    help="额外画 MANUS 节点点云(默认只画骨架线, 更流畅)")
    ap.add_argument("--show-origins", action="store_true",
                    help="醒目标出'手柄原点'(粗轴球)与'MANUS手套腕原点'(细轴球)两个点+连线, "
                         "并打印二者在手柄局部系的偏差(dx,dy,dz cm), 便于手动调 --calib 平移偏移")
    ap.add_argument("--viz-wrist", "--ctrl-raw", dest="viz_wrist", action="store_true",
                    help="额外画遥操腕系细轴(含 (1,0,1)180°); 默认粗轴是统一世界系手柄"
                         "(与头同约定)。--ctrl-raw 为旧别名")
    ap.add_argument("--calib-reload", action="store_true",
                    help="配合 --aligned --loop: 每轮重新读取 --calib 文件, 边改 json 边看对齐效果")
    ap.add_argument("--manus-demo", action="store_true", help="叠一只合成 MANUS 手验证对齐管线")
    ap.add_argument("--overlap", action="store_true",
                    help="把左右手柄两组轴都画到原点(仅比朝向); 举向正前方时应重合、转向一致")
    ap.add_argument("--no-skeleton", action="store_true",
                    help="不画连线火柴人骨架, 只保留坐标轴")
    ap.add_argument("--hands", default="both", choices=list(HANDS_CHOICES),
                    help="只画哪只手: left / right / both(默认)")
    ap.add_argument("--loop", action="store_true", help="循环回放")
    ap.add_argument("--selftest", action="store_true", help="无头自测(不需 meshcat/日志)")
    args = ap.parse_args()

    if args.selftest:
        _selftest()
        return
    if not args.jsonl:
        ap.error("请提供 JSONL 路径, 或用 --selftest")
    if args.aligned:
        replay_aligned(args.jsonl, hz=args.hz, calib_path=args.calib, loop=args.loop,
                       overlap=args.overlap, skeleton=not args.no_skeleton,
                       realtime=args.realtime, draw_points=args.viz_points,
                       max_fps=args.max_fps, show_origins=args.show_origins,
                       calib_reload=args.calib_reload, hands=args.hands)
    else:
        replay_jsonl(args.jsonl, mode=args.mode, hz=args.hz, calib_path=args.calib,
                     manus_demo=args.manus_demo, loop=args.loop, overlap=args.overlap,
                     skeleton=not args.no_skeleton, show_origins=args.show_origins,
                     show_wrist=args.viz_wrist, hands=args.hands)


if __name__ == "__main__":
    main()
