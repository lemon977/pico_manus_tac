#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""export_dataset.py — 把一次采集(PICO + MANUS + 可选触觉)导出成训练包 HDF5。

训练包 schema=`egodex_v1`（旧称「方案B」）: 世界系头/腕位姿 + 腕局部手指关节
+ 可选双手 369 通道原始触觉;
VST 为左右眼并排(SBS), 导出 `video_frame_idx` + 左右 crop 约定, 供单目或双目深度。
统一坐标系: 右手系 X前 Y左 Z上, 单位米 (REP-103)。

管线:
  原始 PICO(左手系) --convert_lh_to_rh--> --apply_pico_to_robot_axes--> 世界系
      头与双手柄同一约定(不做遥操腕系180°); 可选 T_calib 把手柄原点挪到手套腕(物理安装偏移)
  MANUS 25 关节 --> 转到「手腕局部系」(相对腕根, 纯手形, 与手腕世界位姿解耦)
  时间对齐(最近邻, 门限) --> 按断点分段 --> 固定帧率重采样(位置线性/姿态SLERP)
  --> HDF5(带有效性掩码 + attrs)

用法:
  python3 export_dataset.py pico.jsonl manus.jsonl -o dataset.hdf5
  python3 export_dataset.py pico.jsonl manus.jsonl -o d.hdf5 --fps 30 --hands full \
      --calib config/calib_wrist.json --gate-ms 30 --gap-ms 200
"""
from __future__ import annotations
import argparse, json, os, shutil, subprocess, sys, time
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from align_pico_manus import (                                      # noqa: E402
    TACTILE_FRAME_SCHEMA,
    TACTILE_SIDES,
    TACTILE_VALUE_COUNT,
    load_manus,
    load_tactile,
    nearest,
    _pose,
    _qpc_ns,
    _wall_ns,
    choose_alignment_clock,
)
from tactile_layout import (                                        # noqa: E402
    TACTILE_FINGER_ORDER,
    TACTILE_FINGER_REGIONS,
    TACTILE_GRID_COLS,
    TACTILE_GRID_ROWS,
    TACTILE_LAYOUT_SCHEMA,
    TACTILE_LAYOUT_SOURCE,
    TACTILE_PALM_PRESENT,
    TACTILE_PHYSICAL_ACTIVE_COUNT,
    TACTILE_PHYSICAL_FINGER_SHAPE,
    TACTILE_PHYSICAL_SLOT_COUNT,
    TACTILE_TO_MANUS_NODE_IDS,
    active_coordinates,
    finger_physical_active_mask,
    finger_region_metadata,
    validate_manus_finger_nodes,
)
from pico_retarget import (convert_lh_to_rh, apply_pico_to_robot_axes,   # noqa: E402
                           quat_mul, quat_conj,
                           quat_rotate, quat_normalize, compose_pose)

NS = 1_000_000
DEFAULT_MATCH_GATE_MS = 30.0
DEFAULT_MIN_COVERAGE = 0.99
DEFAULT_MIN_COMPLETE_COVERAGE = 0.99
DEFAULT_MAX_P95_SKEW_MS = 20.0
DEFAULT_MAX_SKEW_MS = 30.0


def load_pico2(path):
    """PICO 跟踪帧: 同时保留 传感器时钟 ts(用于分段/重采样) 与 recv_wall(用于挂MANUS)。
    ts 缺失时回退到 wall。按 ts 排序。"""
    frames = []
    with open(path, encoding="utf-8") as f:
        for i, line in enumerate(f):
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except Exception:  # noqa: BLE001
                continue
            data = rec.get("data")
            if not isinstance(data, dict):
                continue
            wall = _wall_ns(rec)
            if wall is None:
                continue
            ts = data.get("timeStampNs")
            ts = int(ts) if ts else wall
            ctrl = data.get("Controller") if isinstance(data.get("Controller"), dict) else {}
            frames.append({
                "ts": ts, "wall_ns": wall, "qpc_ns": _qpc_ns(rec), "idx": i,
                "head": _pose(data.get("Head")),
                "left_ctrl": _pose(ctrl.get("left")),
                "right_ctrl": _pose(ctrl.get("right")),
            })
    # 清洗坏时钟帧: PICO 会话初期 timeStampNs 常未同步(相对 wall 偏移异常),
    # 会污染绝对时间轴。以 (ts-wall) 中位数为基准, 丢掉偏离过大的帧。
    if frames:
        offs = np.array([f["ts"] - f["wall_ns"] for f in frames], float)
        med = float(np.median(offs))
        good = [f for f, o in zip(frames, offs) if abs(o - med) < 3e9]
        dropped = len(frames) - len(good)
        if dropped:
            print(f"[export] 清洗坏时钟帧 {dropped} (ts-wall 偏离中位>3s)", file=sys.stderr)
        frames = good
    frames.sort(key=lambda d: d["ts"])
    return frames


# ------------------------------------------------------------- 坐标转换
def pico_to_world(pose):
    """原始 PICO (pos,quat)左手系 -> 世界右手系 X前Y左Z上。"""
    return apply_pico_to_robot_axes(convert_lh_to_rh(pose))


def ctrl_to_wrist_world(pose, calib, teleop_wrist=False):
    """手柄原点世界位姿 -> 对齐到手套腕的世界位姿。

    ego 默认: 只叠 T_calib(安装平移/旋转 offset), 不做遥操 (1,0,1)180°。
    teleop_wrist=True: 兼容旧遥操管线, 先叠 Q_CTRL_TO_WRIST 再叠 T_calib。
    """
    w = pose
    if teleop_wrist:
        from pico_retarget import Q_CTRL_TO_WRIST
        w = compose_pose(w, ([0.0, 0.0, 0.0], Q_CTRL_TO_WRIST))
    if calib is not None:
        w = compose_pose(w, calib)
    return w


def effective_controller_to_wrist(calib, teleop_wrist=False):
    """返回实际应用的手柄原点系→手腕系固定变换。"""
    transform = ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
    if teleop_wrist:
        from pico_retarget import Q_CTRL_TO_WRIST
        transform = compose_pose(
            transform, ([0.0, 0.0, 0.0], Q_CTRL_TO_WRIST),
        )
    if calib is not None:
        transform = compose_pose(transform, calib)
    return transform


def controller_to_wrist_metadata(calib, teleop_wrist=False):
    """生成可独立复现 wrist_pose 的 HDF5 标定元数据。"""
    result = {
        "schema": "controller_to_wrist_v1",
        "convention": "wrist_pose = compose_pose(controller_pose, transform)",
        "controller_pose_frame": "pico_world_rh_x_forward_y_left_z_up",
        "teleop_wrist": bool(teleop_wrist),
    }
    for side in ("left", "right"):
        pos, quat = effective_controller_to_wrist(
            calib.get(side), teleop_wrist=teleop_wrist,
        )
        result[side] = {
            "pos": [float(value) for value in pos],
            "quat": [float(value) for value in quat],
        }
    return result


def controller_rows_to_wrist(controller_rows, transform):
    """逐行应用固定手柄→手腕变换，保证导出腕位姿可由手柄位姿精确复现。"""
    rows = np.asarray(controller_rows, dtype=float)
    out = np.zeros_like(rows)
    for index, row in enumerate(rows):
        pos, quat = compose_pose(
            (row[:3], row[3:]), transform,
        )
        out[index] = np.asarray(pos + quat, dtype=float)
    return out


def load_calib(path):
    """读取 {left:{pos,quat}|{matrix}, right:{...}}; 缺省单位阵。"""
    if not path:
        return {"left": None, "right": None}
    obj = json.loads(Path(path).read_text(encoding="utf-8"))
    out = {}
    for side in ("left", "right"):
        c = obj.get(side)
        if not c:
            out[side] = None
        elif "pos" in c and "quat" in c:
            out[side] = ([float(x) for x in c["pos"]], quat_normalize([float(x) for x in c["quat"]]))
        elif "matrix" in c:
            m = np.array(c["matrix"], float).reshape(4, 4)
            from pico_retarget import mat_to_quat
            out[side] = (list(m[:3, 3]), mat_to_quat([list(m[i][:3]) for i in range(3)]))
        else:
            out[side] = None
    return out


def hand_local(nodes):
    """MANUS 25x7 节点 -> (25,3) 手腕局部系坐标(相对腕根, 去掉手腕世界朝向)。"""
    a = np.asarray(nodes, float)
    wp = a[0, :3]
    wq = quat_conj(quat_normalize(list(a[0, 3:7])))
    rel = a[:, :3] - wp
    return np.array([quat_rotate(wq, list(r)) for r in rel])


# ------------------------------------------------------------- SLERP(两四元数)
def slerp_np(q0, q1, t):
    q0 = np.asarray(q0, float); q1 = np.asarray(q1, float)
    q0 /= np.linalg.norm(q0) + 1e-12
    q1 /= np.linalg.norm(q1) + 1e-12
    d = float(np.dot(q0, q1))
    if d < 0:
        q1 = -q1; d = -d
    if d > 0.9995:
        q = q0 + t * (q1 - q0)
        return q / (np.linalg.norm(q) + 1e-12)
    th = np.arccos(np.clip(d, -1, 1))
    s = np.sin(th)
    return (np.sin((1 - t) * th) / s) * q0 + (np.sin(t * th) / s) * q1


# ------------------------------------------------------------- 主流程
def build_source(pico, manus, calib, gate_ns, clock_key="wall_ns",
                 teleop_wrist=False):
    """构造源帧: 只保留 头+双手柄 有效的帧; MANUS 按最近邻挂上(可缺失)。"""
    ordered = {
        side: sorted(frames, key=lambda frame: int(frame[clock_key]))
        for side, frames in manus.items()
    }
    keys = {s: [int(d[clock_key]) for d in fr] for s, fr in ordered.items()}
    src = []
    for pf in pico:
        if pf["head"] is None or pf["left_ctrl"] is None or pf["right_ctrl"] is None:
            continue
        lw = pico_to_world(pf["left_ctrl"]); rw = pico_to_world(pf["right_ctrl"])
        # 手柄未握持(零位)跳过
        if np.dot(lw[0], lw[0]) < 1e-8 or np.dot(rw[0], rw[0]) < 1e-8:
            continue
        head = pico_to_world(pf["head"])
        lwr = ctrl_to_wrist_world(lw, calib["left"], teleop_wrist=teleop_wrist)
        rwr = ctrl_to_wrist_world(rw, calib["right"], teleop_wrist=teleop_wrist)
        rec = {"ts": pf["ts"], "wall": pf["wall_ns"],
               "qpc": int(pf.get("qpc_ns") or -1),
               "align": int(pf[clock_key]),
               "head": np.array(head[0] + head[1]),
               "lc": np.array(lw[0] + lw[1]),
               "rc": np.array(rw[0] + rw[1]),
               "lw": np.array(lwr[0] + lwr[1]),
               "rw": np.array(rwr[0] + rwr[1]),
               "lh": None, "rh": None}
        for side, tgt in (("left", "lh"), ("right", "rh")):
            fr = ordered.get(side)
            if not fr:
                continue
            m = nearest(fr, keys[side], int(pf[clock_key]), clock_key)
            if m is None:
                continue
            if gate_ns is not None and abs(
                    int(m[clock_key]) - int(pf[clock_key])) > gate_ns:
                continue
            nodes = m.get("nodes")
            if nodes:
                rec[tgt] = hand_local(nodes)
        src.append(rec)
    return src


def segment(src, gap_ns):
    """按 传感器时钟 ts 的断点分段(recv_wall 突发不可靠)。"""
    segs, cur = [], []
    for r in src:
        if cur and r["ts"] - cur[-1]["ts"] > gap_ns:
            segs.append(cur); cur = []
        cur.append(r)
    if cur:
        segs.append(cur)
    return segs


def resample_segment(seg, fps, njoint):
    """按 PICO 采样钟 ts 重采样; 同时线性插值 recv wall 供视频帧匹配。"""
    ts = np.array([r["ts"] for r in seg], float)
    walls = np.array([r["wall"] for r in seg], float)
    qpcs = np.array([r["qpc"] for r in seg], float)
    aligns = np.array([r["align"] for r in seg], float)
    if len(seg) < 2:
        return None
    dt = 1e9 / fps
    grid = np.arange(ts[0], ts[-1], dt)
    if len(grid) == 0:
        return None
    head = np.array([r["head"] for r in seg])   # (N,7)
    lc = np.array([r["lc"] for r in seg]); rc = np.array([r["rc"] for r in seg])
    lw = np.array([r["lw"] for r in seg]); rw = np.array([r["rw"] for r in seg])

    def hand_arr(key):
        a = np.full((len(seg), njoint, 3), np.nan)
        v = np.zeros(len(seg), bool)
        for i, r in enumerate(seg):
            if r[key] is not None:
                a[i] = r[key][:njoint]; v[i] = True
        return a, v
    lh, lhv = hand_arr("lh"); rh, rhv = hand_arr("rh")

    G = len(grid)
    out = {"t": grid.astype(np.int64),
           "wall": np.zeros(G, np.int64),
           "qpc": np.zeros(G, np.int64),
           "align": np.zeros(G, np.int64),
           "head": np.zeros((G, 7)),
           "lc": np.zeros((G, 7)), "rc": np.zeros((G, 7)),
           "lw": np.zeros((G, 7)), "rw": np.zeros((G, 7)),
           "lh": np.full((G, njoint, 3), np.nan), "rh": np.full((G, njoint, 3), np.nan),
           "lhv": np.zeros(G, bool), "rhv": np.zeros(G, bool)}
    idx = np.searchsorted(ts, grid, side="right") - 1
    idx = np.clip(idx, 0, len(seg) - 2)
    for gi in range(G):
        i = idx[gi]
        wa, wb = ts[i], ts[i + 1]
        a = 0.0 if wb == wa else (grid[gi] - wa) / (wb - wa)
        out["wall"][gi] = int((1 - a) * walls[i] + a * walls[i + 1])
        out["qpc"][gi] = int((1 - a) * qpcs[i] + a * qpcs[i + 1])
        out["align"][gi] = int((1 - a) * aligns[i] + a * aligns[i + 1])
        for key, arr in (("head", head), ("lc", lc), ("rc", rc),
                         ("lw", lw), ("rw", rw)):
            p = (1 - a) * arr[i, :3] + a * arr[i + 1, :3]
            q = slerp_np(arr[i, 3:], arr[i + 1, 3:], a)
            out[key][gi] = np.concatenate([p, q])
        for hkey, harr, hv, ov in (("lh", lh, lhv, "lhv"), ("rh", rh, rhv, "rhv")):
            if hv[i] and hv[i + 1]:
                out[hkey][gi] = (1 - a) * harr[i] + a * harr[i + 1]
                out[ov][gi] = True
    return out


def probe_video_frame_count(video_path):
    """Return the number of decodable video frames reported by ffprobe."""
    p = Path(video_path) if video_path else None
    ffprobe = os.environ.get("FFPROBE") or shutil.which("ffprobe")
    if not p or not p.is_file() or not ffprobe:
        return None
    try:
        proc = subprocess.run(
            [
                ffprobe, "-v", "error", "-count_frames", "-select_streams", "v:0",
                "-show_entries", "stream=nb_read_frames",
                "-of", "default=nokey=1:noprint_wrappers=1", str(p),
            ],
            check=True, capture_output=True, text=True,
        )
        value = int(proc.stdout.strip())
        return value if value > 0 else None
    except (OSError, subprocess.CalledProcessError, ValueError):
        return None


def probe_video_size(video_path):
    """Return the coded (width, height), or None when it cannot be verified."""
    p = Path(video_path) if video_path else None
    ffprobe = os.environ.get("FFPROBE") or shutil.which("ffprobe")
    if not p or not p.is_file() or not ffprobe:
        return None
    try:
        proc = subprocess.run(
            [
                ffprobe, "-v", "error", "-select_streams", "v:0",
                "-show_entries", "stream=width,height", "-of", "json", str(p),
            ],
            check=True, capture_output=True, text=True,
        )
        streams = json.loads(proc.stdout).get("streams") or []
        if len(streams) != 1:
            return None
        width = int(streams[0].get("width") or 0)
        height = int(streams[0].get("height") or 0)
        return (width, height) if width > 0 and height > 0 else None
    except (OSError, subprocess.CalledProcessError, ValueError, TypeError,
            json.JSONDecodeError):
        return None


def _normalize_video_walls(vwalls, video_frame_count):
    """Reconcile legacy packet timestamps with decoded picture frames.

    Older captures timestamped the leading SPS/PPS-only packet.  It normally
    has the same receive timestamp as the first IDR.  Remove only that safely
    identifiable prefix; reject ambiguous excess entries rather than emitting
    an out-of-range ``video_frame_idx``.
    """
    if video_frame_count is None or len(vwalls) <= video_frame_count:
        return vwalls
    extra = len(vwalls) - int(video_frame_count)
    if extra > 0 and extra < len(vwalls) and np.all(vwalls[:extra] == vwalls[extra]):
        print(
            "[export] VST sidecar 含 {} 条前导非画面配置包时间戳；已移除，"
            "解码帧数={}".format(extra, video_frame_count),
            file=sys.stderr,
        )
        return vwalls[extra:]
    raise ValueError(
        "VST sidecar 条目数 {} 超过解码帧数 {}，且无法安全识别前导配置包".format(
            len(vwalls), video_frame_count,
        )
    )


def load_video_timestamps(vst_ts_path, vst_qpc_ts_path=None,
                          video_frame_count=None):
    """读取成对的视频接收时间戳；旧数据可以只有墙钟 sidecar。"""
    wall_path = Path(vst_ts_path) if vst_ts_path else None
    if not wall_path or not wall_path.is_file():
        return {"wall_ns": np.empty(0, np.int64),
                "qpc_ns": np.empty(0, np.int64)}
    walls = np.asarray(
        [int(x) for x in wall_path.read_text().splitlines() if x.strip()],
        np.int64,
    )
    walls = _normalize_video_walls(walls, video_frame_count)
    qpcs = np.empty(0, np.int64)
    qpc_path = Path(vst_qpc_ts_path) if vst_qpc_ts_path else None
    if qpc_path and qpc_path.is_file():
        qpcs = np.asarray(
            [int(x) for x in qpc_path.read_text().splitlines() if x.strip()],
            np.int64,
        )
        qpcs = _normalize_video_walls(qpcs, video_frame_count)
        if len(qpcs) != len(walls):
            raise ValueError(
                "VST QPC/墙钟 sidecar 条目数不同: {}/{}".format(
                    len(qpcs), len(walls)
                )
            )
    return {"wall_ns": walls, "qpc_ns": qpcs}


def match_video_details(target_ns, video_times, clock_key="wall_ns",
                        gate_ms=DEFAULT_MATCH_GATE_MS):
    """视频最近邻匹配，同时返回源时间戳和有符号偏差。"""
    target_ns = np.asarray(target_ns, dtype=np.int64)
    T = len(target_ns)
    result = {
        "frame_idx": np.full(T, -1, np.int32),
        "valid": np.zeros(T, bool),
        "recv_wall_ns": np.full(T, -1, np.int64),
        "recv_qpc_ns": np.full(T, -1, np.int64),
        "offset_ms": np.full(T, np.nan, np.float32),
    }
    source = np.asarray(video_times.get(clock_key, []), dtype=np.int64)
    if not len(source):
        return result
    walls = np.asarray(video_times.get("wall_ns", []), dtype=np.int64)
    qpcs = np.asarray(video_times.get("qpc_ns", []), dtype=np.int64)
    gate = int(gate_ms * NS) if gate_ms and gate_ms > 0 else None
    indices = np.searchsorted(source, target_ns)
    for i, target in enumerate(target_ns):
        j = int(indices[i])
        candidates = [k for k in (j - 1, j) if 0 <= k < len(source)]
        if not candidates:
            continue
        best = min(candidates, key=lambda k: abs(int(source[k]) - int(target)))
        offset_ns = int(source[best]) - int(target)
        if gate is not None and abs(offset_ns) > gate:
            continue
        result["frame_idx"][i] = best
        result["valid"][i] = True
        result["recv_wall_ns"][i] = int(walls[best]) if len(walls) else -1
        result["recv_qpc_ns"][i] = int(qpcs[best]) if len(qpcs) else -1
        result["offset_ms"][i] = offset_ns / NS
    return result


def match_video_frames(walls_ns, vst_ts_path, gate_ms=DEFAULT_MATCH_GATE_MS,
                       video_frame_count=None):
    """walls_ns (T,) → video_frame_idx (T,); 超时或无文件填 -1。"""
    p = Path(vst_ts_path) if vst_ts_path else None
    T = len(walls_ns)
    out = np.full(T, -1, np.int32)
    if not p or not p.is_file():
        return out
    vwalls = np.asarray([int(x) for x in p.read_text().splitlines() if x.strip()], np.int64)
    if len(vwalls) == 0:
        return out
    vwalls = _normalize_video_walls(vwalls, video_frame_count)
    gate = int(gate_ms * 1e6)
    # searchsorted 最近邻
    idx = np.searchsorted(vwalls, walls_ns)
    for i, w in enumerate(walls_ns):
        j = int(idx[i])
        cands = []
        if 0 <= j < len(vwalls):
            cands.append(j)
        if j - 1 >= 0:
            cands.append(j - 1)
        if not cands:
            continue
        best = min(cands, key=lambda k: abs(int(vwalls[k]) - int(w)))
        if abs(int(vwalls[best]) - int(w)) <= gate:
            out[i] = best
    return out


def match_manus_frames(target_ns, manus, clock_key="wall_ns",
                       gate_ms=DEFAULT_MATCH_GATE_MS):
    """为每个导出行记录最近的 MANUS 原始源帧及其偏差。"""
    result = {}
    gate_ns = int(gate_ms * NS) if gate_ms and gate_ms > 0 else None
    for side in ("left", "right"):
        frames = sorted(
            manus.get(side) or [], key=lambda frame: int(frame[clock_key])
        )
        T = len(target_ns)
        valid = np.zeros(T, dtype=bool)
        source_wall = np.full(T, -1, dtype=np.int64)
        source_qpc = np.full(T, -1, dtype=np.int64)
        offset_ms = np.full(T, np.nan, dtype=np.float32)
        if frames:
            keys = [int(frame[clock_key]) for frame in frames]
            for i, target in enumerate(target_ns):
                frame = nearest(frames, keys, int(target), clock_key)
                if frame is None:
                    continue
                offset_ns = int(frame[clock_key]) - int(target)
                if gate_ns is not None and abs(offset_ns) > gate_ns:
                    continue
                valid[i] = True
                source_wall[i] = int(frame["wall_ns"])
                source_qpc[i] = int(frame.get("qpc_ns") or -1)
                offset_ms[i] = offset_ns / NS
        result[side] = {
            "valid": valid,
            "recv_wall_ns": source_wall,
            "recv_qpc_ns": source_qpc,
            "offset_ms": offset_ms,
        }
    return result


def match_tactile_frames(target_ns, tactile,
                         gate_ms=DEFAULT_MATCH_GATE_MS,
                         clock_key="wall_ns"):
    """把双手原始触觉帧最近邻挂到导出主机时钟；值不插值。"""
    result = {}
    gate_ns = int(gate_ms * NS) if gate_ms and gate_ms > 0 else None
    for side in TACTILE_SIDES:
        frames = sorted(
            tactile.get(side) or [], key=lambda frame: int(frame[clock_key])
        )
        T = len(target_ns)
        values = np.zeros((T, TACTILE_VALUE_COUNT), dtype=np.int16)
        valid = np.zeros(T, dtype=bool)
        source_wall = np.full(T, -1, dtype=np.int64)
        source_qpc = np.full(T, -1, dtype=np.int64)
        stream_seq = np.full(T, -1, dtype=np.int64)
        record_seq = np.full(T, -1, dtype=np.int64)
        offset_ms = np.full(T, np.nan, dtype=np.float32)
        if frames:
            keys = [int(frame[clock_key]) for frame in frames]
            for i, target in enumerate(target_ns):
                frame = nearest(frames, keys, int(target), clock_key)
                if frame is None:
                    continue
                offset_ns = int(frame[clock_key]) - int(target)
                if gate_ns is not None and abs(offset_ns) > gate_ns:
                    continue
                values[i] = np.asarray(frame["wire_values"], dtype=np.int16)
                valid[i] = True
                source_wall[i] = int(frame["wall_ns"])
                source_qpc[i] = int(frame.get("qpc_ns") or -1)
                stream_seq[i] = int(frame["stream_seq"])
                record_seq[i] = int(frame["record_seq"])
                offset_ms[i] = offset_ns / NS
        result[side] = {
            "values": values,
            "valid": valid,
            "recv_wall_ns": source_wall,
            "recv_qpc_ns": source_qpc,
            "stream_seq": stream_seq,
            "record_seq": record_seq,
            "offset_ms": offset_ms,
        }
    return result


def tactile_spatial_views(aligned_side, wire_layout):
    """369 wire值 → 24x16 → 实物五指阵列 (5x4x8)。

    只抽取这副 HS13 确实安装的五片上方阵列；上位机里的第二片指部阵列和
    手掌区域不作为物理传感器导出。每块 wire 8x4 转置为物理 4x8，四个
    非拇指最后一列按硬件事实置零并在 active mask 中标为无效。
    """
    values = aligned_side["values"]
    if values.ndim != 2 or values.shape[1] != TACTILE_VALUE_COUNT:
        raise ValueError("触觉 aligned values 形状不是 (T,{})".format(
            TACTILE_VALUE_COUNT))
    coordinates = active_coordinates(wire_layout)
    rows = np.asarray([item[0] for item in coordinates], dtype=np.intp)
    cols = np.asarray([item[1] for item in coordinates], dtype=np.intp)

    matrix = np.zeros(
        (values.shape[0], TACTILE_GRID_ROWS, TACTILE_GRID_COLS),
        dtype=np.int16,
    )
    matrix[:, rows, cols] = values
    active_mask = np.zeros((TACTILE_GRID_ROWS, TACTILE_GRID_COLS), dtype=bool)
    active_mask[rows, cols] = True

    physical_mask = np.asarray(finger_physical_active_mask(), dtype=bool)
    fingers = []
    finger_masks = []
    for finger in TACTILE_FINGER_ORDER:
        r0, r1, c0, c1 = TACTILE_FINGER_REGIONS[finger]
        block = matrix[:, r0:r1, c0:c1].transpose(0, 2, 1)
        block_wire_mask = active_mask[r0:r1, c0:c1].T
        finger_index = len(fingers)
        combined_mask = block_wire_mask & physical_mask[finger_index]
        fingers.append(np.where(combined_mask[None, :, :], block, 0))
        finger_masks.append(combined_mask)
    return {
        "fingers": np.stack(fingers, axis=1),
        "fingers_active_mask": np.stack(finger_masks, axis=0),
    }


def common_valid_interval(pico, manus, tactile, video_times, clock_key,
                          require_video=False):
    """计算所有已请求数据路的公共主机时钟区间。"""
    groups = [("PICO", pico)]
    for side in ("left", "right"):
        groups.append((f"MANUS {side}", manus.get(side) or []))
    if tactile is not None:
        for side in TACTILE_SIDES:
            groups.append((f"TACTILE {side}", tactile.get(side) or []))
    bounds = []
    for name, frames in groups:
        values = [int(frame[clock_key]) for frame in frames
                  if frame.get(clock_key) is not None]
        if not values:
            raise ValueError(f"{name} 在 {clock_key} 上没有有效时间戳")
        bounds.append((name, min(values), max(values)))
    if require_video:
        values = np.asarray(video_times.get(clock_key, []), dtype=np.int64)
        if not len(values):
            raise ValueError(f"VST 在 {clock_key} 上没有有效时间戳")
        bounds.append(("VST", int(values.min()), int(values.max())))
    start = max(item[1] for item in bounds)
    end = min(item[2] for item in bounds)
    if end <= start:
        detail = ", ".join(
            f"{name}=[{first},{last}]" for name, first, last in bounds
        )
        raise ValueError("各数据路没有公共有效区间: " + detail)
    return start, end, bounds


def _alignment_metrics(valid, offset_ms):
    valid = np.asarray(valid, dtype=bool)
    values = np.abs(np.asarray(offset_ms, dtype=float)[valid])
    metrics = {
        "coverage": float(valid.mean()) if len(valid) else 0.0,
        "count": int(valid.sum()),
        "p95_ms": float(np.percentile(values, 95)) if len(values) else float("inf"),
        "max_ms": float(values.max()) if len(values) else float("inf"),
    }
    return metrics


def _check_quality(name, metrics, min_coverage, max_p95_ms,
                   max_offset_ms, errors):
    if metrics["coverage"] < min_coverage:
        errors.append(
            f"{name}覆盖率 {metrics['coverage'] * 100:.2f}% < "
            f"{min_coverage * 100:.2f}%"
        )
    if metrics["p95_ms"] > max_p95_ms:
        errors.append(
            f"{name} |offset| p95={metrics['p95_ms']:.3f}ms > {max_p95_ms:.3f}ms"
        )
    if metrics["max_ms"] > max_offset_ms:
        errors.append(
            f"{name} |offset| max={metrics['max_ms']:.3f}ms > {max_offset_ms:.3f}ms"
        )


def complete_frame_mask(hand_valid, video_valid, tactile_aligned=None,
                        require_video=False):
    """返回所有已请求传感器都存在有效匹配的训练帧掩码。"""
    mask = (np.asarray(hand_valid["left"], dtype=bool)
            & np.asarray(hand_valid["right"], dtype=bool))
    if require_video:
        mask &= np.asarray(video_valid, dtype=bool)
    if tactile_aligned is not None:
        for side in TACTILE_SIDES:
            mask &= np.asarray(tactile_aligned[side]["valid"], dtype=bool)
    return mask


def resegment_complete_rows(base_segment_ids, kept_source_rows):
    """删除不完整帧后在原片段变化或行号缺口处重新编号。"""
    base = np.asarray(base_segment_ids)
    kept = np.asarray(kept_source_rows, dtype=np.int64)
    if not len(kept):
        return np.empty(0, dtype=np.int32)
    out = np.zeros(len(kept), dtype=np.int32)
    segment_id = 0
    for index in range(1, len(kept)):
        if (base[kept[index]] != base[kept[index - 1]]
                or kept[index] != kept[index - 1] + 1):
            segment_id += 1
        out[index] = segment_id
    return out


def main():
    ap = argparse.ArgumentParser(
        description="导出训练包 HDF5 egodex_v1（位姿+手指+可选视频帧号/双手触觉）"
    )
    ap.add_argument("pico"); ap.add_argument("manus")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--hands", choices=["full", "tips"], default="full",
                    help="full=25关节, tips=5指尖")
    ap.add_argument("--calib", default=None,
                    help="外参 json: 手柄原点系->手套腕系 T_calib (缺省单位阵; 主要调 pos 平移)")
    ap.add_argument("--teleop-wrist", action="store_true",
                    help="兼容旧遥操: 叠 Q_CTRL_TO_WRIST(180°); ego 默认不要开")
    ap.add_argument("--gate-ms", type=float, default=DEFAULT_MATCH_GATE_MS,
                    help="MANUS 最近邻门限（默认 30ms）")
    ap.add_argument("--gap-ms", type=float, default=200.0, help="超此间隔视为断点分段")
    ap.add_argument("--vst", default=None,
                    help="视频路径（项目 VST H264 或 PICO 连续录制 MP4；写入 attrs.video_path）")
    ap.add_argument("--vst-ts", default=None,
                    help="逐帧墙钟 sidecar；提供则写入 video_frame_idx（按 wall 最近邻）")
    ap.add_argument("--vst-qpc-ts", default=None,
                    help="vst.qpc.ts.jsonl；新采集优先用高精度 QPC 对齐")
    ap.add_argument("--video-gate-ms", type=float, default=DEFAULT_MATCH_GATE_MS,
                    help="视频帧匹配门限（默认 30ms）")
    ap.add_argument("--video-size", default="auto",
                    help="SBS 整幅宽x高；默认 auto，从实际码流读取")
    ap.add_argument("--video-cam", default="config/pico_cam/vst_cam.json",
                    help="左右眼内参/外参 json(写入 attrs, 供双目深度)")
    ap.add_argument("--video-eye", default="both", choices=["both", "left", "right"],
                    help="声明训练主用眼; both=双目(默认), 左右 crop 均写入 attrs")
    ap.add_argument("--tactile", default=None,
                    help="同一任务 raw/tactile.jsonl（旧 tactile_raw 也兼容）")
    ap.add_argument("--tactile-meta", default=None,
                    help="触觉封存元数据；缺省自动取 tactile.jsonl 旁的 tactile.meta.json")
    ap.add_argument("--tactile-gate-ms", type=float, default=DEFAULT_MATCH_GATE_MS,
                    help="触觉最近邻门限；0=不门限（默认 30ms）")
    ap.add_argument("--min-hand-coverage", type=float, default=DEFAULT_MIN_COVERAGE,
                    help="每侧 MANUS 最小覆盖率（默认 0.99）")
    ap.add_argument("--min-video-coverage", type=float, default=DEFAULT_MIN_COVERAGE,
                    help="提供 VST 时最小覆盖率（默认 0.99）")
    ap.add_argument("--min-tactile-coverage", type=float, default=DEFAULT_MIN_COVERAGE,
                    help="提供触觉时每侧最小覆盖率（默认 0.99）")
    ap.add_argument("--min-complete-coverage", type=float,
                    default=DEFAULT_MIN_COMPLETE_COVERAGE,
                    help="所有已请求传感器同时有效的最小帧比例（默认 0.99）")
    ap.add_argument("--max-p95-skew-ms", type=float, default=DEFAULT_MAX_P95_SKEW_MS,
                    help="各路 |时间偏差| 的 p95 上限（默认 20ms）")
    ap.add_argument("--max-skew-ms", type=float, default=DEFAULT_MAX_SKEW_MS,
                    help="各路单帧 |时间偏差| 上限（默认 30ms）")
    args = ap.parse_args()
    if args.tactile_meta and not args.tactile:
        ap.error("--tactile-meta 必须与 --tactile 一起使用")
    for option, value in (
        ("--min-hand-coverage", args.min_hand_coverage),
        ("--min-video-coverage", args.min_video_coverage),
        ("--min-tactile-coverage", args.min_tactile_coverage),
        ("--min-complete-coverage", args.min_complete_coverage),
    ):
        if not 0.0 <= value <= 1.0:
            ap.error(option + " 必须在 0..1")
    if args.max_p95_skew_ms <= 0 or args.max_skew_ms <= 0:
        ap.error("时间偏差质检阈值必须为正数")

    pico = load_pico2(Path(args.pico))
    manus = load_manus(Path(args.manus))
    if not pico or not manus:
        print("[export] 输入为空。", file=sys.stderr); sys.exit(1)
    tactile = None
    tactile_meta = None
    if args.tactile:
        try:
            tactile, tactile_meta = load_tactile(
                Path(args.tactile),
                Path(args.tactile_meta) if args.tactile_meta else None,
            )
        except Exception as exc:  # noqa: BLE001
            print("[export] {}".format(exc), file=sys.stderr)
            sys.exit(1)

    video_frame_count = probe_video_frame_count(args.vst)
    if args.vst and args.vst_ts and video_frame_count is None:
        print(
            "[export] 无法通过 ffprobe 获取 VST 可解码帧数；拒绝生成未经越界校验的 "
            "video_frame_idx。请确认 ffprobe 在 PATH，或设置 FFPROBE。",
            file=sys.stderr,
        )
        sys.exit(1)
    vst_qpc_ts = args.vst_qpc_ts
    if not vst_qpc_ts and args.vst_ts:
        wall_sidecar = Path(args.vst_ts)
        if wall_sidecar.name.endswith(".ts.jsonl"):
            candidate = wall_sidecar.with_name(
                wall_sidecar.name[:-len(".ts.jsonl")] + ".qpc.ts.jsonl"
            )
            if candidate.is_file():
                vst_qpc_ts = str(candidate)
    try:
        video_times = load_video_timestamps(
            args.vst_ts, vst_qpc_ts, video_frame_count=video_frame_count,
        )
    except ValueError as exc:
        print("[export] {}".format(exc), file=sys.stderr)
        sys.exit(1)

    clock_key = choose_alignment_clock(pico, manus, tactile)
    require_video = bool(args.vst or args.vst_ts)
    if (clock_key == "qpc_ns" and require_video
            and not len(video_times["qpc_ns"])):
        clock_key = "wall_ns"
    alignment_clock = "host_qpc" if clock_key == "qpc_ns" else "host_wall_legacy"
    try:
        common_start, common_end, common_bounds = common_valid_interval(
            pico, manus, tactile, video_times, clock_key,
            require_video=require_video,
        )
    except ValueError as exc:
        print("[export] {}".format(exc), file=sys.stderr)
        sys.exit(1)
    original_pico_count = len(pico)
    pico = [frame for frame in pico
            if common_start <= int(frame[clock_key]) <= common_end]
    if not pico:
        print("[export] 公共有效区间内没有 PICO 帧。", file=sys.stderr)
        sys.exit(1)
    print(
        "[export] 对齐时钟={}；公共有效区间 {:.3f}s；PICO 保留 {}/{} 帧".format(
            alignment_clock, (common_end - common_start) / 1e9,
            len(pico), original_pico_count,
        )
    )
    calib = load_calib(args.calib)
    wrist_calibration = controller_to_wrist_metadata(
        calib, teleop_wrist=args.teleop_wrist,
    )
    gate_ns = int(args.gate_ms * NS) if args.gate_ms > 0 else None
    gap_ns = int(args.gap_ms * NS)

    # MANUS 关节元数据(取首个有效帧)
    chain = joint = None
    for fr in manus.values():
        for m in fr:
            if m.get("chain_types") and m.get("joint_types"):
                chain, joint = m["chain_types"], m["joint_types"]; break
        if chain:
            break
    njoint_full = len(chain) if chain else 25
    if tactile is not None:
        if not chain:
            print("[export] 触觉手指对应要求 MANUS chain_types。", file=sys.stderr)
            sys.exit(1)
        try:
            validate_manus_finger_nodes(chain)
        except ValueError as exc:
            print("[export] 无法建立触觉↔MANUS手指对应: {}".format(exc),
                  file=sys.stderr)
            sys.exit(1)

    # 需要全节点: 若原始 manus 未带 nodes 则报错
    if not any(m.get("nodes") for fr in manus.values() for m in fr[:1]):
        print("[export] 警告: MANUS 帧不含 nodes 字段, 无法导出关节。", file=sys.stderr)

    src = build_source(
        pico, manus, calib, gate_ns, clock_key=clock_key,
        teleop_wrist=args.teleop_wrist,
    )
    if not src:
        print("[export] 无有效源帧(头+双手柄)。", file=sys.stderr); sys.exit(1)
    segs = segment(src, gap_ns)

    if args.hands == "tips":
        tip_idx = [i for i, j in enumerate(joint or []) if str(j).upper() == "TIP"]
        selected_manus_node_ids = list(tip_idx)
        njoint = len(tip_idx)
        joint_names = [f"{chain[i]}_{joint[i]}" for i in tip_idx]
    else:
        tip_idx = None
        selected_manus_node_ids = list(range(njoint_full))
        njoint = njoint_full
        joint_names = [f"{chain[i]}_{joint[i]}" for i in range(njoint_full)] if chain else \
                      [f"j{i}" for i in range(njoint)]

    parts = []
    seg_ids = []
    for sid, seg in enumerate(segs):
        r = resample_segment(seg, args.fps, njoint_full)
        if r is None:
            continue
        if tip_idx is not None:
            r["lh"] = r["lh"][:, tip_idx, :]; r["rh"] = r["rh"][:, tip_idx, :]
        parts.append(r)
        seg_ids.append(np.full(len(r["t"]), sid, np.int32))
    if not parts:
        print("[export] 分段重采样后为空。", file=sys.stderr); sys.exit(1)

    def cat(k):
        return np.concatenate([p[k] for p in parts], axis=0)

    import h5py
    T = sum(len(p["t"]) for p in parts)
    walls = cat("wall")
    qpcs = cat("qpc")
    target_ns = cat("align")
    video_aligned = match_video_details(
        target_ns, video_times, clock_key=clock_key,
        gate_ms=args.video_gate_ms,
    )
    vidx = video_aligned["frame_idx"]
    manus_aligned = match_manus_frames(
        target_ns, manus, clock_key=clock_key, gate_ms=args.gate_ms,
    )
    hand_valid = {
        "left": cat("lhv") & manus_aligned["left"]["valid"],
        "right": cat("rhv") & manus_aligned["right"]["valid"],
    }
    tactile_aligned = None
    tactile_spatial = None
    tactile_coverage = {}
    if tactile is not None:
        tactile_aligned = match_tactile_frames(
            target_ns, tactile, gate_ms=args.tactile_gate_ms,
            clock_key=clock_key,
        )
        for side in TACTILE_SIDES:
            coverage = float(tactile_aligned[side]["valid"].mean()) if T else 0.0
            tactile_coverage[side] = coverage
        tactile_spatial = {}
        assert tactile_meta is not None
        try:
            for side in TACTILE_SIDES:
                tactile_spatial[side] = tactile_spatial_views(
                    tactile_aligned[side],
                    tactile_meta["streams"][side]["wire_layout"],
                )
        except (KeyError, TypeError, ValueError) as exc:
            print("[export] 触觉五指区域展开失败: {}".format(exc), file=sys.stderr)
            sys.exit(1)

    quality = {
        "manus_left": _alignment_metrics(
            hand_valid["left"], manus_aligned["left"]["offset_ms"]),
        "manus_right": _alignment_metrics(
            hand_valid["right"], manus_aligned["right"]["offset_ms"]),
    }
    quality_errors = []
    for side in ("left", "right"):
        _check_quality(
            f"MANUS {side}", quality[f"manus_{side}"],
            args.min_hand_coverage, args.max_p95_skew_ms,
            args.max_skew_ms, quality_errors,
        )
    if require_video:
        quality["video"] = _alignment_metrics(
            video_aligned["valid"], video_aligned["offset_ms"])
        _check_quality(
            "VST", quality["video"], args.min_video_coverage,
            args.max_p95_skew_ms, args.max_skew_ms, quality_errors,
        )
    if tactile_aligned is not None:
        for side in TACTILE_SIDES:
            quality[f"tactile_{side}"] = _alignment_metrics(
                tactile_aligned[side]["valid"],
                tactile_aligned[side]["offset_ms"],
            )
            _check_quality(
                f"TACTILE {side}", quality[f"tactile_{side}"],
                args.min_tactile_coverage, args.max_p95_skew_ms,
                args.max_skew_ms, quality_errors,
            )

    complete_mask = complete_frame_mask(
        hand_valid, video_aligned["valid"], tactile_aligned,
        require_video=require_video,
    )
    complete_required_routes = [
        "pico_head", "pico_controller_left", "pico_controller_right",
        "manus_left", "manus_right",
    ]
    if require_video:
        complete_required_routes.append("vst_stereo_frame")
    if tactile_aligned is not None:
        complete_required_routes.extend(["tactile_left", "tactile_right"])
    complete_count = int(complete_mask.sum())
    complete_coverage = float(complete_mask.mean()) if T else 0.0
    complete_quality = {
        "coverage": complete_coverage,
        "count": complete_count,
        "total": int(T),
        "dropped": int(T - complete_count),
    }
    if complete_coverage < args.min_complete_coverage:
        quality_errors.append(
            "完整帧覆盖率 {:.2f}% < {:.2f}%（要求 MANUS双手/VST/触觉双手同时有效）".format(
                complete_coverage * 100.0,
                args.min_complete_coverage * 100.0,
            )
        )
    if quality_errors:
        print("[export] 严格时间对齐质检失败：", file=sys.stderr)
        for message in quality_errors:
            print("  - " + message, file=sys.stderr)
        sys.exit(1)

    # HDF5 只写所有已请求传感器同时有效的行。任何被剔除的目标行都形成时间缺口，
    # 重新切段，防止下游把缺口两侧误当作连续 30 Hz 序列。
    pre_filter_T = int(T)
    kept_source_rows = np.flatnonzero(complete_mask).astype(np.int64)
    if not len(kept_source_rows):
        print("[export] 没有所有传感器同时有效的完整帧。", file=sys.stderr)
        sys.exit(1)
    base_segment_ids = np.concatenate(seg_ids)
    export_segment_ids = resegment_complete_rows(
        base_segment_ids, kept_source_rows,
    )
    frame_time_ns = cat("t")[complete_mask]
    walls = walls[complete_mask]
    qpcs = qpcs[complete_mask]
    target_ns = target_ns[complete_mask]
    head_pose = cat("head")[complete_mask]
    left_controller_pose = cat("lc")[complete_mask]
    right_controller_pose = cat("rc")[complete_mask]
    left_wrist_pose = controller_rows_to_wrist(
        left_controller_pose,
        effective_controller_to_wrist(
            calib["left"], teleop_wrist=args.teleop_wrist,
        ),
    )
    right_wrist_pose = controller_rows_to_wrist(
        right_controller_pose,
        effective_controller_to_wrist(
            calib["right"], teleop_wrist=args.teleop_wrist,
        ),
    )
    left_hand_joints = cat("lh")[complete_mask]
    right_hand_joints = cat("rh")[complete_mask]

    video_aligned = {
        key: value[complete_mask] for key, value in video_aligned.items()
    }
    vidx = video_aligned["frame_idx"]
    hand_valid = {
        side: value[complete_mask] for side, value in hand_valid.items()
    }
    for side in ("left", "right"):
        manus_aligned[side] = {
            key: value[complete_mask]
            for key, value in manus_aligned[side].items()
        }
    if tactile_aligned is not None:
        assert tactile_spatial is not None
        for side in TACTILE_SIDES:
            tactile_aligned[side] = {
                key: value[complete_mask]
                for key, value in tactile_aligned[side].items()
            }
            tactile_spatial[side]["fingers"] = \
                tactile_spatial[side]["fingers"][complete_mask]
    T = complete_count

    # 完整帧导出后有效性列必须全部为 True；保留这些列供下游通用读取器兼容。
    completeness_checks = [hand_valid["left"], hand_valid["right"]]
    if require_video:
        completeness_checks.append(video_aligned["valid"])
    if tactile_aligned is not None:
        completeness_checks.extend(
            tactile_aligned[side]["valid"] for side in TACTILE_SIDES
        )
    if not all(np.asarray(values, dtype=bool).all()
               for values in completeness_checks):
        raise RuntimeError("完整帧筛选内部错误：导出行仍含无效传感器")

    if args.video_size.lower() == "auto":
        probed_size = probe_video_size(args.vst)
        if args.vst and probed_size is None:
            raise RuntimeError(f"无法从 VST 码流确认分辨率: {args.vst}")
        # No-video exports retain the daily target only as descriptive metadata.
        vw, vh = probed_size or (4096, 1536)
    else:
        try:
            vw, vh = (int(x) for x in args.video_size.lower().split("x"))
        except ValueError:
            print("[export] --video-size 应为 auto 或 宽x高, 如 4096x1536",
                  file=sys.stderr)
            sys.exit(1)
    if vw <= 0 or vh <= 0 or vw % 2:
        raise RuntimeError(f"无效 SBS 分辨率: {vw}x{vh}（整幅宽必须为正偶数）")
    eye_w, eye_h = vw // 2, vh
    # SBS: 左 | 右  （与 overlay_skeleton / 设备约定一致）
    crop_left = {"x": 0, "y": 0, "w": eye_w, "h": eye_h}
    crop_right = {"x": eye_w, "y": 0, "w": eye_w, "h": eye_h}

    cam_blob = None
    cam_path = Path(args.video_cam)
    if cam_path.is_file():
        try:
            cam_blob = json.loads(cam_path.read_text(encoding="utf-8"))
        except Exception as e:  # noqa: BLE001
            print(f"[export] 警告: 读 {cam_path} 失败: {e}", file=sys.stderr)

    manus_capture_meta = None
    manus_meta_path = Path(args.manus).with_name("manus.meta.json")
    if manus_meta_path.is_file():
        try:
            manus_capture_meta = json.loads(
                manus_meta_path.read_text(encoding="utf-8-sig")
            )
        except (OSError, ValueError) as exc:
            raise RuntimeError(f"MANUS 标定证据损坏: {manus_meta_path}: {exc}") from exc
        calibration_evidence = manus_capture_meta.get("calibration") or {}
        required_sides = ("left", "right") if args.hands == "both" else (args.hands,)
        failed_sides = [
            side for side in required_sides
            if (calibration_evidence.get(side) or {}).get("sdk_applied") is not True
        ]
        if failed_sides:
            raise RuntimeError(
                "MANUS 个人手型标定没有成功加载，拒绝导出: "
                + ",".join(failed_sides)
            )

    manus_export_indices = None
    manus_node_id_table = None
    if tactile_aligned is not None:
        export_index_by_node = {
            int(node_id): export_index
            for export_index, node_id in enumerate(selected_manus_node_ids)
        }
        manus_export_indices = {
            finger: [
                export_index_by_node[node_id]
                for node_id in TACTILE_TO_MANUS_NODE_IDS[finger]
                if node_id in export_index_by_node
            ]
            for finger in TACTILE_FINGER_ORDER
        }
        manus_node_id_table = np.full(
            (len(TACTILE_FINGER_ORDER), 5), -1, dtype=np.int16,
        )
        for finger_index, finger in enumerate(TACTILE_FINGER_ORDER):
            node_ids = TACTILE_TO_MANUS_NODE_IDS[finger]
            manus_node_id_table[finger_index, :len(node_ids)] = node_ids

    output_path = Path(args.out)
    partial_output = output_path.with_name(output_path.name + ".partial")
    partial_output.unlink(missing_ok=True)
    with h5py.File(partial_output, "w") as f:
        f.attrs["schema"] = "egodex_v1"
        f.attrs["schema_alias"] = "egodex_B_v1"  # 旧名兼容
        base_revision = (
            "egodex_v1+sync_v2+tactile_v3"
            if tactile_aligned is not None else "egodex_v1+sync_v2"
        )
        f.attrs["schema_revision"] = base_revision + "+complete_frames_v4"
        f.attrs["alignment_clock"] = alignment_clock
        f.attrs["sync_method"] = (
            "single host clock; common valid interval; native-rate nearest/interpolation; "
            "then fixed-rate PICO sensor timeline; incomplete rows omitted and resegmented"
        )
        f.attrs["common_interval_start_ns"] = common_start
        f.attrs["common_interval_end_ns"] = common_end
        f.attrs["common_interval_sources"] = json.dumps(
            common_bounds, ensure_ascii=False,
        )
        f.attrs["alignment_quality"] = json.dumps(quality, ensure_ascii=False)
        f.attrs["complete_frame_quality"] = json.dumps(
            complete_quality, ensure_ascii=False,
        )
        f.attrs["min_hand_coverage"] = args.min_hand_coverage
        f.attrs["min_video_coverage"] = args.min_video_coverage
        f.attrs["min_tactile_coverage"] = args.min_tactile_coverage
        f.attrs["min_complete_coverage"] = args.min_complete_coverage
        f.attrs["max_p95_skew_ms"] = args.max_p95_skew_ms
        f.attrs["max_skew_ms"] = args.max_skew_ms
        f.attrs["all_exported_frames_complete"] = True
        f.attrs["complete_frame_policy"] = (
            "all requested sensor routes valid within nearest-neighbor gates; "
            "incomplete target rows omitted; every omission starts a new segment"
        )
        f.attrs["complete_frame_required_routes"] = json.dumps(
            complete_required_routes, ensure_ascii=False,
        )
        f.attrs["pre_filter_frame_count"] = pre_filter_T
        f.attrs["complete_frame_count"] = T
        f.attrs["dropped_incomplete_frame_count"] = pre_filter_T - T
        f.attrs["complete_frame_coverage"] = complete_coverage
        f.attrs["coord"] = "right-handed X-forward Y-left Z-up, meters (REP-103)"
        f.attrs["fps"] = args.fps
        f.attrs["hands"] = args.hands
        f.attrs["n_joint"] = njoint
        f.attrs["joint_names"] = json.dumps(joint_names, ensure_ascii=False)
        f.attrs["calib"] = "identity" if not args.calib else Path(args.calib).name
        f.attrs["controller_to_wrist_calibration"] = json.dumps(
            wrist_calibration, ensure_ascii=False,
        )
        f.attrs["gate_ms"] = args.gate_ms
        f.attrs["video_gate_ms"] = args.video_gate_ms
        f.attrs["gap_ms"] = args.gap_ms
        f.attrs["n_segments"] = int(export_segment_ids[-1]) + 1
        f.attrs["created"] = time.strftime("%Y-%m-%d %H:%M:%S")
        f.attrs["source_pico"] = str(Path(args.pico))
        f.attrs["source_manus"] = str(Path(args.manus))
        f.attrs["source_manus_meta"] = (
            str(manus_meta_path) if manus_capture_meta is not None else ""
        )
        f.attrs["manus_calibration"] = json.dumps(
            (manus_capture_meta or {}).get("calibration", {}), ensure_ascii=False,
        )
        f.attrs["source_tactile"] = str(Path(args.tactile)) if args.tactile else ""
        f.attrs["source_tactile_meta"] = (
            str(Path(args.tactile_meta)) if args.tactile_meta
            else (str(Path(args.tactile).with_name("tactile.meta.json"))
                  if args.tactile else "")
        )
        f.attrs["tactile_included"] = tactile_aligned is not None
        if tactile_aligned is not None and tactile_meta is not None:
            f.attrs["tactile_schema"] = TACTILE_FRAME_SCHEMA
            f.attrs["tactile_value_count"] = TACTILE_VALUE_COUNT
            f.attrs["tactile_values_semantics"] = tactile_meta["values_semantics"]
            f.attrs["tactile_alignment"] = (
                f"nearest neighbor by {clock_key}; raw int16 values; no interpolation"
            )
            f.attrs["tactile_gate_ms"] = args.tactile_gate_ms
            f.attrs["tactile_export_rate_hz"] = args.fps
            tactile_summary = tactile_meta.get("summary") or {}
            tactile_warnings = tactile_summary.get("transient_health_warnings") or {}
            f.attrs["tactile_quality_status"] = tactile_summary.get(
                "quality_status", "ok"
            )
            f.attrs["tactile_transient_warning_count"] = int(
                tactile_warnings.get("incident_count") or 0
            )
            f.attrs["tactile_transient_health_warnings"] = json.dumps(
                tactile_warnings, ensure_ascii=False,
            )
            requested_rate = tactile_meta.get("requested_rate_hz_per_side")
            if requested_rate is not None:
                f.attrs["tactile_source_requested_rate_hz_per_side"] = float(requested_rate)
            f.attrs["tactile_finger_mapping_schema"] = TACTILE_LAYOUT_SCHEMA
            f.attrs["tactile_finger_mapping_source"] = TACTILE_LAYOUT_SOURCE
            f.attrs["tactile_finger_mapping"] = json.dumps(
                finger_region_metadata(), ensure_ascii=False,
            )
            f.attrs["tactile_finger_order"] = json.dumps(
                list(TACTILE_FINGER_ORDER), ensure_ascii=False,
            )
            f.attrs["tactile_physical_finger_array_shape"] = json.dumps(
                list(TACTILE_PHYSICAL_FINGER_SHAPE), ensure_ascii=False,
            )
            f.attrs["tactile_physical_slot_count"] = TACTILE_PHYSICAL_SLOT_COUNT
            f.attrs["tactile_physical_active_count"] = TACTILE_PHYSICAL_ACTIVE_COUNT
            f.attrs["tactile_palm_present"] = TACTILE_PALM_PRESENT
            f.attrs["tactile_to_manus_source_node_ids"] = json.dumps(
                {
                    finger: list(TACTILE_TO_MANUS_NODE_IDS[finger])
                    for finger in TACTILE_FINGER_ORDER
                },
                ensure_ascii=False,
            )
            f.attrs["tactile_to_manus_exported_joint_indices"] = json.dumps(
                manus_export_indices, ensure_ascii=False,
            )
            f.attrs["tactile_manus_correspondence_level"] = (
                "finger group only; no taxel-to-MANUS-joint mapping"
            )
            for side in TACTILE_SIDES:
                stream = tactile_meta["streams"][side]
                f.attrs[f"tactile_{side}_stream_id"] = stream["stream_id"]
                f.attrs[f"tactile_{side}_cellmap_sha256"] = \
                    stream["wire_layout"].get("cellmap_sha256", "")
                f.attrs[f"tactile_{side}_physical_interpretation"] = json.dumps(
                    finger_region_metadata(), ensure_ascii=False,
                )
                f.attrs[f"tactile_{side}_capture_physical_interpretation"] = json.dumps(
                    stream.get("physical_interpretation") or {}, ensure_ascii=False,
                )
                frames = tactile[side] if tactile is not None else []
                if len(frames) > 1:
                    span_ns = int(frames[-1]["wall_ns"]) - int(frames[0]["wall_ns"])
                    if span_ns > 0:
                        f.attrs[f"tactile_{side}_source_actual_rate_hz"] = (
                            (len(frames) - 1) * 1e9 / span_ns
                        )
        f.attrs["hand_frame"] = "wrist-local (relative to MANUS wrist root)"
        f.attrs["controller_pose_frame"] = (
            "PICO world, right-handed X-forward Y-left Z-up, before wrist calibration"
        )
        f.attrs["wrist_pose_semantics"] = (
            "compose_pose(controller_pose, controller_to_wrist_calibration)"
        )
        f.attrs["ctrl_convention"] = ("teleop_wrist_Q180" if args.teleop_wrist
                                      else "ego_world_same_as_head")
        f.attrs["pose_layout"] = "pos(xyz) + quat(x,y,z,w)"
        f.attrs["timestamp_clock"] = "PICO timeStampNs (sensor clock, resample timeline)"
        # ---- 双目 VST (SBS 单文件, 左右同 frame_idx) ----
        f.attrs["video_path"] = str(args.vst or "")
        f.attrs["video_ts_path"] = str(args.vst_ts or "")
        f.attrs["video_qpc_ts_path"] = str(vst_qpc_ts or "")
        f.attrs["video_stereo"] = True
        f.attrs["video_layout"] = "sbs_lr"  # left|right horizontal
        f.attrs["video_width"] = vw
        f.attrs["video_height"] = vh
        if video_frame_count is not None:
            f.attrs["video_frame_count"] = int(video_frame_count)
        f.attrs["video_eye_width"] = eye_w
        f.attrs["video_eye_height"] = eye_h
        f.attrs["video_eye"] = args.video_eye  # both|left|right 使用偏好
        f.attrs["video_left_crop"] = json.dumps(crop_left)
        f.attrs["video_right_crop"] = json.dumps(crop_right)
        f.attrs["video_cam_path"] = str(cam_path) if cam_path.is_file() else ""
        f.attrs["video_note"] = (
            "PICO SBS video: same video_frame_idx for both eyes; "
            "left=frame[:,0:eye_w], right=frame[:,eye_w:eye_w*2]. "
            "For NVIDIA stereo depth use BOTH crops + video_cam left/right extrinsics."
        )
        if cam_blob is not None:
            # 精简写入左右 K/R/t, 避免整文件过大
            slim = {}
            for side in ("left", "right"):
                if side in cam_blob and isinstance(cam_blob[side], dict):
                    slim[side] = {k: cam_blob[side][k]
                                  for k in ("R", "t", "fx", "fy", "cx", "cy")
                                  if k in cam_blob[side]}
            if "intrinsics_native" in cam_blob:
                slim["intrinsics_native"] = cam_blob["intrinsics_native"]
            if "extrinsic_convention" in cam_blob:
                slim["extrinsic_convention"] = cam_blob["extrinsic_convention"]
            f.attrs["video_cam"] = json.dumps(slim, ensure_ascii=False)

        f.create_dataset("timestamp_ns", data=frame_time_ns, compression="gzip")
        f.create_dataset("source_row_idx", data=kept_source_rows, compression="gzip")
        f.create_dataset("segment_id", data=export_segment_ids, compression="gzip")
        f.create_dataset("recv_wall_ns", data=walls, compression="gzip")
        f.create_dataset("recv_qpc_ns", data=qpcs, compression="gzip")
        # 左右眼共享同一 SBS 帧号（深度模型各自 crop）
        f.create_dataset("video_frame_idx", data=vidx, compression="gzip")
        f.create_dataset("video_frame_idx_left", data=vidx, compression="gzip")
        f.create_dataset("video_frame_idx_right", data=vidx, compression="gzip")
        f.create_dataset("video_valid", data=video_aligned["valid"], compression="gzip")
        f.create_dataset("video_source_recv_wall_ns",
                         data=video_aligned["recv_wall_ns"], compression="gzip")
        f.create_dataset("video_source_recv_qpc_ns",
                         data=video_aligned["recv_qpc_ns"], compression="gzip")
        f.create_dataset("video_offset_ms", data=video_aligned["offset_ms"],
                         compression="gzip")
        f.create_dataset("head_pose", data=head_pose, compression="gzip")
        f.create_dataset("left_controller_pose", data=left_controller_pose,
                         compression="gzip")
        f.create_dataset("right_controller_pose", data=right_controller_pose,
                         compression="gzip")
        f.create_dataset("left_wrist_pose", data=left_wrist_pose, compression="gzip")
        f.create_dataset("right_wrist_pose", data=right_wrist_pose, compression="gzip")
        f.create_dataset("left_hand_joints", data=left_hand_joints, compression="gzip")
        f.create_dataset("right_hand_joints", data=right_hand_joints, compression="gzip")
        f.create_dataset("left_hand_valid", data=hand_valid["left"], compression="gzip")
        f.create_dataset("right_hand_valid", data=hand_valid["right"], compression="gzip")
        for side in ("left", "right"):
            item = manus_aligned[side]
            f.create_dataset(f"{side}_hand_source_recv_wall_ns",
                             data=item["recv_wall_ns"], compression="gzip")
            f.create_dataset(f"{side}_hand_source_recv_qpc_ns",
                             data=item["recv_qpc_ns"], compression="gzip")
            f.create_dataset(f"{side}_hand_offset_ms", data=item["offset_ms"],
                             compression="gzip")
        if tactile_aligned is not None:
            assert tactile_spatial is not None
            assert manus_node_id_table is not None
            f.create_dataset(
                "tactile_finger_manus_node_ids", data=manus_node_id_table,
            )
            for side in TACTILE_SIDES:
                item = tactile_aligned[side]
                spatial = tactile_spatial[side]
                prefix = f"{side}_tactile"
                f.create_dataset(f"{prefix}_values", data=item["values"], compression="gzip")
                f.create_dataset(f"{prefix}_valid", data=item["valid"], compression="gzip")
                f.create_dataset(f"{prefix}_recv_wall_ns", data=item["recv_wall_ns"],
                                 compression="gzip")
                f.create_dataset(f"{prefix}_recv_qpc_ns", data=item["recv_qpc_ns"],
                                 compression="gzip")
                f.create_dataset(f"{prefix}_stream_seq", data=item["stream_seq"],
                                 compression="gzip")
                f.create_dataset(f"{prefix}_record_seq", data=item["record_seq"],
                                 compression="gzip")
                f.create_dataset(f"{prefix}_offset_ms", data=item["offset_ms"],
                                 compression="gzip")
                f.create_dataset(f"{prefix}_fingers", data=spatial["fingers"],
                                 compression="gzip")
                f.create_dataset(
                    f"{prefix}_fingers_active_mask",
                    data=spatial["fingers_active_mask"],
                )
    os.replace(partial_output, output_path)

    lcov = 100 * hand_valid["left"].mean()
    rcov = 100 * hand_valid["right"].mean()
    dur = T / args.fps
    span = (src[-1]["ts"] - src[0]["ts"]) / 1e9
    calib_name = "identity" if not args.calib else Path(args.calib).name
    vcov = 100 * (vidx >= 0).mean()
    print(f"[export] 写出 {args.out}  schema=egodex_v1 (stereo SBS)")
    print(f"  对齐时钟={alignment_clock}  公共有效区间={(common_end-common_start)/1e9:.2f}s")
    print(f"  源帧(头+双手柄有效)={len(src)}  跨度≈{span:.1f}s")
    print(f"  完整帧筛选={T}/{pre_filter_T} ({complete_coverage*100:.2f}%)  "
          f"剔除={pre_filter_T-T}  缺口后分段={int(export_segment_ids[-1])+1}")
    print(f"  导出帧数={T}  等效时长≈{dur:.1f}s  标称帧率={args.fps}Hz")
    print(f"  手部关节={njoint} ({args.hands})  左手有效={lcov:.0f}%  右手有效={rcov:.0f}%")
    print(f"  视频帧匹配={vcov:.0f}%  stereo=SBS {vw}x{vh} 眼={eye_w}x{eye_h}  "
          f"eye={args.video_eye}  vst={args.vst or '(无)'}")
    for name, metrics in quality.items():
        print("  时间质检 {}: 覆盖={:.2f}% p95={:.3f}ms max={:.3f}ms".format(
            name, metrics["coverage"] * 100.0,
            metrics["p95_ms"], metrics["max_ms"],
        ))
    if tactile_aligned is not None:
        print("  触觉原始值=双手×{}通道（不插值）  左覆盖={:.1f}%  右覆盖={:.1f}%".format(
            TACTILE_VALUE_COUNT,
            tactile_coverage["left"] * 100.0,
            tactile_coverage["right"] * 100.0,
        ))
        print("  触觉派生=五片实物指端阵列(thumb,index,middle,ring,pinky) 4x8; "
              "拇指32点、其余各28点，共144个有效点；无手掌阵列")
    print(f"  坐标系: 右手系 X前Y左Z上, 米; 手指=腕局部系; 标定={calib_name}")


if __name__ == "__main__":
    main()
