#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""align_pico_manus.py — 用统一主机时间戳把 MANUS/触觉帧对齐到 PICO 帧。

新采集优先使用各路 `recv_qpc_ns` 高精度单调钟；任一路缺失时整条会话统一回退
`recv_wall_ns`，不会混用时钟域。对每个 PICO 跟踪帧按最近邻挂上左/右手套和
触觉源帧，并输出质检（中位 |Δt|、最大间隙、超阈值比例）。

用法：
  python3 align_pico_manus.py pico_xxx.jsonl manus_yyy.jsonl -o aligned.jsonl
  python3 align_pico_manus.py pico_xxx.jsonl manus_yyy.jsonl --hdf5 aligned.hdf5
  python3 align_pico_manus.py ... --max-skew-ms 20 --full   # --full 保留全部节点

对齐基准默认 PICO（ego 视频/头显时间轴）。用 --base manus 反过来。
"""
from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import re
import statistics
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from tactile_layout import active_coordinates

NS_PER_MS = 1_000_000
TACTILE_FRAME_SCHEMA = "pico_tactile_wire_frame_v1"
TACTILE_META_SCHEMA = "pico_tactile_raw_meta_v1"
TACTILE_VALUE_COUNT = 369
TACTILE_SIDES = ("left", "right")


# ------------------------------------------------------------------ 解析

def _wall_ns(rec: dict) -> Optional[int]:
    if "recv_wall_ns" in rec and rec["recv_wall_ns"]:
        return int(rec["recv_wall_ns"])
    if "recv_ts" in rec and rec["recv_ts"]:
        return int(float(rec["recv_ts"]) * 1e9)
    return None


def _qpc_ns(rec: dict) -> Optional[int]:
    value = rec.get("recv_qpc_ns")
    if value:
        return int(value)
    return None


def _pose(node: Any) -> Optional[Tuple[List[float], List[float]]]:
    """从 PICO 字段取 (pos[3], quat[4])。兼容 pos/quat 数组或 'pose' 字符串。"""
    if isinstance(node, dict):
        pos = node.get("pos")
        quat = node.get("quat") or node.get("ori")
        if pos and quat and len(pos) >= 3 and len(quat) >= 4:
            return [float(x) for x in pos[:3]], [float(x) for x in quat[:4]]
        s = node.get("pose")
        if isinstance(s, str):
            try:
                v = [float(x) for x in s.split(",")]
            except ValueError:
                v = []
            if len(v) >= 7:
                return v[:3], v[3:7]
    return None


def load_pico(path: Path) -> List[dict]:
    """PICO 跟踪帧列表：{wall_ns, idx, head, left_ctrl, right_ctrl}。"""
    frames: List[dict] = []
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
            ctrl = data.get("Controller") if isinstance(data.get("Controller"), dict) else {}
            frames.append({
                "wall_ns": wall,
                "qpc_ns": _qpc_ns(rec),
                "idx": i,
                "head": _pose(data.get("Head")),
                "left_ctrl": _pose(ctrl.get("left")),
                "right_ctrl": _pose(ctrl.get("right")),
            })
    frames.sort(key=lambda d: d["wall_ns"])
    return frames


def load_manus(path: Path) -> Dict[str, List[dict]]:
    """MANUS 帧按 side 分组，各自按 wall_ns 排序。"""
    by_side: Dict[str, List[dict]] = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or not line.startswith("{"):
                continue
            try:
                obj = json.loads(line)
            except Exception:  # noqa: BLE001
                continue
            if obj.get("type") != "manus_frame":
                continue
            wall = _wall_ns(obj)
            if wall is None:
                continue
            obj["wall_ns"] = wall
            obj["qpc_ns"] = _qpc_ns(obj)
            by_side.setdefault(obj.get("side", "unknown"), []).append(obj)
    for side in by_side:
        by_side[side].sort(key=lambda d: d["wall_ns"])
    return by_side


def _tactile_error(message: str) -> ValueError:
    return ValueError("触觉输入无效: " + message)


def _tactile_directory_session_names(path: Path) -> set[str]:
    """返回触觉目录可能对应的会话名，兼容扁平和批次目录。

    扁平布局为 ``tactile_raw/3_001/tactile.jsonl``；批次布局为
    ``tactile_raw/3/001/tactile.jsonl``，但 meta 中始终保存 ``3_001``。
    """
    directory = Path(path).parent
    # Task-first layout: data/sessions/<prefix>/<index>/raw/tactile.jsonl.
    # The session identity belongs to raw/ 的父目录，而不是名为 "raw" 的目录。
    if directory.name == "raw" and directory.parent.name:
        directory = directory.parent
    names = {directory.name}
    if re.fullmatch(r"[0-9]{3,4}", directory.name) and directory.parent.name:
        names.add(f"{directory.parent.name}_{directory.name}")
    return names


def load_tactile(path: Path, meta_path: Optional[Path] = None
                 ) -> Tuple[Dict[str, List[dict]], Dict[str, Any]]:
    """严格读取已封存的双手触觉帧，并校验旁路元数据与文件完整性。

    触觉会直接进入训练包，因此这里与 PICO/MANUS 的容错读取不同：任何损坏、
    未完成封存、通道数变化或左右流缺失都报错，不静默丢行。
    """
    path = Path(path)
    meta_path = Path(meta_path) if meta_path else path.with_name("tactile.meta.json")
    if not path.is_file():
        raise _tactile_error("数据文件不存在: {}".format(path))
    if not meta_path.is_file():
        raise _tactile_error("缺少封存元数据: {}".format(meta_path))

    try:
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
    except Exception as exc:  # noqa: BLE001
        raise _tactile_error("无法解析 {}: {}".format(meta_path, exc)) from exc
    if not isinstance(meta, dict) or meta.get("schema") != TACTILE_META_SCHEMA:
        raise _tactile_error("meta schema 不是 {}".format(TACTILE_META_SCHEMA))
    if meta.get("complete") is not True or meta.get("capture_valid") is not True:
        raise _tactile_error("采集未完成封存（需要 complete=true 且 capture_valid=true）")
    if meta.get("state") != "complete":
        raise _tactile_error("meta state 不是 complete")
    if meta.get("frame_schema") != TACTILE_FRAME_SCHEMA:
        raise _tactile_error("frame_schema 不是 {}".format(TACTILE_FRAME_SCHEMA))
    if meta.get("values_semantics") != "wire_active_order_unmasked":
        raise _tactile_error("不支持的 values_semantics: {}".format(
            meta.get("values_semantics")))
    if meta.get("data_file") not in (path.name, None) or \
            meta.get("final_data_file") not in (path.name, None):
        raise _tactile_error("meta 指向的数据文件与 {} 不一致".format(path.name))
    session = meta.get("session")
    directory_sessions = _tactile_directory_session_names(path)
    if session and session not in directory_sessions:
        raise _tactile_error("meta session={}，目录允许的会话名={}".format(
            session, sorted(directory_sessions)))

    summary = meta.get("summary")
    if not isinstance(summary, dict) or summary.get("complete") is not True:
        raise _tactile_error("meta.summary 不完整")
    if int(summary.get("writer_errors", -1)) != 0:
        raise _tactile_error("writer_errors 非零")
    if int(summary.get("subscription_gap_frames", -1)) != 0:
        raise _tactile_error("subscription_gap_frames 非零")
    transient_warnings = summary.get("transient_health_warnings") or {}
    transient_warning_count = int(transient_warnings.get("incident_count") or 0)
    if transient_warning_count:
        print(
            "[tactile] WARNING: 本次触觉采集包含 {} 个可恢复串口瞬时告警；"
            "数据已完整封存，详情见 tactile.meta.json summary.transient_health_warnings".format(
                transient_warning_count
            ),
            file=sys.stderr,
        )

    stat = path.stat()
    if int(summary.get("data_bytes", -1)) != stat.st_size:
        raise _tactile_error("文件字节数与 meta 不一致")
    expected_sha = str(summary.get("data_sha256") or "").lower()
    if len(expected_sha) != 64:
        raise _tactile_error("meta 缺少有效 data_sha256")
    digest = hashlib.sha256()
    with open(path, "rb") as raw:
        for chunk in iter(lambda: raw.read(1024 * 1024), b""):
            digest.update(chunk)
    if digest.hexdigest() != expected_sha:
        raise _tactile_error("文件 SHA-256 与 meta 不一致")

    streams = meta.get("streams")
    if not isinstance(streams, dict):
        raise _tactile_error("meta 缺少 streams")
    requested = set(meta.get("requested_sides") or [])
    if not set(TACTILE_SIDES).issubset(requested):
        raise _tactile_error("requested_sides 必须包含 left/right")
    expected_stream_ids: Dict[str, str] = {}
    for side in TACTILE_SIDES:
        stream = streams.get(side)
        layout = stream.get("wire_layout") if isinstance(stream, dict) else None
        if not isinstance(layout, dict):
            raise _tactile_error("meta 缺少 {} wire_layout".format(side))
        if int(layout.get("active_count", -1)) != TACTILE_VALUE_COUNT:
            raise _tactile_error("{} active_count 不是 {}".format(
                side, TACTILE_VALUE_COUNT))
        if layout.get("wire_payload_encoding") != "signed_int16_le":
            raise _tactile_error("{} 不是 signed_int16_le".format(side))
        try:
            active_coordinates(layout)
        except ValueError as exc:
            raise _tactile_error("{} {}".format(side, exc)) from exc
        sid = stream.get("stream_id")
        if not isinstance(sid, str) or not sid:
            raise _tactile_error("meta 缺少 {} stream_id".format(side))
        expected_stream_ids[side] = sid

    by_side: Dict[str, List[dict]] = {side: [] for side in TACTILE_SIDES}
    last_wall: Dict[str, Optional[int]] = {side: None for side in TACTILE_SIDES}
    last_qpc: Dict[str, Optional[int]] = {side: None for side in TACTILE_SIDES}
    last_stream_seq: Dict[str, Optional[int]] = {side: None for side in TACTILE_SIDES}
    last_record_seq: Optional[int] = None
    records_total = 0
    with open(path, encoding="utf-8") as src:
        for lineno, line in enumerate(src, 1):
            if not line.strip():
                continue
            try:
                obj = json.loads(line)
            except Exception as exc:  # noqa: BLE001
                raise _tactile_error("第 {} 行 JSON 损坏: {}".format(lineno, exc)) from exc
            if not isinstance(obj, dict) or obj.get("type") != "tactile_frame" or \
                    obj.get("schema") != TACTILE_FRAME_SCHEMA:
                raise _tactile_error("第 {} 行不是 {} 触觉帧".format(
                    lineno, TACTILE_FRAME_SCHEMA))
            side = obj.get("side")
            if side not in TACTILE_SIDES:
                raise _tactile_error("第 {} 行 side 无效: {}".format(lineno, side))
            if obj.get("stream_id") != expected_stream_ids[side]:
                raise _tactile_error("第 {} 行 {} stream_id 与 meta 不一致".format(
                    lineno, side))

            record_seq = obj.get("record_seq")
            stream_seq = obj.get("stream_seq")
            wall = obj.get("recv_wall_ns")
            qpc = obj.get("recv_qpc_ns")
            for name, value in (("record_seq", record_seq),
                                ("stream_seq", stream_seq),
                                ("recv_wall_ns", wall)):
                if isinstance(value, bool) or not isinstance(value, int):
                    raise _tactile_error("第 {} 行 {} 不是整数".format(lineno, name))
            if wall <= 0:
                raise _tactile_error("第 {} 行 recv_wall_ns 非正数".format(lineno))
            if qpc is not None:
                if isinstance(qpc, bool) or not isinstance(qpc, int) or qpc <= 0:
                    raise _tactile_error("第 {} 行 recv_qpc_ns 无效".format(lineno))
            if last_record_seq is not None and record_seq <= last_record_seq:
                raise _tactile_error("第 {} 行 record_seq 未严格递增".format(lineno))
            if last_stream_seq[side] is not None and stream_seq <= last_stream_seq[side]:
                raise _tactile_error("第 {} 行 {} stream_seq 未严格递增".format(
                    lineno, side))
            if last_wall[side] is not None and wall <= last_wall[side]:
                raise _tactile_error("第 {} 行 {} recv_wall_ns 未严格递增".format(
                    lineno, side))
            if (qpc is not None and last_qpc[side] is not None
                    and qpc <= last_qpc[side]):
                raise _tactile_error("第 {} 行 {} recv_qpc_ns 未严格递增".format(
                    lineno, side))

            values = obj.get("wire_values")
            if not isinstance(values, list) or len(values) != TACTILE_VALUE_COUNT:
                raise _tactile_error("第 {} 行 wire_values 数量不是 {}".format(
                    lineno, TACTILE_VALUE_COUNT))
            if any(isinstance(v, bool) or not isinstance(v, int) or
                   v < -32768 or v > 32767 for v in values):
                raise _tactile_error("第 {} 行 wire_values 不是 signed int16 整数".format(
                    lineno))

            obj["wall_ns"] = wall
            obj["qpc_ns"] = int(qpc) if qpc else None
            by_side[side].append(obj)
            last_record_seq = record_seq
            last_stream_seq[side] = stream_seq
            last_wall[side] = wall
            if qpc is not None:
                last_qpc[side] = qpc
            records_total += 1

    frames_by_side = summary.get("frames_by_side") or {}
    for side in TACTILE_SIDES:
        actual = len(by_side[side])
        if actual == 0:
            raise _tactile_error("{} 流为空".format(side))
        if int(frames_by_side.get(side, -1)) != actual:
            raise _tactile_error("{} 帧数与 meta 不一致".format(side))
    for count_key in ("records_total", "durable_records"):
        if int(summary.get(count_key, -1)) != records_total:
            raise _tactile_error("{} 与实际行数不一致".format(count_key))
    return by_side, meta


# ------------------------------------------------------------------ 最近邻

def nearest(sorted_frames: List[dict], keys: List[int], t: int,
            key_name: str = "wall_ns") -> Optional[dict]:
    if not sorted_frames:
        return None
    j = bisect.bisect_left(keys, t)
    best = None
    best_dt = None
    for k in (j - 1, j):
        if 0 <= k < len(sorted_frames):
            dt = abs(int(sorted_frames[k][key_name]) - t)
            if best_dt is None or dt < best_dt:
                best_dt = dt
                best = sorted_frames[k]
    return best


def _manus_out(obj: dict, dt_ns: int, full: bool) -> dict:
    out = {
        "wall_ns": obj["wall_ns"],
        "qpc_ns": obj.get("qpc_ns"),
        "dt_ms": abs(dt_ns) / NS_PER_MS,
        "offset_ms": dt_ns / NS_PER_MS,
        "glove_id": obj.get("glove_id"),
        "node_count": obj.get("node_count"),
        "fingertip_xyz": _fingertips(obj),
    }
    nodes = obj.get("nodes") or []
    if nodes:
        out["wrist_pose"] = nodes[0]  # node0 = 腕根
    if full:
        out["nodes"] = nodes
        out["node_ids"] = obj.get("node_ids")
        out["parent_ids"] = obj.get("parent_ids")
        out["joint_types"] = obj.get("joint_types")
        out["chain_types"] = obj.get("chain_types")
    return out


def _tactile_out(obj: dict, dt_ns: int) -> dict:
    """保留一帧全部原始通道，并记录可复查的源帧身份与有符号偏差。"""
    return {
        "wall_ns": obj["wall_ns"],
        "qpc_ns": obj.get("qpc_ns"),
        "dt_ms": abs(dt_ns) / NS_PER_MS,
        "offset_ms": dt_ns / NS_PER_MS,
        "record_seq": obj["record_seq"],
        "stream_seq": obj["stream_seq"],
        "stream_id": obj["stream_id"],
        "wire_values": obj["wire_values"],
    }


_JOINT_TIP = {"tip", "5"}
_CHAIN_ORDER = (
    {"thumb", "fingerthumb", "5"}, {"index", "fingerindex", "6"},
    {"middle", "fingermiddle", "7"}, {"ring", "fingerring", "8"},
    {"pinky", "fingerpinky", "9"},
)


def _fingertips(obj: dict) -> List[List[float]]:
    nodes = obj.get("nodes") or []
    jt = obj.get("joint_types") or []
    ct = obj.get("chain_types") or []
    out = [[0.0, 0.0, 0.0] for _ in range(5)]
    n = min(len(nodes), len(jt), len(ct))
    for i in range(n):
        if str(jt[i] or "").strip().lower() not in _JOINT_TIP:
            continue
        chain = str(ct[i] or "").strip().lower()
        for fi, aliases in enumerate(_CHAIN_ORDER):
            if chain in aliases:
                out[fi] = list(nodes[i][:3])
                break
    return out


# ------------------------------------------------------------------ 对齐主流程

def choose_alignment_clock(pico: List[dict], manus: Dict[str, List[dict]],
                           tactile: Optional[Dict[str, List[dict]]] = None) -> str:
    """为整次会话选择唯一主机时钟，禁止不同数据路混用时钟域。"""
    groups: List[List[dict]] = [pico]
    groups.extend(manus.get(side) or [] for side in ("left", "right"))
    if tactile:
        groups.extend(tactile.get(side) or [] for side in TACTILE_SIDES)
    have_complete_qpc = bool(groups) and all(
        group and all(frame.get("qpc_ns") for frame in group)
        for group in groups
    )
    return "qpc_ns" if have_complete_qpc else "wall_ns"


def align(pico: List[dict], manus: Dict[str, List[dict]], full: bool,
          gate_ns: Optional[int] = None,
          tactile: Optional[Dict[str, List[dict]]] = None,
          tactile_gate_ns: Optional[int] = None) -> Tuple[List[dict], Dict[str, Any]]:
    tactile = tactile or {}
    clock_key = choose_alignment_clock(pico, manus, tactile)
    pico = sorted(pico, key=lambda frame: int(frame[clock_key]))
    manus = {
        side: sorted(frames, key=lambda frame: int(frame[clock_key]))
        for side, frames in manus.items()
    }
    tactile = {
        side: sorted(frames, key=lambda frame: int(frame[clock_key]))
        for side, frames in tactile.items()
    }
    keys = {
        side: [int(d[clock_key]) for d in frames]
        for side, frames in manus.items()
    }
    tactile_keys = {
        side: [int(d[clock_key]) for d in frames]
        for side, frames in tactile.items()
    }
    aligned: List[dict] = []
    dts: Dict[str, List[float]] = {s: [] for s in manus}
    matched: Dict[str, int] = {s: 0 for s in ("left", "right")}
    tactile_dts: Dict[str, List[float]] = {s: [] for s in TACTILE_SIDES}
    tactile_matched: Dict[str, int] = {s: 0 for s in TACTILE_SIDES}
    for pf in pico:
        rec = {
            "pico_wall_ns": pf["wall_ns"],
            "pico_qpc_ns": pf.get("qpc_ns"),
            "alignment_clock": "host_qpc" if clock_key == "qpc_ns" else "host_wall",
            "pico_idx": pf["idx"],
            "head": pf["head"],
            "left_ctrl": pf["left_ctrl"],
            "right_ctrl": pf["right_ctrl"],
            "manus": {},
            "tactile": {},
        }
        for side in ("left", "right"):
            frames = manus.get(side)
            if not frames:
                continue
            target_ns = int(pf[clock_key])
            m = nearest(frames, keys[side], target_ns, clock_key)
            if m is None:
                continue
            dt_ns = int(m[clock_key]) - target_ns
            # 门限外(两端未同时录制的时段)不硬挂, 视为该帧无手套数据
            if gate_ns is not None and abs(dt_ns) > gate_ns:
                continue
            dts[side].append(abs(dt_ns) / NS_PER_MS)
            matched[side] += 1
            rec["manus"][side] = _manus_out(m, dt_ns, full)
        for side in TACTILE_SIDES:
            frames = tactile.get(side)
            if not frames:
                continue
            target_ns = int(pf[clock_key])
            tframe = nearest(frames, tactile_keys[side], target_ns, clock_key)
            if tframe is None:
                continue
            dt_ns = int(tframe[clock_key]) - target_ns
            if tactile_gate_ns is not None and abs(dt_ns) > tactile_gate_ns:
                continue
            tactile_dts[side].append(abs(dt_ns) / NS_PER_MS)
            tactile_matched[side] += 1
            rec["tactile"][side] = _tactile_out(tframe, dt_ns)
        aligned.append(rec)

    qc: Dict[str, Any] = {
        "pico_frames": len(pico),
        "alignment_clock": "host_qpc" if clock_key == "qpc_ns" else "host_wall",
    }
    for side, frames in manus.items():
        qc[f"manus_{side}_frames"] = len(frames)
    for side in ("left", "right"):
        qc[f"matched_{side}"] = matched[side]
        d = dts.get(side) or []
        if d:
            qc[f"dt_{side}_ms"] = {
                "median": round(statistics.median(d), 3),
                "mean": round(statistics.fmean(d), 3),
                "p95": round(sorted(d)[int(0.95 * (len(d) - 1))], 3),
                "max": round(max(d), 3),
            }
    for side in TACTILE_SIDES:
        if side not in tactile:
            continue
        qc[f"tactile_{side}_frames"] = len(tactile[side])
        qc[f"tactile_matched_{side}"] = tactile_matched[side]
        d = tactile_dts[side]
        if d:
            qc[f"tactile_dt_{side}_ms"] = {
                "median": round(statistics.median(d), 3),
                "mean": round(statistics.fmean(d), 3),
                "p95": round(sorted(d)[int(0.95 * (len(d) - 1))], 3),
                "max": round(max(d), 3),
            }
    return aligned, qc


def write_hdf5(path: Path, aligned: List[dict]) -> None:
    import h5py
    import numpy as np
    n = len(aligned)

    def pose_arr(key):
        a = np.full((n, 7), np.nan)
        for i, r in enumerate(aligned):
            p = r.get(key)
            if p:
                a[i, :3] = p[0]
                a[i, 3:] = p[1]
        return a

    with h5py.File(path, "w") as f:
        clock = aligned[0].get("alignment_clock", "host_wall") if aligned else "host_wall"
        f.attrs["sync_method"] = clock + "_nearest_neighbor"
        f.attrs["alignment_clock"] = clock
        f.attrs["base"] = "pico"
        f.create_dataset("pico_wall_ns", data=np.array([r["pico_wall_ns"] for r in aligned], dtype=np.int64))
        f.create_dataset("pico_qpc_ns", data=np.array([
            int(r.get("pico_qpc_ns") or -1) for r in aligned
        ], dtype=np.int64))
        f.create_dataset("pico_idx", data=np.array([r["pico_idx"] for r in aligned], dtype=np.int64))
        f.create_dataset("head_pose", data=pose_arr("head"))
        f.create_dataset("left_ctrl_pose", data=pose_arr("left_ctrl"))
        f.create_dataset("right_ctrl_pose", data=pose_arr("right_ctrl"))
        for side in ("left", "right"):
            dt = np.full(n, np.nan)
            offset = np.full(n, np.nan)
            source_wall = np.full(n, -1, dtype=np.int64)
            source_qpc = np.full(n, -1, dtype=np.int64)
            wrist = np.full((n, 7), np.nan)
            ft = np.full((n, 5, 3), np.nan)
            for i, r in enumerate(aligned):
                m = r["manus"].get(side)
                if not m:
                    continue
                dt[i] = m["dt_ms"]
                offset[i] = m.get("offset_ms", np.nan)
                source_wall[i] = int(m["wall_ns"])
                source_qpc[i] = int(m.get("qpc_ns") or -1)
                if m.get("wrist_pose"):
                    wrist[i] = m["wrist_pose"][:7]
                if m.get("fingertip_xyz"):
                    ft[i] = np.asarray(m["fingertip_xyz"])[:5, :3]
            g = f.create_group(f"manus_{side}")
            g.create_dataset("dt_ms", data=dt)
            g.create_dataset("offset_ms", data=offset)
            g.create_dataset("recv_wall_ns", data=source_wall)
            g.create_dataset("recv_qpc_ns", data=source_qpc)
            g.create_dataset("wrist_pose", data=wrist)
            g.create_dataset("fingertip_xyz", data=ft)
        for side in TACTILE_SIDES:
            if not any((rec.get("tactile") or {}).get(side) for rec in aligned):
                continue
            values = np.zeros((n, TACTILE_VALUE_COUNT), dtype=np.int16)
            valid = np.zeros(n, dtype=bool)
            source_wall = np.full(n, -1, dtype=np.int64)
            source_qpc = np.full(n, -1, dtype=np.int64)
            stream_seq = np.full(n, -1, dtype=np.int64)
            record_seq = np.full(n, -1, dtype=np.int64)
            dt_ms = np.full(n, np.nan, dtype=np.float32)
            for i, rec in enumerate(aligned):
                tactile_rec = (rec.get("tactile") or {}).get(side)
                if not tactile_rec:
                    continue
                values[i] = np.asarray(tactile_rec["wire_values"], dtype=np.int16)
                valid[i] = True
                source_wall[i] = int(tactile_rec["wall_ns"])
                source_qpc[i] = int(tactile_rec.get("qpc_ns") or -1)
                stream_seq[i] = int(tactile_rec["stream_seq"])
                record_seq[i] = int(tactile_rec["record_seq"])
                dt_ms[i] = float(tactile_rec["offset_ms"])
            g = f.create_group(f"tactile_{side}")
            g.attrs["value_count"] = TACTILE_VALUE_COUNT
            g.attrs["values_semantics"] = "wire_active_order_unmasked"
            g.create_dataset("values", data=values, compression="gzip")
            g.create_dataset("valid", data=valid, compression="gzip")
            g.create_dataset("recv_wall_ns", data=source_wall, compression="gzip")
            g.create_dataset("recv_qpc_ns", data=source_qpc, compression="gzip")
            g.create_dataset("stream_seq", data=stream_seq, compression="gzip")
            g.create_dataset("record_seq", data=record_seq, compression="gzip")
            g.create_dataset("offset_ms", data=dt_ms, compression="gzip")


def main() -> None:
    ap = argparse.ArgumentParser(description="按统一主机时间戳对齐 PICO、MANUS 与可选双手触觉")
    ap.add_argument("pico", help="pico_*.jsonl")
    ap.add_argument("manus", help="manus_*.jsonl")
    ap.add_argument("-o", "--out", default=None, help="对齐结果 JSONL 输出路径")
    ap.add_argument("--hdf5", default=None, help="额外/替代输出 HDF5 (需要 h5py)")
    ap.add_argument("--full", action="store_true", help="JSONL 里保留 MANUS 全部节点")
    ap.add_argument("--max-skew-ms", type=float, default=20.0, help="超过此中位偏差则告警")
    ap.add_argument("--gate-ms", type=float, default=30.0,
                    help="最近邻超过此值则视为该帧无手套数据(两端未同时录制的时段)。0=不门限")
    ap.add_argument("--tactile", default=None,
                    help="同一任务 raw/tactile.jsonl（旧 tactile_raw 也兼容）")
    ap.add_argument("--tactile-meta", default=None,
                    help="触觉封存元数据；缺省自动取 tactile.jsonl 旁的 tactile.meta.json")
    ap.add_argument("--tactile-gate-ms", type=float, default=30.0,
                    help="触觉最近邻门限；0=不门限（默认 30ms）")
    args = ap.parse_args()
    if args.tactile_meta and not args.tactile:
        ap.error("--tactile-meta 必须与 --tactile 一起使用")

    pico = load_pico(Path(args.pico))
    manus = load_manus(Path(args.manus))
    if not pico:
        print("[align] PICO 无有效跟踪帧。", file=sys.stderr)
        sys.exit(1)
    if not manus:
        print("[align] MANUS 无有效帧。", file=sys.stderr)
        sys.exit(1)

    tactile: Dict[str, List[dict]] = {}
    tactile_meta: Optional[Dict[str, Any]] = None
    if args.tactile:
        try:
            tactile, tactile_meta = load_tactile(
                Path(args.tactile),
                Path(args.tactile_meta) if args.tactile_meta else None,
            )
        except Exception as exc:  # noqa: BLE001
            print("[align] {}".format(exc), file=sys.stderr)
            sys.exit(1)

    gate_ns = int(args.gate_ms * NS_PER_MS) if args.gate_ms and args.gate_ms > 0 else None
    tactile_gate_ns = (int(args.tactile_gate_ms * NS_PER_MS)
                       if args.tactile_gate_ms and args.tactile_gate_ms > 0 else None)
    aligned, qc = align(
        pico, manus, args.full, gate_ns=gate_ns,
        tactile=tactile, tactile_gate_ns=tactile_gate_ns,
    )

    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            for r in aligned:
                f.write(json.dumps(r, ensure_ascii=False) + "\n")
        print(f"[align] 写出 {len(aligned)} 帧 -> {args.out}")
    if args.hdf5:
        try:
            write_hdf5(Path(args.hdf5), aligned)
            print(f"[align] 写出 HDF5 -> {args.hdf5}")
        except Exception as e:  # noqa: BLE001
            print(f"[align] HDF5 写出失败: {e}", file=sys.stderr)

    # 质检报告
    print("\n===== 对齐质检 =====")
    print(f"PICO 帧: {qc['pico_frames']}")
    warn = False
    npico = qc["pico_frames"]
    for side in ("left", "right"):
        nf = qc.get(f"manus_{side}_frames")
        st = qc.get(f"dt_{side}_ms")
        mt = qc.get(f"matched_{side}", 0)
        if nf is None:
            continue
        cov = 100.0 * mt / npico if npico else 0.0
        if st is None:
            print(f"MANUS {side}: {nf} 帧, 门限内匹配 0/{npico} (0%)")
            continue
        flag = ""
        if st["median"] > args.max_skew_ms:
            flag = "  ⚠ 中位超阈值"
            warn = True
        print(f"MANUS {side}: {nf} 帧 | 门限内匹配 {mt}/{npico} ({cov:.0f}%) | "
              f"Δt 中位={st['median']}ms p95={st['p95']}ms 最大={st['max']}ms{flag}")
    if tactile_meta is not None:
        for side in TACTILE_SIDES:
            nf = qc.get(f"tactile_{side}_frames", 0)
            mt = qc.get(f"tactile_matched_{side}", 0)
            st = qc.get(f"tactile_dt_{side}_ms")
            cov = 100.0 * mt / npico if npico else 0.0
            if st is None:
                print(f"TACTILE {side}: {nf} 帧, 门限内匹配 0/{npico} (0%)")
                continue
            flag = ""
            if st["median"] > args.max_skew_ms:
                flag = "  ⚠ 中位超阈值"
                warn = True
            print(f"TACTILE {side}: {nf} 帧 | 门限内匹配 {mt}/{npico} ({cov:.0f}%) | "
                  f"Δt 中位={st['median']}ms p95={st['p95']}ms 最大={st['max']}ms{flag}")
    span_s = (pico[-1]["wall_ns"] - pico[0]["wall_ns"]) / 1e9
    tactile_gate_note = (f", 触觉门限 {args.tactile_gate_ms}ms"
                         if tactile_meta is not None else "")
    print(f"PICO 时长≈{span_s:.1f}s  (告警阈值 {args.max_skew_ms}ms, "
          f"MANUS 门限 {args.gate_ms}ms{tactile_gate_note})")
    if warn:
        print("提示: 中位偏差偏大, 检查两端是否同机、是否同一次会话、MANUS 帧率是否过低。")
    print("说明: 覆盖率<100% 多因两端起/停录时间不同(仅重叠时段有手套数据), 属正常。")


if __name__ == "__main__":
    main()
