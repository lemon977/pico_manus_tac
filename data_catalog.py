#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""data_catalog.py — 会话清单 + 训练包 schema（对接用）。

文档只有三份: README.md / docs/PIPELINE_ZH.md / docs/CALIB_QA_ZH.md

  python3 data_catalog.py --schema
  python3 data_catalog.py
  python3 data_catalog.py vc1 --write-manifest
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from session_layout import (
    DEFAULT_DATA_ROOT,
    list_sessions,
    resolve_session,
    write_manifest,
    new_session_paths,
)

SCHEMA = {
    "docs": [
        "README.md",
        "docs/PIPELINE_ZH.md",
        "docs/CALIB_QA_ZH.md",
        "docs/STEREO_PNP_ZH.md (optional dual-eye overlay PnP)",
    ],
    "training_package": {
        "name": "egodex_v1",
        "alias_old": "方案B / egodex_B_v1",
        "meaning": "完整帧：世界系头/手柄/腕位姿 + 腕局部手指 + VST 双目帧号 + 双手369通道原始触觉",
        "hdf5_datasets": {
            "timestamp_ns": "(T,) PICO 采样钟重采样时间轴",
            "source_row_idx": "(T,) 完整帧筛选前的30Hz目标行号；缺口会重新切段",
            "recv_wall_ns": "(T,) 目标行电脑墙钟（兼容/审计）",
            "recv_qpc_ns": "(T,) 目标行高精度主机单调钟；新会话主对齐钟",
            "segment_id": "(T,)",
            "head_pose": "(T,7) 世界系",
            "left_controller_pose / right_controller_pose": "(T,7) 校准前PICO世界系手柄位姿",
            "left_wrist_pose / right_wrist_pose": "(T,7) 世界系=手柄位姿复合T_calib（含腕偏移 pos）",
            "left_hand_joints / right_hand_joints": "(T,25,3) 腕局部",
            "left_hand_valid / right_hand_valid": "(T,) complete_frames_v4 中恒为 True",
            "left/right_hand_source_recv_wall_ns": "(T,) 最近 MANUS 原始帧墙钟；invalid=-1",
            "left/right_hand_source_recv_qpc_ns": "(T,) 最近 MANUS 原始帧 QPC；invalid=-1",
            "left/right_hand_offset_ms": "(T,) MANUS 源帧-目标行；invalid=NaN",
            "left_tactile_values / right_tactile_values": "(T,369) int16，wire 原始活动通道顺序；不插值，invalid 行填0",
            "left_tactile_valid / right_tactile_valid": "(T,) bool，complete_frames_v4 中恒为 True",
            "left_tactile_recv_wall_ns / right_tactile_recv_wall_ns": "(T,) int64，源触觉帧墙钟；invalid=-1",
            "left_tactile_recv_qpc_ns / right_tactile_recv_qpc_ns": "(T,) int64，源触觉帧 QPC；invalid=-1",
            "left_tactile_stream_seq / right_tactile_stream_seq": "(T,) int64，源设备流序号；invalid=-1",
            "left_tactile_record_seq / right_tactile_record_seq": "(T,) int64，源文件全局记录序号；invalid=-1",
            "left_tactile_offset_ms / right_tactile_offset_ms": "(T,) float32，源触觉墙钟-目标墙钟；invalid=NaN",
            "left_tactile_fingers / right_tactile_fingers": "(T,5,4,8) int16，五片实物指端阵列；顺序 thumb/index/middle/ring/pinky；物理无效点置0",
            "left_tactile_fingers_active_mask / right_tactile_fingers_active_mask": "(5,4,8) bool，拇指32点、其余四指各28点，总计144个物理有效点",
            "tactile_finger_manus_node_ids": "(5,5) int16，五指对应MANUS源节点；thumb尾部=-1",
            "video_frame_idx": "(T,) SBS 整帧号；正式含视频包中均为有效索引",
            "video_frame_idx_left / _right": "(T,) 与上相同（左右共享 SBS 帧）",
            "video_valid": "(T,) 正式含视频包中恒为 True",
            "video_source_recv_wall_ns / video_source_recv_qpc_ns": "(T,) 视频源帧双时钟来源",
            "video_offset_ms": "(T,) 视频源帧-目标行；invalid=NaN",
        },
        "attrs": [
            "video_path", "video_stereo=True", "video_layout=sbs_lr",
            "video_left_crop / video_right_crop", "video_cam", "video_eye=both",
            "coord", "calib", "fps",
            "schema_revision=egodex_v1+sync_v2+tactile_v3+complete_frames_v4", "tactile_included",
            "alignment_clock", "common_interval_start_ns/end_ns",
            "alignment_quality", "min_*_coverage", "max_*_skew_ms",
            "all_exported_frames_complete=True", "complete_frame_quality",
            "complete_frame_required_routes", "controller_to_wrist_calibration",
            "tactile_schema", "tactile_value_count=369", "tactile_values_semantics",
            "tactile_alignment", "tactile_*_cellmap_sha256",
            "tactile_finger_order", "tactile_finger_mapping_schema/source",
            "tactile_to_manus_source_node_ids", "tactile_physical_active_count=144",
            "tactile_palm_present=False",
        ],
    },
    "clocks": {
        "align": "recv_qpc_ns/perf_counter/steady-clock preferred; whole-session recv_wall_ns fallback",
        "resample": "PICO timeStampNs",
        "do_not": "NTP PICO OS to Linux for align — not used",
    },
    "layout": {
        "session": "data/sessions/<prefix>/<index>/ — 一条任务的全部采集和处理产物",
        "raw": "data/sessions/<prefix>/<index>/raw/ — PICO/MANUS/触觉/VST 原始数据",
        "aligned": "data/sessions/<prefix>/<index>/aligned.jsonl — 对齐中间产物",
        "export": "data/sessions/<prefix>/<index>/dataset.hdf5 — 训练包 egodex_v1",
        "review": "data/sessions/<prefix>/<index>/review/ — 验收视频和 probe PNG（不进训练）",
        "legacy": "data/raw|tactile_raw|aligned|export|review — 旧数据只读兼容",
        "config": "config/ — 全局标定 calib_wrist / pico_cam / *.mcal",
        "run": ".run/ — 服务日志，非数据集",
    },
    "sensor_rates": {
        "pico_tracking": "~72 Hz (vc1); file=data/sessions/<S>/raw/pico.jsonl",
        "manus_gloves": "~120 Hz (vc1); file=data/sessions/<S>/raw/manus.jsonl",
        "vst_video": "~50 Hz actual (request 60); 2160x810 SBS; vst.h264 + vst.ts.jsonl",
        "tactile_sleeves": "60 Hz requested each side; 369 signed-int16 values; file=data/sessions/<S>/raw/tactile.jsonl",
        "export_resample": "30 Hz default into HDF5",
    },
    "vst_video": {
        "file": "data/sessions/<session>/raw/vst.h264 (elementary H.264, no container)",
        "layout": "sbs_lr left|right",
        "resolution_full": "2160x810 (requested default; matches stream)",
        "resolution_eye": "1080x810 each",
        "crop_left": {"x": 0, "y": 0, "w": 1080, "h": 810},
        "crop_right": {"x": 1080, "y": 0, "w": 1080, "h": 810},
        "fps_requested": 60,
        "fps_actual": "~50 (measure via vst.ts.jsonl / sidecar_fps; vc1≈50.1)",
        "fps_do_not_trust": "OpenCV/ffprobe on bare .h264 often reports 25",
        "optics": "rectified pinhole VST (not native fisheye RAW SBS)",
        "timestamp": "vst.ts.jsonl wall_ns + vst.qpc.ts.jsonl high-resolution host clock per frame",
        "docs": "README.md §3",
    },
}


def _file_info(p: Path) -> dict:
    if not p or not p.exists():
        return {"path": str(p) if p else None, "exists": False}
    st = p.stat()
    info = {"path": str(p), "exists": True, "bytes": st.st_size}
    if p.suffix == ".jsonl" and p.is_file():
        try:
            with open(p, "rb") as f:
                info["lines"] = sum(1 for _ in f)
        except OSError:
            pass
    return info


def describe_session(session: str, data_root: str, logs_dir: str) -> dict:
    paths = resolve_session(session, data_root=data_root, logs_dir=logs_dir)
    if paths is None:
        paths = new_session_paths(Path(data_root) / "sessions", session, data_root=data_root)
        paths["legacy"] = False
        missing = True
    else:
        missing = False
    return {
        "session": session,
        "missing": missing,
        "layout": "legacy" if paths.get("legacy") else paths.get("layout", "data/sessions/<session>"),
        "files": {k: _file_info(paths[k]) for k in
                  ("pico", "manus", "manus_meta", "vst", "vst_ts", "vst_qpc_ts", "tactile", "tactile_meta",
                   "aligned", "export", "overlay", "manifest")},
    }


def print_human(doc: dict):
    print(f"\n======== {doc['session']} ({doc['layout']}) ========")
    for k, info in doc["files"].items():
        if info.get("exists"):
            extra = f", lines={info['lines']}" if "lines" in info else ""
            print(f"  OK  {k:10s} {info['bytes']/1e6:8.2f} MB{extra}  {info['path']}")
        else:
            print(f"  --  {k:10s}  {info.get('path')}")


def main():
    ap = argparse.ArgumentParser(description="会话清单 / 训练包 schema")
    ap.add_argument("session", nargs="?")
    ap.add_argument("--data-root", default=DEFAULT_DATA_ROOT)
    ap.add_argument("--logs-dir", default="logs")
    ap.add_argument("--schema", action="store_true")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--write-manifest", action="store_true")
    args = ap.parse_args()

    if args.schema:
        print(json.dumps(SCHEMA, ensure_ascii=False, indent=2))
        return

    sessions = ([args.session] if args.session
                else list_sessions(args.data_root, args.logs_dir))
    if not sessions:
        print("无会话。见 README.md / docs/PIPELINE_ZH.md", file=sys.stderr)
        print(json.dumps(SCHEMA, ensure_ascii=False, indent=2))
        return

    reports = [describe_session(s, args.data_root, args.logs_dir) for s in sessions]
    if args.write_manifest:
        for s in sessions:
            paths = resolve_session(s, args.data_root, args.logs_dir)
            if paths is None:
                paths = new_session_paths(Path(args.data_root) / "sessions", s,
                                          data_root=args.data_root)
            desc = next(r for r in reports if r["session"] == s)
            print("[manifest]", write_manifest(paths, extra={"status": desc, "schema": SCHEMA}))

    if args.json:
        print(json.dumps({"schema": SCHEMA, "sessions": reports},
                         ensure_ascii=False, indent=2))
    else:
        print("文档: README.md | docs/PIPELINE_ZH.md | docs/CALIB_QA_ZH.md")
        print("训练包: egodex_v1  (python3 data_catalog.py --schema)")
        for r in reports:
            print_human(r)


if __name__ == "__main__":
    main()
