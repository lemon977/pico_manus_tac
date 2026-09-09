#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""make_visualization.py — 为 egodex_v1 HDF5 训练包生成静态可视化 HTML。

用法:
  python3 make_visualization.py data/export/vc1.hdf5 -o data/export/vc1_visualization.html
  python3 make_visualization.py data/export/vc1.hdf5 --video-dir data/raw/vc1 --review data/review/review_vc1_overlay.mp4
"""
from __future__ import annotations

import argparse
import html
import json
import os
from pathlib import Path
from urllib.parse import quote

import h5py
import numpy as np


NS = 1_000_000_000

COLORS = [
    "#e24a4a", "#4a90e2", "#4ae290", "#e2c84a",
    "#a04ae2", "#e24a90", "#4ae2e2", "#e28f4a",
    "#8e44ad", "#16a085", "#d35400", "#2c3e50",
]


def fmt_size(num_bytes: int) -> str:
    n = float(num_bytes)
    for unit in ("B", "KB", "MB", "GB"):
        if abs(n) < 1024.0 or unit == "GB":
            return f"{n:.2f} {unit}" if unit != "B" else f"{int(n)} B"
        n /= 1024.0
    return f"{n:.2f} GB"


def rel_path(from_file: Path, to_file: Path) -> str:
    """生成从 HTML 到媒体文件的相对 URL 路径。"""
    try:
        return quote(os.path.relpath(str(to_file.resolve()), str(from_file.parent.resolve())), safe="/")
    except Exception:
        return quote(str(to_file), safe="/")


# ------------------------------------------------------------------ SVG 绘图

def make_line_chart(data: np.ndarray, labels: list[str], title: str,
                    y_label: str = "", x_label: str = "帧序号",
                    width: int = 900, height: int = 260,
                    top_pad: int = 50, bot_pad: int = 50,
                    left_pad: int = 60, right_pad: int = 20) -> str:
    """data: (T, D)；绘制多维度折线 SVG。"""
    if data.ndim == 1:
        data = data.reshape(-1, 1)
    n_frames, n_dim = data.shape
    plot_w = width - left_pad - right_pad
    plot_h = height - top_pad - bot_pad

    flat = data.reshape(-1)
    y_min = float(np.nanmin(flat))
    y_max = float(np.nanmax(flat))
    if y_min == y_max:
        y_min -= 1.0
        y_max += 1.0

    dx = plot_w / max(n_frames - 1, 1)

    lines = []
    for d in range(n_dim):
        ys = data[:, d]
        pts = []
        for i, v in enumerate(ys):
            if np.isnan(v):
                continue
            x = left_pad + i * dx
            y = top_pad + plot_h - ((float(v) - y_min) / (y_max - y_min) * plot_h)
            pts.append(f"{x:.2f},{y:.2f}")
        if pts:
            color = COLORS[d % len(COLORS)]
            lines.append(f"<polyline points='{' '.join(pts)}' fill='none' stroke='{color}' stroke-width='1.5' />")

    axis = [
        f"<line x1='{left_pad}' y1='{top_pad + plot_h}' x2='{left_pad + plot_w}' y2='{top_pad + plot_h}' stroke='#999' stroke-width='1' />",
        f"<line x1='{left_pad}' y1='{top_pad}' x2='{left_pad}' y2='{top_pad + plot_h}' stroke='#999' stroke-width='1' />",
        f"<text x='{left_pad - 8}' y='{top_pad - 4}' text-anchor='end' font-size='11' fill='#555'>{y_max:.3f}</text>",
        f"<text x='{left_pad - 8}' y='{top_pad + plot_h + 14}' text-anchor='end' font-size='11' fill='#555'>{y_min:.3f}</text>",
    ]
    if x_label:
        axis.append(f"<text x='{left_pad + plot_w / 2}' y='{height - 6}' text-anchor='middle' font-size='12' fill='#333'>{html.escape(x_label)}</text>")
    if y_label:
        axis.append(f"<text x='15' y='{height / 2}' text-anchor='middle' font-size='12' fill='#333' transform='rotate(-90 15 {height / 2})'>{html.escape(y_label)}</text>")

    legend_x = left_pad
    legend_y = 18
    legend = []
    for d, label in enumerate(labels):
        color = COLORS[d % len(COLORS)]
        legend.append(f"<rect x='{legend_x + d * 90}' y='{legend_y - 10}' width='10' height='10' fill='{color}' />")
        legend.append(f"<text x='{legend_x + d * 90 + 14}' y='{legend_y}' font-size='10' fill='#333'>{html.escape(str(label))}</text>")

    return f"""<div class="chart-title">{html.escape(title)}</div>
<svg viewBox="0 0 {width} {height}" class="chart">
  {' '.join(lines)}
  {' '.join(axis)}
  {' '.join(legend)}
</svg>"""


def make_step_chart(mask: np.ndarray, title: str, labels: list[str],
                    width: int = 900, height: int = 120) -> str:
    """把 (T, D) bool 矩阵画成阶跃条。"""
    if mask.ndim == 1:
        mask = mask.reshape(-1, 1)
    n_frames, n_dim = mask.shape
    plot_w = width - 60 - 20
    plot_h = height - 40 - 30
    top_pad, left_pad, bot_pad = 30, 60, 30
    dx = plot_w / max(n_frames - 1, 1)
    row_h = plot_h / n_dim

    rects = []
    for d in range(n_dim):
        y = top_pad + d * row_h + 2
        h = row_h - 4
        color = COLORS[d % len(COLORS)]
        in_seg = False
        seg_start = 0
        for i, v in enumerate(mask[:, d]):
            if v and not in_seg:
                in_seg = True
                seg_start = i
            elif not v and in_seg:
                x1 = left_pad + seg_start * dx
                x2 = left_pad + (i - 1) * dx
                rects.append(f"<rect x='{x1:.2f}' y='{y:.2f}' width='{max(x2 - x1, 1):.2f}' height='{h:.2f}' fill='{color}' opacity='0.85' />")
                in_seg = False
        if in_seg:
            x1 = left_pad + seg_start * dx
            x2 = left_pad + (n_frames - 1) * dx
            rects.append(f"<rect x='{x1:.2f}' y='{y:.2f}' width='{max(x2 - x1, 1):.2f}' height='{h:.2f}' fill='{color}' opacity='0.85' />")

    texts = []
    for d, label in enumerate(labels):
        y = top_pad + d * row_h + row_h / 2 + 3
        texts.append(f"<text x='{left_pad - 8}' y='{y:.2f}' text-anchor='end' font-size='11' fill='#555'>{html.escape(str(label))}</text>")

    axis = [
        f"<line x1='{left_pad}' y1='{top_pad + plot_h}' x2='{left_pad + plot_w}' y2='{top_pad + plot_h}' stroke='#999' stroke-width='1' />",
        f"<text x='{left_pad + plot_w / 2}' y='{height - 4}' text-anchor='middle' font-size='12' fill='#333'>帧序号</text>",
    ]

    return f"""<div class="chart-title">{html.escape(title)}</div>
<svg viewBox="0 0 {width} {height}" class="chart">
  {' '.join(rects)}
  {' '.join(texts)}
  {' '.join(axis)}
</svg>"""


def make_topdown_traj(head: np.ndarray, lw: np.ndarray, rw: np.ndarray,
                      width: int = 520, height: int = 520) -> str:
    """绘制 X-Z 平面轨迹（俯视）。X 向前，Z 向上为屏幕 y。"""
    pts = np.vstack([head[:, [0, 2]], lw[:, [0, 2]], rw[:, [0, 2]]])
    x_min, x_max = float(np.nanmin(pts[:, 0])), float(np.nanmax(pts[:, 0]))
    z_min, z_max = float(np.nanmin(pts[:, 1])), float(np.nanmax(pts[:, 1]))
    margin = max(x_max - x_min, z_max - z_min) * 0.15 + 0.05
    x_min -= margin; x_max += margin
    z_min -= margin; z_max += margin

    pad = 50
    plot_w = width - 2 * pad
    plot_h = height - 2 * pad

    def pxz(arr: np.ndarray, idx: int) -> tuple[float, float]:
        x = pad + (arr[idx, 0] - x_min) / (x_max - x_min) * plot_w
        z = pad + plot_h - (arr[idx, 2] - z_min) / (z_max - z_min) * plot_h
        return x, z

    def poly(arr: np.ndarray, color: str) -> str:
        pts = [f"{pxz(arr, i)[0]:.2f},{pxz(arr, i)[1]:.2f}" for i in range(len(arr)) if not np.isnan(arr[i]).any()]
        return f"<polyline points='{' '.join(pts)}' fill='none' stroke='{color}' stroke-width='1.2' />"

    def mark(arr: np.ndarray, color: str, label: str) -> list[str]:
        x0, z0 = pxz(arr, 0)
        x1, z1 = pxz(arr, len(arr) - 1)
        return [
            f"<circle cx='{x0:.2f}' cy='{z0:.2f}' r='4' fill='{color}' opacity='0.5' />",
            f"<circle cx='{x1:.2f}' cy='{z1:.2f}' r='4' fill='{color}' />",
            f"<text x='{x1 + 6:.2f}' y='{z1 + 3:.2f}' font-size='10' fill='#333'>{html.escape(label)}</text>",
        ]

    head_line = poly(head, "#e24a4a")
    lw_line = poly(lw, "#4a90e2")
    rw_line = poly(rw, "#4ae290")
    h_marks = mark(head, "#e24a4a", "头")
    lw_marks = mark(lw, "#4a90e2", "左腕")
    rw_marks = mark(rw, "#4ae290", "右腕")

    axis = [
        f"<line x1='{pad}' y1='{pad + plot_h}' x2='{pad + plot_w}' y2='{pad + plot_h}' stroke='#999' stroke-width='1' />",
        f"<line x1='{pad}' y1='{pad}' x2='{pad}' y2='{pad + plot_h}' stroke='#999' stroke-width='1' />",
        f"<text x='{width - pad}' y='{pad + plot_h + 16}' text-anchor='end' font-size='11' fill='#555'>X 前 (m)</text>",
        f"<text x='{pad + 4}' y='{pad - 8}' font-size='11' fill='#555'>Z 上 (m)</text>",
    ]

    return f"""<svg viewBox="0 0 {width} {height}" class="chart">
  {head_line} {lw_line} {rw_line}
  {' '.join(h_marks)} {' '.join(lw_marks)} {' '.join(rw_marks)}
  {' '.join(axis)}
</svg>"""


# ------------------------------------------------------------------ 统计与 HTML 片段

def pose_stats(arr: np.ndarray) -> dict:
    """arr: (T,7) pos+quat。"""
    pos = arr[:, :3]
    quat = arr[:, 3:]
    return {
        "pos_min": [float(np.nanmin(pos[:, i])) for i in range(3)],
        "pos_max": [float(np.nanmax(pos[:, i])) for i in range(3)],
        "pos_mean": [float(np.nanmean(pos[:, i])) for i in range(3)],
        "quat_mean": [float(np.nanmean(quat[:, i])) for i in range(4)],
    }


def fingertip_distances(joints: np.ndarray, joint_names: list[str]) -> tuple[np.ndarray, list[str]]:
    """返回每帧每个 fingertip 到 wrist root(node0) 的距离。"""
    tips = [i for i, n in enumerate(joint_names) if str(n).upper().endswith("TIP")]
    names = [joint_names[i] for i in tips]
    wrist = joints[:, 0, :]  # (T,3)
    dists = np.linalg.norm(joints[:, tips, :] - wrist[:, None, :], axis=2)  # (T, n_tip)
    return dists, names


def attr_table_rows(attrs: dict, hide_keys: set | None = None) -> str:
    """把 HDF5 attrs 渲染成表格行。"""
    hide_keys = hide_keys or set()
    rows = []
    descriptions = {
        "schema": "数据格式 schema",
        "schema_alias": "旧名兼容",
        "coord": "坐标系约定",
        "fps": "导出帧率",
        "hands": "手部节点模式",
        "n_joint": "单手关节数",
        "joint_names": "关节名列表（JSON）",
        "calib": "手柄→腕外参文件",
        "gate_ms": "MANUS 最近邻门限",
        "gap_ms": "断点分段门限",
        "n_segments": "切出的连续段数",
        "created": "导出时间",
        "source_pico": "原始 PICO 文件",
        "source_manus": "原始 MANUS 文件",
        "hand_frame": "手指坐标系约定",
        "ctrl_convention": "手柄约定",
        "pose_layout": "位姿向量排布",
        "timestamp_clock": "时间轴时钟源",
        "video_path": "原始 VST SBS 视频",
        "video_ts_path": "VST 墙钟 sidecar",
        "video_stereo": "是否双目",
        "video_layout": "SBS 布局",
        "video_width": "整幅宽",
        "video_height": "整幅高",
        "video_eye_width": "单眼宽",
        "video_eye_height": "单眼高",
        "video_eye": "训练主用眼",
        "video_left_crop": "左眼 crop",
        "video_right_crop": "右眼 crop",
        "video_cam_path": "相机参数文件",
        "video_cam": "左右相机内外参（JSON）",
        "video_note": "视频使用说明",
    }
    for k, v in attrs.items():
        if k in hide_keys:
            continue
        desc = descriptions.get(k, "")
        if isinstance(v, str):
            val = html.escape(v)
            if (k.endswith("_crop") or k in ("video_cam", "joint_names")) and len(v) > 40:
                try:
                    obj = json.loads(v)
                    val = f"""<details class='json-fold'><summary class='json-summary'>{k} JSON</summary><pre class='json-pre'>{html.escape(json.dumps(obj, ensure_ascii=False, indent=2))}</pre></details>"""
                except Exception:
                    pass
        elif isinstance(v, (np.ndarray, list, tuple)):
            val = html.escape(str(v)[:200])
        else:
            val = html.escape(str(v))
        rows.append(f"""<tr><td class='attr-k'>{html.escape(k)}</td><td class='attr-desc'>{html.escape(desc)}</td><td class='attr-val'>{val}</td></tr>""")
    return "\n".join(rows)


def dataset_tree(f: h5py.File) -> str:
    parts = []
    for name, ds in f.items():
        shape = " × ".join(str(s) for s in ds.shape)
        dtype = str(ds.dtype)
        nbytes = ds.size * ds.dtype.itemsize
        parts.append(
            f"""<div class='tree-row tree-ds' style='padding-left:18px'>
  <span class='tree-icon'>▸</span>
  <span class='tree-name'>{html.escape(name)}</span>
  <span class='tree-cn'>{html.escape(_ds_cn(name))}</span>
  <span class='tree-meta'>{shape} · {dtype} · {fmt_size(nbytes)}</span>
</div>""")
    return "\n".join(parts)


def _ds_cn(name: str) -> str:
    return {
        "timestamp_ns": "重采样时间轴",
        "recv_wall_ns": "墙钟（挂视频）",
        "segment_id": "片段号",
        "video_frame_idx": "VST 整帧号",
        "video_frame_idx_left": "左眼帧号",
        "video_frame_idx_right": "右眼帧号",
        "head_pose": "头部位姿",
        "left_wrist_pose": "左手腕位姿",
        "right_wrist_pose": "右手腕位姿",
        "left_hand_joints": "左手关节局部坐标",
        "right_hand_joints": "右手关节局部坐标",
        "left_hand_valid": "左手有效掩码",
        "right_hand_valid": "右手有效掩码",
    }.get(name, "")


# ------------------------------------------------------------------ 主渲染

def render(f: h5py.File, h5_path: Path, out_path: Path, video_dir: Path | None = None,
           review_path: Path | None = None, task_name: str = "") -> str:
    attrs = {k: f.attrs[k] for k in f.attrs.keys()}
    fps = float(attrs.get("fps", 30.0))
    T = f["timestamp_ns"].shape[0]
    dur = T / fps
    n_joint = int(attrs.get("n_joint", 25))

    # 位姿
    head = f["head_pose"][:]
    lw = f["left_wrist_pose"][:]
    rw = f["right_wrist_pose"][:]
    lh = f["left_hand_joints"][:]
    rh = f["right_hand_joints"][:]
    lhv = f["left_hand_valid"][:]
    rhv = f["right_hand_valid"][:]
    vidx = f["video_frame_idx"][:]
    seg = f["segment_id"][:]
    ts = f["timestamp_ns"][:]

    joint_names = json.loads(attrs.get("joint_names", "[]"))
    if not joint_names or len(joint_names) != n_joint:
        joint_names = [f"j{i}" for i in range(n_joint)]

    # 指尖距离
    lh_dist, lh_tip_names = fingertip_distances(lh, joint_names)
    rh_dist, rh_tip_names = fingertip_distances(rh, joint_names)

    # 位姿统计
    hs = pose_stats(head); lws = pose_stats(lw); rws = pose_stats(rw)

    # 视频匹配率
    vmatch = 100.0 * (vidx >= 0).mean() if vidx.size else 0.0
    vmatch_first = int(vidx[vidx >= 0].min()) if (vidx >= 0).any() else -1
    vmatch_last = int(vidx[vidx >= 0].max()) if (vidx >= 0).any() else -1

    # 片段
    seg_info = []
    for s in np.unique(seg):
        cnt = int((seg == s).sum())
        seg_info.append(f"段{s}: {cnt}帧 ({cnt / fps:.1f}s)")

    # 图表
    charts = []
    charts.append(make_line_chart(head[:, :3], ["X", "Y", "Z"], "头部位置 (m)", y_label="m"))
    charts.append(make_line_chart(lw[:, :3], ["X", "Y", "Z"], "左手腕位置 (m)", y_label="m"))
    charts.append(make_line_chart(rw[:, :3], ["X", "Y", "Z"], "右手腕位置 (m)", y_label="m"))
    charts.append(make_step_chart(np.column_stack([lhv, rhv]), "左右手有效掩码", ["左手", "右手"]))
    if lh_tip_names:
        charts.append(make_line_chart(lh_dist, lh_tip_names, "左手指尖到腕根距离 (m)", y_label="m"))
    if rh_tip_names:
        charts.append(make_line_chart(rh_dist, rh_tip_names, "右手指尖到腕根距离 (m)", y_label="m"))
    if vidx.size:
        charts.append(make_line_chart(vidx.astype(float), ["video_frame_idx"], "视频帧号匹配", y_label="idx"))

    topdown = make_topdown_traj(head, lw, rw)

    # 视频卡片
    video_cards = []
    if video_dir and video_dir.is_dir():
        for label, name in [("SBS 整幅", "vst.mp4"), ("左眼", "vst_left.mp4"), ("右眼", "vst_right.mp4")]:
            p = video_dir / name
            if p.is_file():
                video_cards.append((label, rel_path(out_path, p)))
    if review_path and review_path.is_file():
        video_cards.append(("review 叠图", rel_path(out_path, review_path)))

    video_html = ""
    if video_cards:
        cards = "\n".join(
            f"""<div class='video-card'><h4>{html.escape(label)}</h4><video controls preload='metadata'><source src='{html.escape(url)}' type='video/mp4'></video></div>"""
            for label, url in video_cards
        )
        video_html = f"""<h2>相机 / 视频</h2>
<div class='video-grid'>
  {cards}
</div>"""

    in_name = h5_path.stem
    if task_name and task_name != in_name:
        title = f"{task_name} — {in_name}"
    else:
        title = in_name

    charts_html = '\n'.join(charts)

    html_doc = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<title>PICO + MANUS 采集数据 — {html.escape(title)}</title>
<style>
  body {{ font-family: "PingFang SC", "Microsoft YaHei", sans-serif; margin: 0; padding: 32px 24px; background: #fff; color: #1a1a1a; line-height: 1.6; }}
  .container {{ max-width: 1100px; margin: 0 auto; }}
  h1 {{ font-size: 22px; font-weight: 600; margin: 0 0 6px; }}
  .file-name {{ color: #888; font-size: 13px; margin-bottom: 32px; }}
  h2 {{ font-size: 16px; font-weight: 600; margin: 36px 0 14px; padding-bottom: 6px; border-bottom: 1px solid #e8e8e8; }}
  h3.group-title {{ font-size: 14px; font-weight: 600; color: #555; margin: 24px 0 10px; }}
  .brief {{ list-style: none; padding: 16px 20px; margin: 0; background: #f8f9fa; border-radius: 6px; }}
  .brief li {{ margin: 4px 0; font-size: 15px; }}
  .brief li::before {{ content: "· "; color: #999; }}
  .field-grid {{ display: grid; grid-template-columns: repeat(auto-fill, minmax(300px, 1fr)); gap: 12px; }}
  .field-card {{ border: 1px solid #e8e8e8; border-radius: 6px; padding: 14px 16px; }}
  .field-label {{ font-size: 15px; font-weight: 600; margin-bottom: 2px; }}
  .field-path {{ font-size: 11px; color: #aaa; font-family: ui-monospace, monospace; margin-bottom: 8px; word-break: break-all; }}
  .field-desc {{ font-size: 13px; color: #444; margin: 0 0 10px; }}
  .field-meta {{ display: flex; flex-wrap: wrap; gap: 6px; }}
  .tag {{ font-size: 11px; color: #666; background: #f0f0f0; padding: 2px 8px; border-radius: 3px; white-space: nowrap; }}
  .chart {{ width: 100%; height: auto; margin: 8px 0 20px; }}
  .chart-title {{ font-size: 13px; color: #555; margin: 12px 0 4px; }}
  .section-note {{ color: #888; font-size: 13px; margin: -6px 0 10px; }}
  .video-grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 16px; }}
  .video-card {{ text-align: center; }}
  .video-card h4 {{ font-size: 13px; font-weight: 500; margin: 0 0 8px; color: #555; }}
  .video-card video {{ width: 100%; max-width: 420px; border-radius: 4px; background: #000; }}
  .h5-panel {{ border: 1px solid #ddd; border-radius: 8px; overflow: hidden; margin: 12px 0 24px; }}
  .h5-file-head {{ display: flex; align-items: center; gap: 10px; padding: 12px 16px; background: #f5f5f5; border-bottom: 1px solid #e0e0e0; }}
  .h5-file-icon {{ font-size: 18px; }}
  .h5-file-name {{ font-family: ui-monospace, monospace; font-size: 14px; font-weight: 600; }}
  .h5-file-size {{ margin-left: auto; font-size: 12px; color: #888; }}
  .h5-section-label {{ font-size: 12px; color: #888; padding: 10px 16px 4px; letter-spacing: 0.5px; }}
  .attr-table {{ width: 100%; border-collapse: collapse; font-size: 13px; }}
  .attr-table th {{ text-align: left; padding: 6px 16px; background: #fafafa; border-bottom: 1px solid #e8e8e8; color: #666; font-weight: 500; font-size: 12px; }}
  .attr-table td {{ padding: 8px 16px; border-top: 1px solid #f0f0f0; vertical-align: top; }}
  .attr-k {{ font-family: ui-monospace, monospace; color: #1a5276; width: 180px; white-space: nowrap; }}
  .attr-desc {{ color: #666; width: 220px; font-size: 12px; line-height: 1.5; }}
  .attr-val {{ color: #222; word-break: break-word; }}
  .json-pre {{ font-size: 11px; line-height: 1.55; white-space: pre-wrap; word-break: break-all; max-height: 220px; overflow: auto; margin: 6px 0 0; padding: 10px 12px; background: #f7f8fa; border-radius: 5px; color: #444; }}
  .json-summary {{ cursor: pointer; user-select: none; list-style: none; font-size: 13px; color: #444; }}
  .json-summary::-webkit-details-marker {{ display: none; }}
  .json-summary::before {{ content: '▸ '; color: #bbb; font-size: 11px; }}
  details[open] > .json-summary::before {{ content: '▾ '; }}
  .h5-tree {{ padding: 8px 16px 16px; font-family: ui-monospace, monospace; font-size: 13px; line-height: 1.9; }}
  .tree-row {{ display: flex; align-items: baseline; gap: 8px; flex-wrap: wrap; }}
  .tree-icon {{ flex-shrink: 0; width: 16px; text-align: center; font-size: 12px; }}
  .tree-name {{ color: #1a5276; font-weight: 500; }}
  .tree-cn {{ font-size: 12px; color: #888; font-family: "PingFang SC", sans-serif; }}
  .tree-cn::before {{ content: "— "; }}
  .tree-meta {{ font-size: 11px; color: #999; margin-left: auto; }}
  .note-warn {{ color: #b35c00; font-size: 13px; margin: 8px 0; }}
  .note-info {{ color: #555; font-size: 13px; margin: 8px 0; }}
</style>
</head>
<body>
<div class="container">
<h1>PICO + MANUS 采集数据</h1>
<p class="file-name">{html.escape(h5_path.name)}</p>

<h2>概览</h2>
<ul class='brief'>
  <li>schema: <strong>{html.escape(str(attrs.get('schema', '?')))}</strong> ({html.escape(str(attrs.get('schema_alias', '')))})</li>
  <li>时长: {dur:.1f} 秒（{T} 帧，{fps:g} fps）</li>
  <li>内容: 头部 + 双手腕位姿、双手 25 关节（腕局部系）、双目 VST 帧号</li>
  <li>坐标系: {html.escape(str(attrs.get('coord', '?')))}</li>
  <li>位姿排布: {html.escape(str(attrs.get('pose_layout', '?')))}</li>
  <li>手部关节: {n_joint} 个 · 模式 {html.escape(str(attrs.get('hands', '?')))}</li>
  <li>分段: {html.escape('，'.join(seg_info))}</li>
  <li>视频帧匹配率: {vmatch:.0f}%（匹配帧号 {vmatch_first} ~ {vmatch_last}）</li>
  <li>导出时间: {html.escape(str(attrs.get('created', '?')))}</li>
</ul>

<h2>文件结构</h2>
<p class="section-note">HDF5 里实际存了什么。</p>

<div class="h5-panel">
  <div class="h5-file-head">
    <span class="h5-file-icon">📄</span>
    <span class="h5-file-name">{html.escape(h5_path.name)}</span>
    <span class="h5-file-size">{fmt_size(h5_path.stat().st_size)}</span>
  </div>

  <div class="h5-section-label">文件属性（根 attrs）</div>
  <table class="attr-table attr-root">
    <thead><tr><th>属性</th><th>说明</th><th>值</th></tr></thead>
    <tbody>
      {attr_table_rows(attrs)}
    </tbody>
  </table>

  <div class="h5-section-label">数据树</div>
  <div class="h5-tree">
    {dataset_tree(f)}
  </div>
</div>

<h2>字段说明</h2>
<h3 class='group-title'>世界系位姿</h3>
<div class='field-grid'>
  <div class='field-card'>
    <div class='field-label'>头部位姿</div>
    <div class='field-path'>head_pose</div>
    <p class='field-desc'>头部在世界系中的位置 + 朝向。pos(xyz) + quat(xyzw)。</p>
    <div class='field-meta'><span class='tag'>形状 {T} × 7</span><span class='tag'>单位 m / 四元数</span><span class='tag'>X {hs['pos_min'][0]:.3f} ~ {hs['pos_max'][0]:.3f}</span><span class='tag'>Y {hs['pos_min'][1]:.3f} ~ {hs['pos_max'][1]:.3f}</span><span class='tag'>Z {hs['pos_min'][2]:.3f} ~ {hs['pos_max'][2]:.3f}</span></div>
  </div>
  <div class='field-card'>
    <div class='field-label'>左手腕位姿</div>
    <div class='field-path'>left_wrist_pose</div>
    <p class='field-desc'>左手腕世界位姿，已叠 T_calib（安装偏移）。</p>
    <div class='field-meta'><span class='tag'>形状 {T} × 7</span><span class='tag'>单位 m / 四元数</span><span class='tag'>X {lws['pos_min'][0]:.3f} ~ {lws['pos_max'][0]:.3f}</span><span class='tag'>Y {lws['pos_min'][1]:.3f} ~ {lws['pos_max'][1]:.3f}</span><span class='tag'>Z {lws['pos_min'][2]:.3f} ~ {lws['pos_max'][2]:.3f}</span></div>
  </div>
  <div class='field-card'>
    <div class='field-label'>右手腕位姿</div>
    <div class='field-path'>right_wrist_pose</div>
    <p class='field-desc'>右手腕世界位姿，已叠 T_calib（安装偏移）。</p>
    <div class='field-meta'><span class='tag'>形状 {T} × 7</span><span class='tag'>单位 m / 四元数</span><span class='tag'>X {rws['pos_min'][0]:.3f} ~ {rws['pos_max'][0]:.3f}</span><span class='tag'>Y {rws['pos_min'][1]:.3f} ~ {rws['pos_max'][1]:.3f}</span><span class='tag'>Z {rws['pos_min'][2]:.3f} ~ {rws['pos_max'][2]:.3f}</span></div>
  </div>
</div>

<h3 class='group-title'>手部与视频</h3>
<div class='field-grid'>
  <div class='field-card'>
    <div class='field-label'>左手关节</div>
    <div class='field-path'>left_hand_joints</div>
    <p class='field-desc'>腕局部系下的 25 个关节坐标（相对 MANUS 腕根）。</p>
    <div class='field-meta'><span class='tag'>形状 {T} × {n_joint} × 3</span><span class='tag'>单位 m</span><span class='tag'>有效 {100*lhv.mean():.0f}%</span></div>
  </div>
  <div class='field-card'>
    <div class='field-label'>右手关节</div>
    <div class='field-path'>right_hand_joints</div>
    <p class='field-desc'>腕局部系下的 25 个关节坐标（相对 MANUS 腕根）。</p>
    <div class='field-meta'><span class='tag'>形状 {T} × {n_joint} × 3</span><span class='tag'>单位 m</span><span class='tag'>有效 {100*rhv.mean():.0f}%</span></div>
  </div>
  <div class='field-card'>
    <div class='field-label'>视频帧号</div>
    <div class='field-path'>video_frame_idx</div>
    <p class='field-desc'>SBS VST 中与该训练帧最近的视频帧号；-1 表示未匹配。</p>
    <div class='field-meta'><span class='tag'>形状 ({T},)</span><span class='tag'>int32</span><span class='tag'>匹配率 {vmatch:.0f}%</span></div>
  </div>
</div>

<h2>运动曲线</h2>
{charts_html}

<h2>俯视轨迹（X 前 · Z 上）</h2>
<p class="section-note">同一水平面投影：红=头、蓝=左腕、绿=右腕；空心圆=起点，实心圆=终点。</p>
{topdown}

{video_html}

</div>
</body>
</html>"""
    return html_doc


def main():
    ap = argparse.ArgumentParser(description="为 egodex_v1 HDF5 生成静态可视化 HTML")
    ap.add_argument("hdf5", help="输入 HDF5 路径")
    ap.add_argument("-o", "--out", help="输出 HTML 路径；默认与 HDF5 同名 .html")
    ap.add_argument("--video-dir", help="包含 vst.mp4 / vst_left.mp4 / vst_right.mp4 的目录")
    ap.add_argument("--review", help="review 叠图 mp4 路径")
    ap.add_argument("--task", default="", help="任务名/会话名，用于标题")
    args = ap.parse_args()

    h5_path = Path(args.hdf5)
    out_path = Path(args.out) if args.out else h5_path.with_suffix(".html")
    video_dir = Path(args.video_dir) if args.video_dir else None
    review_path = Path(args.review) if args.review else None

    if not h5_path.is_file():
        raise SystemExit(f"找不到 HDF5: {h5_path}")

    with h5py.File(h5_path, "r") as f:
        doc = render(f, h5_path, out_path, video_dir=video_dir, review_path=review_path, task_name=args.task)

    out_path.write_text(doc, encoding="utf-8")
    print(f"[viz] 已生成 {out_path}")


if __name__ == "__main__":
    main()
