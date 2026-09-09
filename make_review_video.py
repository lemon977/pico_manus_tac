#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""make_review_video.py — 把「真实 VST 视频」与「3D 重建」渲染成一个并排 MP4，
方便离线逐帧核对 3D 姿态是否正确（远程无显示器时尤其有用）。

左：头显 VST 视频（默认取左目）；右：由对齐数据渲染的 3D（头/双手柄世界系 +
火柴人骨架 + 真实 MANUS 手）。两侧按同一时间轴推进。

时间同步说明：
  - **优先用逐帧时间戳 sidecar**（`vst_x.ts.jsonl`，service 采集时自动生成）：
    每视频帧带到达 wall 时间，与 pose `recv_wall_ns` 同一时钟，按 wall 最近邻精确
    映射，**无需手调 offset**（此时 --offset 仅作额外微调）。自动在视频同目录找，
    或 --sidecar 指定；--no-sidecar 可强制关闭。
  - **无 sidecar 时回退**：按「起点 + 可调偏移 + fps」近似对齐。用 --offset 秒微调
    （正=视频相对姿态更晚）；先 --start/--duration 渲一小段反复试。找 offset 技巧：
    录制开头做个明显动作（挥手/拍手），调 offset 让左右两边同刻发生。
    （可用 video_offset.py 自动估 offset。）

用法：
  python3 make_review_video.py \
      data/aligned/S.jsonl data/raw/S/vst.h264 \
      -o data/review/review_S.mp4 --out-fps 30 --overlay
  # 左= VST(+可选骨架) 右=3D；两侧同一墙钟时间轴、同一 out-fps（实时）
  # 先试 10 秒： --start 20 --duration 10
"""
import argparse
import json
import sys
import bisect

import numpy as np
import cv2

from pico_controller_viz import _aligned_to_bundle, pose_to_T, build_body_points, _elbow_point, BODY_DIMS


def _mpl():
    """延迟加载 matplotlib(仅并排 3D 渲染需要; overlay 只复用 sidecar 工具不必装齐)。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    return plt


def find_sidecar(video_path, override=None):
    """定位逐帧时间戳 sidecar: 显式 --sidecar 优先, 否则视频同名 .ts.jsonl。"""
    import os
    if override:
        return override if os.path.isfile(override) else None
    base = video_path
    for ext in (".h264", ".mp4", ".mkv", ".avi"):
        if base.endswith(ext):
            base = base[:-len(ext)]
            break
    cand = base + ".ts.jsonl"
    return cand if os.path.isfile(cand) else None


def load_sidecar(path):
    """读 sidecar -> 每视频帧到达 wall_ns 的 np 数组(行序=帧序)。
    兼容两种格式: 纯数字每行, 或 {"wall_ns":..} JSON 每行。"""
    walls = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                if line[0] == "{":
                    walls.append(int(json.loads(line).get("wall_ns")))
                else:
                    walls.append(int(line))
            except (ValueError, TypeError, json.JSONDecodeError):
                continue
    return np.array(walls, dtype=np.int64)


def sidecar_fps(sc_walls):
    """由 sidecar wall 时间估真实采集帧率。裸 H.264 的 OpenCV/ffprobe 常误报 25。"""
    if sc_walls is None or len(sc_walls) < 2:
        return None
    span = (int(sc_walls[-1]) - int(sc_walls[0])) / 1e9
    if span <= 0:
        return None
    return (len(sc_walls) - 1) / span


def ensure_seekable_mp4(video_path):
    """裸 .h264 无容器时戳, cv2 seek 不稳; 转临时 mp4(-c copy) 便于按帧号取帧。"""
    import os
    import subprocess
    import tempfile
    if not video_path.lower().endswith(".h264"):
        return video_path
    mp4 = os.path.join(tempfile.gettempdir(),
                       os.path.basename(video_path)[:-5] + "_seek.mp4")
    if not os.path.isfile(mp4) or os.path.getmtime(video_path) > os.path.getmtime(mp4):
        print(f"[review] 裸 h264 转 mp4 便于 seek: {mp4}")
        subprocess.run(["ffmpeg", "-y", "-fflags", "+genpts", "-i", video_path,
                        "-c", "copy", mp4],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return mp4 if os.path.isfile(mp4) else video_path


def load_aligned(path):
    frames = []
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
            frames.append(rec)
    frames.sort(key=lambda r: r["pico_wall_ns"])
    return frames


def scene_bounds(frames):
    pts = []
    for rec in frames[::5]:
        b = _aligned_to_bundle(rec)
        if b.get("head"):
            pts.append(b["head"][0])
        for side in ("left", "right"):
            r = b[side].get("pico_ref")
            if r:
                pts.append(r[0])
    if not pts:
        return np.array([-0.6, -0.6, 0.0]), np.array([0.6, 0.6, 1.2])
    a = np.array(pts, float)
    lo = a.min(0) - 0.25
    hi = a.max(0) + 0.25
    # 保证立方体等比例
    c = (lo + hi) / 2
    r = max(hi - lo) / 2
    return c - r, c + r


def render_3d(ax, bundle, lo, hi):
    ax.clear()
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(lo[2], hi[2])
    ax.set_xlabel("X fwd"); ax.set_ylabel("Y left"); ax.set_zlabel("Z up")
    ax.view_init(elev=18, azim=-60)

    head = bundle.get("head")
    refs = {}
    for side in ("left", "right"):
        r = bundle[side].get("pico_ref")
        if r:
            refs[side] = pose_to_T(*r)

    # 世界原点参考轴
    L = 0.15
    for vec, col in (((L, 0, 0), "r"), ((0, L, 0), "g"), ((0, 0, L), "b")):
        ax.plot([0, vec[0]], [0, vec[1]], [0, vec[2]], color=col, lw=1)

    # 火柴人骨架（头/躯干/双臂）
    if head:
        head_T = pose_to_T(*head)
        hp = head_T[:3, 3]
        ax.scatter([hp[0]], [hp[1]], [hp[2]], c="k", s=60)
        try:
            body = build_body_points(hp, head_T[:3, :3], refs)
            chain = [body["head"], body["neck"], body["chest"]]
            cp = np.array(chain)
            ax.plot(cp[:, 0], cp[:, 1], cp[:, 2], c="0.3", lw=3)
            for side, sh_key in (("left", "shoulderL"), ("right", "shoulderR")):
                if side not in refs:
                    continue
                sh = body[sh_key]
                wr = refs[side][:3, 3]
                el = _elbow_point(sh, wr, BODY_DIMS["upper_arm"], BODY_DIMS["forearm"], body["up"])
                arm = np.array([body["chest"], sh, el, wr])
                ax.plot(arm[:, 0], arm[:, 1], arm[:, 2],
                        c="c" if side == "left" else "orange", lw=3)
        except Exception:  # noqa: BLE001
            pass

    # 手柄原点 + MANUS 手套腕/手
    for side in ("left", "right"):
        if side not in refs:
            continue
        T = refs[side]
        wp = T[:3, 3]
        col = "c" if side == "left" else "orange"
        ax.scatter([wp[0]], [wp[1]], [wp[2]], c=col, s=40, marker="s")
        nodes = bundle[side].get("manus_nodes")
        parents = bundle[side].get("manus_parents")
        cnt = bundle[side].get("manus_count") or 0
        if nodes is not None and cnt > 0:
            nn = min(cnt, len(nodes))
            local = np.asarray(nodes)[:nn, :3]
            world = (T[:3, :3] @ local.T).T + T[:3, 3]
            if parents is not None:
                for i in range(nn):
                    p = int(parents[i]) if i < len(parents) else -1
                    if 0 <= p < nn:
                        ax.plot([world[i, 0], world[p, 0]],
                                [world[i, 1], world[p, 1]],
                                [world[i, 2], world[p, 2]], c=col, lw=1)


def fig_to_bgr(fig):
    fig.canvas.draw()
    buf = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
    w, h = fig.canvas.get_width_height()
    img = buf.reshape(h, w, 3)
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)


def main():
    ap = argparse.ArgumentParser(description="真实VST视频 + 3D重建 并排 MP4")
    ap.add_argument("aligned", help="align_pico_manus.py 的对齐 JSONL(建议 --full)")
    ap.add_argument("video", help="VST mp4/h264")
    ap.add_argument("-o", "--out", required=True, help="输出 MP4")
    ap.add_argument("--offset", type=float, default=0.0,
                    help="视频相对姿态的时间偏移(秒), 正=视频更晚。默认0")
    ap.add_argument("--out-fps", type=float, default=30.0,
                    help="输出帧率(默认30)。左右两侧同一时间轴按此时长推进(实时)")
    ap.add_argument("--start", type=float, default=0.0, help="从姿态时间轴第几秒开始渲染")
    ap.add_argument("--duration", type=float, default=0.0, help="渲染时长(秒), 0=到结尾")
    ap.add_argument("--eye", choices=["left", "right", "both"], default="left",
                    help="取哪只眼的画面(默认左目)")
    ap.add_argument("--panel", type=int, default=720, help="每侧画面高度(像素)")
    ap.add_argument("--overlay", action="store_true",
                    help="左侧 VST 上叠加骨架(与 overlay_skeleton 同一投影)")
    ap.add_argument("--no-manus", action="store_true", help="叠骨架时不画手指(只留手柄)")
    ap.add_argument("--calib", default="config/calib_wrist.json")
    ap.add_argument("--no-calib", action="store_true")
    ap.add_argument("--cam", default="config/pico_cam/vst_cam.json")
    ap.add_argument("--headcam", default="config/pico_cam/head_to_cam_solved.json")
    ap.add_argument("--sidecar", default=None,
                    help="逐帧时间戳 sidecar(vst_x.ts.jsonl); 缺省自动在视频同目录找。"
                         "有则按 wall 时间精确同步, --offset 仅作额外微调")
    ap.add_argument("--no-sidecar", action="store_true", help="忽略 sidecar, 强制用 offset+fps")
    args = ap.parse_args()

    frames = load_aligned(args.aligned)
    if not frames:
        print("[review] 对齐文件无有效帧", file=sys.stderr)
        sys.exit(1)
    walls = [r["pico_wall_ns"] for r in frames]
    wall0 = walls[0]
    dur_pose = (walls[-1] - wall0) / 1e9
    lo, hi = scene_bounds(frames)

    # 逐帧时间戳 sidecar: 有则按 wall 时间精确映射视频帧(无需靠 offset+fps 猜)。
    sc_walls = None
    if not args.no_sidecar:
        sc_path = find_sidecar(args.video, args.sidecar)
        if sc_path:
            sc_walls = load_sidecar(sc_path)
            if sc_walls.size == 0:
                sc_walls = None
            else:
                print(f"[review] 用 sidecar 精确同步: {sc_path} ({sc_walls.size} 帧); "
                      f"--offset 仅作额外微调")
    if sc_walls is None:
        print(f"[review] 无 sidecar, 用 offset({args.offset:+.2f}s)+fps 近似同步")

    video = ensure_seekable_mp4(args.video)
    cap = cv2.VideoCapture(video)
    if not cap.isOpened():
        print(f"[review] 打不开视频 {args.video}", file=sys.stderr)
        sys.exit(1)
    fps_容器 = cap.get(cv2.CAP_PROP_FPS) or 0.0
    nframes_v = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    if nframes_v < 0:
        nframes_v = 0
    vw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    vh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    real_fps = sidecar_fps(sc_walls)
    fps_v = real_fps if real_fps else (fps_容器 or 60.0)
    if real_fps:
        print(f"[review] 视频 {vw}x{vh}  容器标称={fps_容器:.1f}fps  真实≈{real_fps:.2f}fps "
              f"(sidecar {sc_walls.size}帧)")
    else:
        print(f"[review] 视频 {vw}x{vh}  容器标称={fps_容器:.1f}fps(无 sidecar, 可能不准)")
    print(f"[review] 姿态时长≈{dur_pose:.1f}s  输出={args.out_fps:.0f}fps(左右同轴实时)")

    # 可选: 左侧叠骨架(延迟 import, 避免与 overlay_skeleton 循环依赖)
    ov_draw = None
    ov_K = ov_cam = ov_calib = None
    if args.overlay:
        from overlay_skeleton import (draw_overlay, load_cam, load_headcam,  # noqa: E402
                                      DEF_FX, DEF_FY, DEF_CX, DEF_CY, EYE_W, EYE_H)
        from export_dataset import load_calib
        import os
        crop_w_eye = (vw if args.eye == "both" else vw // 2)
        sx = crop_w_eye / EYE_W
        sy = vh / EYE_H
        ov_K = (DEF_FX * sx, DEF_FY * sy, DEF_CX * sx, DEF_CY * sy)
        eye = "left" if args.eye == "both" else args.eye
        if args.headcam and os.path.isfile(args.headcam) and args.cam and os.path.isfile(args.cam):
            ov_cam = load_headcam(args.headcam, args.cam, eye, crop_w_eye, vh)
            print(f"[review] 左幅叠骨架: PnP {args.headcam}")
        elif args.cam and os.path.isfile(args.cam):
            ov_cam = load_cam(args.cam, eye, crop_w_eye, vh)
            print(f"[review] 左幅叠骨架: 工厂外参 {args.cam}")
        else:
            print("[review] 左幅叠骨架: 无 cam 文件, 跳过投影标定(可能不准)")
        if not args.no_calib and args.calib and os.path.isfile(args.calib):
            ov_calib = load_calib(args.calib)
        ov_draw = draw_overlay

    t0 = args.start
    t1 = dur_pose if args.duration <= 0 else min(dur_pose, args.start + args.duration)
    K = int((t1 - t0) * args.out_fps)
    if K <= 0:
        print("[review] 渲染区间为空", file=sys.stderr)
        sys.exit(1)

    H = args.panel
    plt = _mpl()
    fig = plt.figure(figsize=(H / 100.0, H / 100.0), dpi=100)
    ax = fig.add_subplot(111, projection="3d")

    # 视频左目裁剪范围
    if args.eye == "both":
        cx0, cx1 = 0, vw
    elif args.eye == "left":
        cx0, cx1 = 0, vw // 2
    else:
        cx0, cx1 = vw // 2, vw
    # 左侧视频面板固定尺寸(否则视频未就绪的占位帧与就绪帧尺寸不一致,
    # 会导致 VideoWriter 静默丢帧)
    crop_w = cx1 - cx0
    panel_l_w = int(crop_w * (H / vh))

    writer = None
    last_vidx = -1
    vframe = None
    for k in range(K):
        t = t0 + k / args.out_fps
        # 姿态：最近邻
        wt = wall0 + int(t * 1e9)
        j = bisect.bisect_left(walls, wt)
        j = min(max(j, 0), len(frames) - 1)
        if j > 0 and abs(walls[j - 1] - wt) < abs(walls[j] - wt):
            j -= 1
        bundle = _aligned_to_bundle(frames[j])
        render_3d(ax, bundle, lo, hi)
        panel_r = fig_to_bgr(fig)
        panel_r = cv2.resize(panel_r, (H, H))

        # 视频帧: 有 sidecar 按 wall 时间最近邻(精确), 否则 offset+fps(近似)
        if sc_walls is not None:
            want = wall0 + int((t - args.offset) * 1e9)   # offset 作额外微调
            vidx = int(np.searchsorted(sc_walls, want))
            if vidx > 0 and (vidx >= sc_walls.size or
                             abs(sc_walls[vidx - 1] - want) < abs(sc_walls[vidx] - want)):
                vidx -= 1
        else:
            vidx = int((t - args.offset) * fps_v)
        if 0 <= vidx < (nframes_v or 10 ** 9):
            if vidx != last_vidx:
                cap.set(cv2.CAP_PROP_POS_FRAMES, vidx)
                ok, fr = cap.read()
                if ok:
                    vframe = fr
                last_vidx = vidx
        if vframe is not None:
            crop = vframe[:, cx0:cx1].copy()
            if ov_draw is not None:
                ov_draw(crop, bundle, ov_K, "xfwd",
                        skeleton=True, cam=ov_cam, manus=not args.no_manus,
                        calib=ov_calib)
            panel_l = cv2.resize(crop, (panel_l_w, H))
        else:
            panel_l = np.zeros((H, panel_l_w, 3), np.uint8)

        combo = np.hstack([panel_l, panel_r])
        cv2.putText(combo, f"t={t:6.2f}s  offset={args.offset:+.2f}s  vidx={vidx}",
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        left_tag = "VST+SKEL" if ov_draw is not None else "REAL VST"
        cv2.putText(combo, left_tag, (10, combo.shape[0] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(combo, "3D RECON", (panel_l.shape[1] + 10, combo.shape[0] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        if writer is None:
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(args.out, fourcc, args.out_fps,
                                     (combo.shape[1], combo.shape[0]))
        writer.write(combo)
        if k % 30 == 0:
            print(f"\r[review] {k+1}/{K} ({100*(k+1)/K:.0f}%)", end="", file=sys.stderr, flush=True)

    if writer:
        writer.release()
    cap.release()
    plt.close(fig)
    print(f"\n[review] 写出 {args.out}", file=sys.stderr)


if __name__ == "__main__":
    main()
