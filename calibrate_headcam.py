#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""calibrate_headcam.py — 用"手柄3D(头局部)↔画面像素"多帧对应, solvePnP 解出头->相机精确 [R|t]。

为什么: 工厂内参可信、每帧头位姿可信, 唯一没标准的是"头->相机固定安装角"。
单帧凑出的轴变换只在那个头姿附近准, 头一俯仰就飘。这里用多帧、不同头姿的对应,
一次解出精确 [R|t](把"转换后头局部系"直接映射到相机系), 之后 overlay 全程像素级。

两步:
  1) 标注(需要显示器+cv2 GUI): 自动挑手柄在画面里、且头姿有俯仰差异的若干帧,
     你在每帧上点【左手柄原点】再点【右手柄原点】; 存 clicks.json。
       python3 calibrate_headcam.py annotate ALIGNED VIDEO [--n 15] [--out clicks.json]
     交互键: 左键点2个点(先左后右) / n=该帧不清楚跳过 / r=重点本帧 / q=提前结束
  2) 求解(任意机器): 读 clicks + 对齐数据, solvePnP -> head_to_cam_solved.json。
       python3 calibrate_headcam.py solve ALIGNED --clicks clicks.json \
           [--cam config/pico_cam/vst_cam.json] [--out config/pico_cam/head_to_cam_solved.json]

标注这一步需要有画面的机器。若 181 无显示器: 把 ALIGNED / VIDEO / sidecar 拉到本机跑 annotate,
再把 clicks.json 拷回 181 跑 solve(纯数值, 无需显示器)。
"""
import argparse
import bisect
import json
import os
import sys

import numpy as np
import cv2

from pico_controller_viz import _aligned_to_bundle, pose_to_T
from make_review_video import load_aligned, find_sidecar, load_sidecar
from overlay_skeleton import load_cam, C_CONV2RAW


def _walls(frames):
    w = [r["pico_wall_ns"] for r in frames]
    return w, w[0]


def head_local_refs(rec):
    """该对齐帧 -> (head_T, {side: 头局部3D手柄原点}); 仅含 active 的手柄。"""
    b = _aligned_to_bundle(rec)
    if not b.get("head"):
        return None, {}
    head_T = pose_to_T(*b["head"])
    Rh, th = head_T[:3, :3], head_T[:3, 3]
    out = {}
    for side in ("left", "right"):
        r = b[side].get("pico_ref")
        if not r:
            continue
        P = pose_to_T(*r)[:3, 3]
        if float(np.dot(P, P)) < 1e-8:          # 未握持(原点)
            continue
        out[side] = Rh.T @ (np.asarray(P, float) - th)
    return head_T, out


def factory_guess(p_local, cam):
    """工厂外参粗投影(仅用于标注时给参考标记 / 选帧)。"""
    fx, fy, cx, cy = cam["K"]
    pc = cam["R"] @ (C_CONV2RAW @ p_local) + cam["t"]
    if pc[2] <= 1e-4:
        return None
    return (fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy)


def select_frames(frames, cam, crop_w, crop_h, n):
    """挑 n 帧: 双手柄 active、工厂粗投影大致在画面内, 且按头俯仰(前向z)均匀取以增加多样性。"""
    cand = []
    for idx in range(0, len(frames), 2):
        head_T, refs = head_local_refs(frames[idx])
        if head_T is None or "left" not in refs or "right" not in refs:
            continue
        guesses = {s: factory_guess(refs[s], cam) for s in ("left", "right")}
        if any(g is None for g in guesses.values()):
            continue
        inframe = all(-250 <= g[0] <= crop_w + 250 and -250 <= g[1] <= crop_h + 250
                      for g in guesses.values())
        if not inframe:
            continue
        fwd_z = (head_T[:3, :3] @ np.array([1.0, 0.0, 0.0]))[2]   # 头前向的竖直分量=俯仰
        cand.append((fwd_z, idx))
    if not cand:
        return []
    cand.sort(key=lambda x: x[0])
    if len(cand) <= n:
        return [i for _, i in cand]
    pick = np.linspace(0, len(cand) - 1, n).round().astype(int)
    return [cand[k][1] for k in pick]


def get_frame(cap, sc_walls, walls, wall0, fps_v, nframes_v, cx0, cx1, want_wall):
    if sc_walls is not None:
        vidx = int(np.searchsorted(sc_walls, want_wall))
        if vidx > 0 and (vidx >= sc_walls.size or
                         abs(sc_walls[vidx - 1] - want_wall) < abs(sc_walls[vidx] - want_wall)):
            vidx -= 1
    else:
        vidx = int((want_wall - wall0) / 1e9 * fps_v)
    if not (0 <= vidx < (nframes_v or 10 ** 9)):
        return None
    cap.set(cv2.CAP_PROP_POS_FRAMES, vidx)
    ok, fr = cap.read()
    if not ok:
        return None
    return fr[:, cx0:cx1].copy()


def open_video(video):
    if video.lower().endswith(".h264"):
        import subprocess, tempfile
        mp4 = os.path.join(tempfile.gettempdir(), os.path.basename(video)[:-5] + "_seek.mp4")
        if not os.path.isfile(mp4):
            subprocess.run(["ffmpeg", "-y", "-fflags", "+genpts", "-i", video, "-c", "copy", mp4],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if os.path.isfile(mp4):
            video = mp4
    cap = cv2.VideoCapture(video)
    return cap


def cmd_annotate(args):
    frames = load_aligned(args.aligned)
    if not frames:
        print("对齐文件无有效帧", file=sys.stderr); sys.exit(1)
    walls, wall0 = _walls(frames)

    cap = open_video(args.video)
    if not cap.isOpened():
        print(f"打不开视频 {args.video}", file=sys.stderr); sys.exit(1)
    fps_v = cap.get(cv2.CAP_PROP_FPS) or 30.0
    nframes_v = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    if nframes_v < 0:
        nframes_v = 0
    vw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)); vh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cx0 = 0 if args.eye == "left" else vw // 2
    cx1 = vw // 2 if args.eye == "left" else vw
    crop_w, crop_h = cx1 - cx0, vh
    cam = load_cam(args.cam, args.eye, crop_w, crop_h)

    sc_walls = None
    if not args.no_sidecar:
        sc_path = find_sidecar(args.video, args.sidecar)
        if sc_path:
            sc_walls = load_sidecar(sc_path)
            sc_walls = sc_walls if sc_walls.size else None

    idxs = select_frames(frames, cam, crop_w, crop_h, args.n)
    if not idxs:
        print("没挑到合适帧(手柄未在画面内?)。请确认录制时手全程在视野里。", file=sys.stderr)
        sys.exit(1)
    print(f"[annotate] 选了 {len(idxs)} 帧。每帧: 先点【左手柄原点】再点【右手柄原点】; "
          f"n=跳过 r=重点 q=结束")

    state = {"clicks": []}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(state["clicks"]) < 2:
            state["clicks"].append((float(x), float(y)))

    win = "annotate (L then R)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(win, on_mouse)

    items = []
    for k, idx in enumerate(idxs):
        rec = frames[idx]
        want = walls[idx]
        fr = get_frame(cap, sc_walls, walls, wall0, fps_v, nframes_v, cx0, cx1, want)
        if fr is None:
            continue
        head_T, refs = head_local_refs(rec)
        guesses = {s: factory_guess(refs[s], cam) for s in refs}
        state["clicks"] = []
        while True:
            disp = fr.copy()
            for s, col in (("left", (255, 220, 0)), ("right", (0, 165, 255))):
                g = guesses.get(s)
                if g:
                    cv2.drawMarker(disp, (int(g[0]), int(g[1])), col,
                                   cv2.MARKER_TILTED_CROSS, 22, 1)
            for i, (cxp, cyp) in enumerate(state["clicks"]):
                col = (255, 220, 0) if i == 0 else (0, 165, 255)
                cv2.circle(disp, (int(cxp), int(cyp)), 6, col, -1)
            t = (want - wall0) / 1e9
            msg = f"[{k+1}/{len(idxs)}] t={t:.1f}s  clicked={len(state['clicks'])}/2  " \
                  f"(细叉=工厂粗投影参考)  L then R | n skip  r redo  q done"
            cv2.putText(disp, msg, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            cv2.imshow(win, disp)
            key = cv2.waitKey(20) & 0xFF
            if key == ord('q'):
                cap.release(); cv2.destroyAllWindows()
                _save_clicks(args, items); return
            if key == ord('n'):
                break
            if key == ord('r'):
                state["clicks"] = []
            if len(state["clicks"]) == 2:
                items.append({"t": t, "idx": idx,
                              "left": list(state["clicks"][0]),
                              "right": list(state["clicks"][1])})
                print(f"  记录 t={t:.1f}s L={state['clicks'][0]} R={state['clicks'][1]}")
                break
    cap.release(); cv2.destroyAllWindows()
    _save_clicks(args, items)


def _save_clicks(args, items):
    out = {"eye": args.eye, "aligned": os.path.basename(args.aligned),
           "cam": args.cam, "items": items}
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print(f"[annotate] 存 {len(items)} 帧对应 -> {args.out}")


def cmd_solve(args):
    with open(args.clicks, encoding="utf-8") as f:
        clk = json.load(f)
    eye = clk.get("eye", "left")
    if not args.out:
        args.out = f"config/pico_cam/head_to_cam_solved_{eye}.json"
    frames = load_aligned(args.aligned)
    walls, wall0 = _walls(frames)

    # 分辨率: 用 native 内参缩到裁剪分辨率; 裁剪宽=native VST 每眼(=vst 宽/2), 高=vst 高。
    # 这里直接按 overlay 默认(1080x810)取; 与 annotate 时一致。
    crop_w, crop_h = args.crop_w, args.crop_h
    cam = load_cam(args.cam, eye, crop_w, crop_h)
    fx, fy, cx, cy = cam["K"]
    Kmat = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])

    obj, img = [], []
    for it in clk["items"]:
        idx = it.get("idx")
        rec = frames[idx] if idx is not None else None
        if rec is None:                      # 兜底: 按时间找
            wt = wall0 + int(it["t"] * 1e9)
            j = bisect.bisect_left(walls, wt); rec = frames[min(max(j, 0), len(frames) - 1)]
        _, refs = head_local_refs(rec)
        for side in ("left", "right"):
            if it.get(side) and side in refs:
                obj.append(refs[side]); img.append(it[side])
    obj = np.asarray(obj, np.float64); img = np.asarray(img, np.float64)
    if len(obj) < 4:
        print(f"有效对应只有 {len(obj)} 个, 至少要 4(建议 >=8)。", file=sys.stderr); sys.exit(1)

    # 初值: 工厂外参 R_ext@C, t_ext
    R_init = cam["R"] @ C_CONV2RAW
    rvec0, _ = cv2.Rodrigues(R_init)
    tvec0 = cam["t"].reshape(3, 1).astype(np.float64)
    dist = np.zeros(5)
    ok, rvec, tvec = cv2.solvePnP(obj, img, Kmat, dist, rvec0.copy(), tvec0.copy(),
                                  useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        print("solvePnP 失败", file=sys.stderr); sys.exit(1)
    rvec, tvec = cv2.solvePnPRefineLM(obj, img, Kmat, dist, rvec, tvec)

    proj, _ = cv2.projectPoints(obj, rvec, tvec, Kmat, dist)
    proj = proj.reshape(-1, 2)
    err = np.linalg.norm(proj - img, axis=1)
    R, _ = cv2.Rodrigues(rvec)
    out = {"_note": "solvePnP 求得的 头(转换后局部系)->相机 [R|t]; overlay --headcam 用它投影",
           "eye": eye, "crop_w": crop_w, "crop_h": crop_h,
           "R": R.tolist(), "t": tvec.reshape(3).tolist(),
           "n_points": int(len(obj)),
           "reproj_px_mean": float(err.mean()), "reproj_px_max": float(err.max())}
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print(f"[solve] 用 {len(obj)} 点; 重投影误差 均值={err.mean():.1f}px 最大={err.max():.1f}px")
    print(f"[solve] 存 -> {args.out}")
    if err.mean() > 25:
        print("[solve] 均值>25px: 标注可能有点飘或帧太少/头姿多样性不够, 可多标几帧再求解。")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)

    a = sub.add_parser("annotate", help="点选手柄像素")
    a.add_argument("aligned"); a.add_argument("video")
    a.add_argument("--eye", default="left", choices=["left", "right"])
    a.add_argument("--n", type=int, default=15, help="挑多少帧标注")
    a.add_argument("--cam", default="config/pico_cam/vst_cam.json")
    a.add_argument("--sidecar", default=None); a.add_argument("--no-sidecar", action="store_true")
    a.add_argument("--out", default="clicks.json")
    a.set_defaults(func=cmd_annotate)

    s = sub.add_parser("solve", help="solvePnP 求 [R|t]")
    s.add_argument("aligned"); s.add_argument("--clicks", default="clicks.json")
    s.add_argument("--cam", default="config/pico_cam/vst_cam.json")
    s.add_argument("--crop-w", type=int, default=1080); s.add_argument("--crop-h", type=int, default=810)
    s.add_argument("--out", default="",
                   help="默认 config/pico_cam/head_to_cam_solved_{eye}.json")
    s.set_defaults(func=cmd_solve)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
