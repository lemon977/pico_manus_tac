#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""overlay_skeleton.py — 把 3D 骨架/手直接画到真实 VST 视频画面上(单目叠加)。

思路(最简版, 不需要去畸变):
  1. VST 每眼是矫正后的针孔图(app 用 76.35°x61.05° FOV 报内参), 直接用针孔 K。
  2. 骨架点都在世界系(X前 Y左 Z上, 与 aligned/pico jsonl 一致)。
  3. 每帧相机≈头位姿; 世界点先转到头局部系, 再按"相机沿头 +X 看"投影到像素:
         p_local = R_head^T (P_world - head_pos)
         p_cam   = [-p_local_y, -p_local_z, p_local_x]   # 右=-Y_head 下=-Z_head 前=+X_head
         u = cx + fx * p_cam_x/p_cam_z ;  v = cy + fy * p_cam_y/p_cam_z   (p_cam_z>0 才可见)
  4. 画手柄/手腕/MANUS 骨架/手臂; 写 MP4。

内参默认取自设备(trackingData GetCameraIntrinsicsfor4U, 左眼 1080x810), 可 --fx/--fy/--cx/--cy 覆盖。
时间同步复用 make_review_video 的 sidecar 逻辑(service 采集自带 vst_x.ts.jsonl, 精确; 否则 --offset)。

用法:
  # 验证(出几张叠加 PNG 看对不对):
  python3 overlay_skeleton.py aligned_calib.jsonl logs/vst_calib.h264 --probe 8 --probe-dir /tmp/ov
  # 出视频(默认按 sidecar 真实帧率实时写出; --out-fps 15=抽帧仍实时):
  python3 overlay_skeleton.py data/aligned/S.jsonl data/raw/S/vst.h264 \
      -o data/review/overlay_S.mp4 --out-fps 30
"""
import argparse
import bisect
import json
import os
import sys

import numpy as np
import cv2

from pico_controller_viz import (_aligned_to_bundle, pose_to_T, build_body_points,
                                 _elbow_point, BODY_DIMS)
from make_review_video import find_sidecar, load_sidecar, load_aligned, sidecar_fps
from export_dataset import load_calib, hand_local
from pico_retarget import quat_to_mat

# 设备内参(左眼, 矫正针孔, 1080x810; 来自 trackingData GetCameraIntrinsicsfor4U 76.35x61.05)
DEF_FX, DEF_FY, DEF_CX, DEF_CY = 686.83, 686.87, 539.75, 404.5
EYE_W, EYE_H = 1080, 810

# 转换后世界头局部系(X前Y左Z上) -> 工厂外参期望的相机头系(近似)。
# 精确叠加请优先用 calibrate_headcam.py 解出的 head_to_cam_solved.json。
C_CONV2RAW = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float)


def load_cam(path, eye, crop_w, crop_h):
    """读 vst_cam.json -> dict(R,t, K=(fx,fy,cx,cy) 缩放到裁剪分辨率)。"""
    with open(path, encoding="utf-8") as f:
        cam = json.load(f)
    intr = cam["intrinsics_native"]
    sx = crop_w / intr["native_w"]
    sy = crop_h / intr["native_h"]
    K = (intr["fx"] * sx, intr["fy"] * sy, intr["cx"] * sx, intr["cy"] * sy)
    side = cam[eye]
    return {"R": np.array(side["R"], float), "t": np.array(side["t"], float), "K": K}


def project_ext(P_world, head_T, cam):
    """把世界点投到像素。
    cam["solved"]=True: 用 solvePnP 求得的 head(转换后局部)->相机 [R|t](精确);
    否则用工厂外参 + 常量 C 近似: p_cam = R_ext·C·p_head_local + t_ext。"""
    Rh, th = head_T[:3, :3], head_T[:3, 3]
    p_local = Rh.T @ (np.asarray(P_world, float) - th)   # 转换后头局部
    if cam.get("solved"):
        pc = cam["R"] @ p_local + cam["t"]
    else:
        pc = cam["R"] @ (C_CONV2RAW @ p_local) + cam["t"]
    if pc[2] <= 1e-4:
        return None, False
    fx, fy, cx, cy = cam["K"]
    return (float(fx * pc[0] / pc[2] + cx), float(fy * pc[1] / pc[2] + cy)), True


def load_headcam(path, cam_path, eye, crop_w, crop_h):
    """读 solvePnP 标定的 head->camera [R|t], 内参用 vst_cam.json(缩到裁剪分辨率)。

    path 可为:
      - 单眼文件 (含 "eye"/"R"/"t")
      - 双目文件 (含 "left"/"right" 各一套 R,t)
    右眼不要套左眼单目解；缺对应眼时抛 ValueError 以便回退工厂外参。
    """
    base = load_cam(cam_path, eye, crop_w, crop_h)
    with open(path, encoding="utf-8") as f:
        s = json.load(f)
    # 双目汇总文件
    if eye in ("left", "right") and isinstance(s.get(eye), dict) and "R" in s[eye]:
        side = s[eye]
        return {"R": np.array(side["R"], float), "t": np.array(side["t"], float),
                "K": base["K"], "solved": True}
    solved_eye = str(s.get("eye") or s.get("camera") or "left").lower()
    if eye != solved_eye and eye != "both":
        raise ValueError(
            f"headcam 标定眼={solved_eye}, 请求眼={eye}; "
            f"请用 head_to_cam_solved_{eye}.json 或工厂外参"
        )
    return {"R": np.array(s["R"], float), "t": np.array(s["t"], float),
            "K": base["K"], "solved": True}


def resolve_headcam_path(preferred: str, eye: str):
    """按眼别找 PnP 文件: 显式路径 → solved_{eye}.json → solved.json(仅左眼兼容)。"""
    import os
    cands = []
    if preferred:
        cands.append(preferred)
    cands.append(f"config/pico_cam/head_to_cam_solved_{eye}.json")
    if eye == "left":
        cands.append("config/pico_cam/head_to_cam_solved.json")
    for p in cands:
        if p and os.path.isfile(p):
            return p
    return None


def mount_R(pitch_deg=0.0, yaw_deg=0.0, roll_deg=0.0):
    """相机相对头的安装旋转(头局部系 X前 Y左 Z上)。
    pitch>0=镜头下俯; yaw>0=左偏; roll>0=顺时针。返回 3x3(把头局部点转到相机安装系)。"""
    p, y, r = np.radians([pitch_deg, yaw_deg, roll_deg])
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    cr, sr = np.cos(r), np.sin(r)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])     # 俯仰(绕 Y 左)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])     # 偏航(绕 Z 上)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])     # 滚转(绕 X 前)
    return Rz @ Ry @ Rx


def project(P_world, head_T, K, axes="xfwd", Rmount=None):
    """世界点 -> 左眼像素(u,v)与可见性。head_T: 4x4 头位姿。axes 控制相机看向哪个头轴。"""
    R = head_T[:3, :3]
    t = head_T[:3, 3]
    pl = R.T @ (np.asarray(P_world, float) - t)     # 头局部
    if Rmount is not None:
        pl = Rmount.T @ pl                           # 叠加相机安装角
    x, y, z = pl
    # 相机坐标: +Zc=前(光轴), +Xc=右, +Yc=下
    if axes == "xfwd":       # 头 +X=前 +Y=左 +Z=上
        pc = np.array([-y, -z, x])
    elif axes == "zfwd":     # 头 +Z=前 +X=右 +Y=上(Unity 风格)
        pc = np.array([x, -y, z])
    elif axes == "negzfwd":  # 头 -Z=前
        pc = np.array([-x, -y, -z])
    else:
        pc = np.array([-y, -z, x])
    if pc[2] <= 1e-4:
        return None, False
    u = K[0] * pc[0] / pc[2] + K[2]
    v = K[1] * pc[1] / pc[2] + K[3]
    return (float(u), float(v)), True


def _draw_pt(img, uv, color, r=7, label=None):
    if uv is None:
        return
    u, v = int(round(uv[0])), int(round(uv[1]))
    if -50 <= u < img.shape[1] + 50 and -50 <= v < img.shape[0] + 50:
        cv2.circle(img, (u, v), r, color, -1)
        if label:
            cv2.putText(img, label, (u + 8, v - 8), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, color, 2)


def _draw_seg(img, a, b, color, w=2):
    if a is None or b is None:
        return
    cv2.line(img, (int(round(a[0])), int(round(a[1]))),
             (int(round(b[0])), int(round(b[1]))), color, w)


def draw_overlay(img, bundle, K, axes="xfwd", skeleton=True, Rmount=None, cam=None,
                 manus=True, hands=None, calib=None):
    """在左眼图 img 上画骨架/手柄/MANUS。返回可见关键点数(用于自动挑轴)。
    cam!=None 时用真实工厂外参精确投影, 否则用"头原点+可调俯角"近似。
    manus=False 时不画 MANUS 手指(只留手柄原点+手臂), 便于干净核对对齐。
    calib: load_calib() 的 {side:(pos,quat)}; 给则 MANUS 腕挂到 手柄∘T_calib(默认单位阵)。
    hands: frozenset({'left'}|{'right'}|both); None=both。"""
    from record_control import parse_hands
    allow = parse_hands(hands) if hands is not None else parse_hands("both")
    head = bundle.get("head")
    if not head:
        return 0
    head_T = pose_to_T(*head)

    def PJ(P):
        if cam is not None:
            return project_ext(P, head_T, cam)
        return project(P, head_T, K, axes, Rmount)

    vis = 0
    refs = {}
    for side in ("left", "right"):
        if side not in allow:
            continue
        r = bundle[side].get("pico_ref")
        if r:
            refs[side] = pose_to_T(*r)

    # 手臂火柴人(可选)
    if skeleton:
        try:
            hp = head_T[:3, 3]
            body = build_body_points(hp, head_T[:3, :3], refs)
            def pj(P):
                uv, ok = PJ(P); return uv if ok else None
            chest = pj(body["chest"])
            for side, sh_key, col in (("left", "shoulderL", (255, 200, 0)),
                                       ("right", "shoulderR", (0, 165, 255))):
                if side not in refs:
                    continue
                sh = body[sh_key]; wr = refs[side][:3, 3]
                el = _elbow_point(sh, wr, BODY_DIMS["upper_arm"], BODY_DIMS["forearm"], body["up"])
                pts = [pj(chest_p) for chest_p in (body["chest"], sh, el, wr)]
                for a, b in zip(pts[:-1], pts[1:]):
                    _draw_seg(img, a, b, col, 3)
        except Exception:  # noqa: BLE001
            pass

    # 手柄原点 + MANUS 手
    for side in ("left", "right"):
        if side not in refs:
            continue
        T = refs[side]
        col = (255, 220, 0) if side == "left" else (0, 165, 255)
        uv, ok = PJ(T[:3, 3])
        if ok:
            _draw_pt(img, uv, col, 8, side[0].upper())
            vis += 1
        nodes = bundle[side].get("manus_nodes")
        parents = bundle[side].get("manus_parents")
        cnt = bundle[side].get("manus_count") or 0
        if manus and nodes is not None and cnt > 0:
            Tm = T
            if calib and calib.get(side):
                cp, cq = calib[side]
                Tc = np.eye(4)
                Tc[:3, :3] = quat_to_mat(cq)
                Tc[:3, 3] = cp
                Tm = T @ Tc                      # 腕 = 手柄原点 ∘ T_calib
            nn = min(cnt, len(nodes))
            arr = np.asarray(nodes)[:nn]
            # IMU 下 node0 四元数非 I 时位置轴随腕转; 与标定一致先 hand_local
            local = hand_local(arr) if arr.ndim == 2 and arr.shape[1] >= 7 else arr[:, :3]
            world = (Tm[:3, :3] @ local.T).T + Tm[:3, 3]
            uvs = []
            for i in range(nn):
                uv, ok = PJ(world[i])
                uvs.append(uv if ok else None)
                if ok:
                    _draw_pt(img, uv, col, 3)
                    vis += 1
            if parents is not None:
                for i in range(nn):
                    p = int(parents[i]) if i < len(parents) else -1
                    if 0 <= p < nn:
                        _draw_seg(img, uvs[i], uvs[p], col, 1)
    return vis


def main():
    ap = argparse.ArgumentParser(description="把 3D 骨架叠画到真实 VST 视频(单目)")
    ap.add_argument("aligned", help="align_pico_manus.py 对齐 JSONL(建议 --full)")
    ap.add_argument("video", help="VST h264/mp4")
    ap.add_argument("-o", "--out", default=None, help="输出 MP4")
    ap.add_argument("--eye", choices=["left", "right"], default="left")
    ap.add_argument("--axes", choices=["xfwd", "zfwd", "negzfwd", "auto"], default="auto",
                    help="相机相对头的朝向约定; auto=自动挑可见关键点最多的")
    ap.add_argument("--cam", default="config/pico_cam/vst_cam.json",
                    help="工厂内外参 json; 存在则用它投影, 无则回退俯角近似")
    ap.add_argument("--headcam", default="",
                    help="solvePnP 文件; 空则自动找 head_to_cam_solved_{eye}.json "
                         "(左眼兼容 head_to_cam_solved.json)")
    ap.add_argument("--no-cam", action="store_true", help="忽略外参, 用 --pitch 近似")
    ap.add_argument("--pitch", type=float, default=12.0,
                    help="[近似模式]相机相对头的下俯角(度); 点偏低就调大")
    ap.add_argument("--yaw", type=float, default=0.0, help="相机左右偏角(度)")
    ap.add_argument("--roll", type=float, default=0.0, help="相机滚转角(度)")
    ap.add_argument("--fx", type=float, default=DEF_FX)
    ap.add_argument("--fy", type=float, default=DEF_FY)
    ap.add_argument("--cx", type=float, default=DEF_CX)
    ap.add_argument("--cy", type=float, default=DEF_CY)
    ap.add_argument("--offset", type=float, default=0.0, help="视频相对姿态偏移(秒), 有 sidecar 时仅微调")
    ap.add_argument("--out-fps", type=float, default=0.0,
                    help="输出帧率; 0=跟 sidecar 真实采集率(实时)。"
                         "设为 15/30 会按墙钟抽帧, 播放仍≈实时(不会再慢放)")
    ap.add_argument("--gate-ms", type=float, default=200.0,
                    help="某视频帧离最近 pose 超过此毫秒则只显示原画面不画骨架(开头空段)")
    ap.add_argument("--start", type=float, default=0.0, help="按 pose 时间裁剪起点(秒), 默认整段")
    ap.add_argument("--duration", type=float, default=0.0, help="裁剪时长(秒), 0=到结尾")
    ap.add_argument("--no-skeleton", action="store_true", help="不画手臂火柴人")
    ap.add_argument("--no-manus", action="store_true", help="不画 MANUS 手指(只留手柄原点+手臂, 便于核对)")
    ap.add_argument("--calib", default="config/calib_wrist.json",
                    help="手腕外参 T_calib json; 文件存在则 MANUS 腕挂 手柄∘T_calib, 缺省/缺失=单位阵")
    ap.add_argument("--no-calib", action="store_true", help="强制 T_calib=I(对比用)")
    ap.add_argument("--hands", default="both", choices=["left", "right", "both", "l", "r", "all"],
                    help="只叠哪只手: left / right / both")
    ap.add_argument("--sidecar", default=None)
    ap.add_argument("--no-sidecar", action="store_true")
    ap.add_argument("--probe", type=int, default=0, help=">0: 只导出这么多张叠加 PNG 用于验证")
    ap.add_argument("--probe-dir", default="/tmp/overlay_probe")
    args = ap.parse_args()

    K = (args.fx, args.fy, args.cx, args.cy)
    Rmount = mount_R(args.pitch, args.yaw, args.roll)
    calib = None
    if not args.no_calib and args.calib and os.path.isfile(args.calib):
        calib = load_calib(args.calib)
        print(f"[overlay] 手腕外参 T_calib: {args.calib}")
    elif not args.no_calib:
        print(f"[overlay] 无 {args.calib}, T_calib=I (腕=手柄原点)")
    frames = load_aligned(args.aligned)
    if not frames:
        print("[overlay] 对齐文件无有效帧", file=sys.stderr); sys.exit(1)
    walls = [r["pico_wall_ns"] for r in frames]
    wall0 = walls[0]
    dur_pose = (walls[-1] - wall0) / 1e9

    # 裸 .h264 用 cv2 seek 不可靠且帧数为负: 先转成带时间戳的临时 mp4(-c copy, 秒级)。
    video = args.video
    if video.lower().endswith(".h264"):
        import subprocess, tempfile
        mp4 = os.path.join(tempfile.gettempdir(),
                           os.path.basename(video)[:-5] + "_seek.mp4")
        if not os.path.isfile(mp4) or os.path.getmtime(video) > os.path.getmtime(mp4):
            print(f"[overlay] 裸 h264 转 mp4 便于 seek: {mp4}")
            subprocess.run(["ffmpeg", "-y", "-fflags", "+genpts", "-i", video,
                            "-c", "copy", mp4],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if os.path.isfile(mp4):
            video = mp4

    cap = cv2.VideoCapture(video)
    if not cap.isOpened():
        print(f"[overlay] 打不开视频 {video}", file=sys.stderr); sys.exit(1)
    fps_容器 = cap.get(cv2.CAP_PROP_FPS) or 0.0
    nframes_v = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    if nframes_v < 0:
        nframes_v = 0                                   # 未知: frame_at 用 10**9 兜底
    vw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    vh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cx0 = 0 if args.eye == "left" else vw // 2
    cx1 = vw // 2 if args.eye == "left" else vw
    # 内参按裁剪后分辨率缩放(默认内参对应 1080x810)
    sx = (cx1 - cx0) / EYE_W
    sy = vh / EYE_H
    K = (args.fx * sx, args.fy * sy, args.cx * sx, args.cy * sy)

    sc_walls = None
    if not args.no_sidecar:
        sc_path = find_sidecar(args.video, args.sidecar)
        if sc_path:
            sc_walls = load_sidecar(sc_path)
            sc_walls = sc_walls if sc_walls.size else None
            if sc_walls is not None:
                print(f"[overlay] sidecar 精确同步: {sc_path} ({sc_walls.size}帧)")
    if sc_walls is None:
        print(f"[overlay] 无 sidecar, 用 offset({args.offset:+.2f})+fps 近似")

    real_fps = sidecar_fps(sc_walls)
    fps_v = real_fps if real_fps else (fps_容器 or 60.0)
    if real_fps:
        print(f"[overlay] 视频 {vw}x{vh} 容器标称={fps_容器:.1f}fps 真实≈{real_fps:.2f}fps "
              f"{sc_walls.size}帧; 眼={args.eye} 裁剪宽={cx1-cx0}")
    else:
        print(f"[overlay] 视频 {vw}x{vh}@{fps_v:.1f}fps(容器,可能不准) {nframes_v}帧; "
              f"眼={args.eye} 裁剪宽={cx1-cx0}")
    print(f"[overlay] K=fx{K[0]:.1f} fy{K[1]:.1f} cx{K[2]:.1f} cy{K[3]:.1f}")

    # 投影模式优先级: 该眼的 solvePnP > 工厂外参 > 俯角近似
    cam = None
    headcam_path = resolve_headcam_path(args.headcam, args.eye)
    if not args.no_cam and headcam_path and args.cam and os.path.isfile(args.cam):
        try:
            cam = load_headcam(headcam_path, args.cam, args.eye, cx1 - cx0, vh)
            print(f"[overlay] 标定模式(最准): {headcam_path} (眼={args.eye})")
        except ValueError as e:
            print(f"[overlay] {e}")
            cam = load_cam(args.cam, args.eye, cx1 - cx0, vh)
            print(f"[overlay] 回退工厂外参: {args.cam} (眼={args.eye})")
    elif not args.no_cam and args.cam and os.path.isfile(args.cam):
        cam = load_cam(args.cam, args.eye, cx1 - cx0, vh)
        print(f"[overlay] 工厂外参近似: {args.cam} (眼={args.eye}); "
              f"建议左右眼分别 calibrate_headcam → head_to_cam_solved_{{left,right}}.json")
    else:
        print(f"[overlay] 俯角近似: pitch={args.pitch:.0f}")

    def frame_at(t):
        """取姿态时间 t 对应的视频帧(左/右眼裁剪)。"""
        if sc_walls is not None:
            want = wall0 + int((t - args.offset) * 1e9)
            vidx = int(np.searchsorted(sc_walls, want))
            if vidx > 0 and (vidx >= sc_walls.size or
                             abs(sc_walls[vidx - 1] - want) < abs(sc_walls[vidx] - want)):
                vidx -= 1
        else:
            vidx = int((t - args.offset) * fps_v)
        if not (0 <= vidx < (nframes_v or 10 ** 9)):
            return None
        cap.set(cv2.CAP_PROP_POS_FRAMES, vidx)
        ok, fr = cap.read()
        if not ok:
            return None
        return fr[:, cx0:cx1].copy()

    def bundle_at(t):
        wt = wall0 + int(t * 1e9)
        j = bisect.bisect_left(walls, wt)
        j = min(max(j, 0), len(frames) - 1)
        if j > 0 and abs(walls[j - 1] - wt) < abs(walls[j] - wt):
            j -= 1
        return _aligned_to_bundle(frames[j])

    # 自动挑坐标轴约定: 用中间几帧, 选可见关键点最多者(仅近似模式需要)
    axes = args.axes
    if cam is None and axes == "auto":
        best, bestn = "xfwd", -1
        tmid = args.start + (min(dur_pose, args.start + (args.duration or dur_pose)) - args.start) * 0.5
        for cand in ("xfwd", "zfwd", "negzfwd"):
            tot = 0
            for tt in (tmid, tmid + 0.5, tmid + 1.0):
                b = bundle_at(min(tt, dur_pose))
                img = np.zeros((vh, cx1 - cx0, 3), np.uint8)
                tot += draw_overlay(img, b, K, cand, skeleton=False)
            if tot > bestn:
                best, bestn = cand, tot
        axes = best
        print(f"[overlay] auto 选定 axes={axes} (可见关键点计数={bestn})")

    if args.probe > 0:
        os.makedirs(args.probe_dir, exist_ok=True)
        t1 = dur_pose if args.duration <= 0 else min(dur_pose, args.start + args.duration)
        ts = np.linspace(args.start, max(args.start, t1 - 0.1), args.probe)
        n = 0
        for i, t in enumerate(ts):
            fr = frame_at(t)
            if fr is None:
                continue
            draw_overlay(fr, bundle_at(t), K, axes, skeleton=not args.no_skeleton,
                         Rmount=Rmount, cam=cam, manus=not args.no_manus,
                         hands=args.hands, calib=calib)
            tag = ("PnP" if (cam and cam.get("solved")) else "EXT") if cam else f"pitch={args.pitch:.0f}"
            cv2.putText(fr, f"t={t:.2f}s {tag}", (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            p = os.path.join(args.probe_dir, f"probe_{i:02d}.png")
            cv2.imwrite(p, fr); n += 1
        print(f"[overlay] 写出 {n} 张验证图到 {args.probe_dir}")
        cap.release(); return

    if not args.out:
        print("[overlay] 需要 -o 输出, 或用 --probe", file=sys.stderr); sys.exit(1)

    # 顺序解码; 容器 fps 必须与「写出帧数 / 墙钟跨度」一致, 否则会慢放/快放。
    # --out-fps=0 → 写每一源帧, 用真实采集率; >0 → 按墙钟抽帧到目标 fps(仍实时)。
    native_fps = real_fps if real_fps else (fps_容器 or 30.0)
    out_fps = args.out_fps if args.out_fps > 0 else native_fps
    min_dt = 1.0 / out_fps if out_fps > 0 else 0.0
    next_write_t = None
    gate_ns = int(args.gate_ms * 1e6)
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
    writer = None
    vidx = -1
    n_draw = 0
    n_write = 0
    t_first = t_last = None
    print(f"[overlay] 写出策略: native≈{native_fps:.2f}fps → out={out_fps:.2f}fps "
          f"({'全帧' if args.out_fps <= 0 else '墙钟抽帧'}, 播放≈实时)")
    while True:
        ok, fr = cap.read()
        if not ok:
            break
        vidx += 1
        if sc_walls is not None and vidx < sc_walls.size:
            want = int(sc_walls[vidx]) - int(args.offset * 1e9)
        else:
            want = wall0 + int((vidx / (fps_v or 30.0) - args.offset) * 1e9)
        tpose = (want - wall0) / 1e9
        if tpose < args.start:
            continue
        if args.duration > 0 and tpose > args.start + args.duration:
            break
        if next_write_t is None:
            next_write_t = tpose
        if tpose + 1e-9 < next_write_t:
            continue
        next_write_t = tpose + min_dt
        crop = fr[:, cx0:cx1].copy()
        # 最近邻 pose
        j = bisect.bisect_left(walls, want)
        j = min(max(j, 0), len(frames) - 1)
        if j > 0 and abs(walls[j - 1] - want) < abs(walls[j] - want):
            j -= 1
        near = abs(walls[j] - want) <= gate_ns
        if near:
            draw_overlay(crop, _aligned_to_bundle(frames[j]), K, axes,
                         skeleton=not args.no_skeleton, Rmount=Rmount, cam=cam,
                         manus=not args.no_manus, hands=args.hands, calib=calib)
            n_draw += 1
        tag = ("PnP" if (cam and cam.get("solved")) else "EXT") if cam else f"pitch={args.pitch:.0f}"
        note = "" if near else "  (no pose)"
        cv2.putText(crop, f"t={tpose:6.2f}s {tag}{note}", (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0) if near else (0, 180, 255), 2)
        if writer is None:
            writer = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*"mp4v"),
                                     out_fps, (crop.shape[1], crop.shape[0]))
        writer.write(crop)
        n_write += 1
        t_first = tpose if t_first is None else t_first
        t_last = tpose
        if vidx % 60 == 0:
            print(f"\r[overlay] 帧 {vidx} t={tpose:.1f}s 已叠={n_draw} 写出={n_write}",
                  end="", file=sys.stderr, flush=True)
    if writer:
        writer.release()
    cap.release()
    span = (t_last - t_first) if (t_first is not None and t_last is not None) else 0.0
    expect = n_write / out_fps if out_fps > 0 else 0.0
    print(f"\n[overlay] 写出 {args.out} 叠画={n_draw} 写出帧={n_write} @{out_fps:.1f}fps",
          file=sys.stderr)
    print(f"[overlay] 墙钟跨度≈{span:.2f}s  容器播放时长≈{expect:.2f}s "
          f"(应接近; 差很多说明帧率标错)", file=sys.stderr)


if __name__ == "__main__":
    main()
