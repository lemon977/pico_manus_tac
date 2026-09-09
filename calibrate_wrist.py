#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""calibrate_wrist.py — 从一次「基准姿态」采集解出外参 T_calib(左右各一)。

术语: 手柄没有"手腕", 它只有自己的原点/跟踪点; "手腕"只属于 MANUS 手套(node0)。
T_calib 求的是 **手柄原点系 → 手套腕系** 的固定安装偏移, 配合 export_dataset.py
的 --calib 使用。ego 简约定下默认只补物理安装差(正戴时主要是平移 offset)。

标定动作(录一小段 3~5s 专门数据), 两件事要同时满足:
  1) 握法不变: 手柄握持/佩戴方式必须与真实采集时完全一致——
     T_calib 捕捉的就是这个固定偏移, 握法一变即作废;
  2) 标定窗静止且姿态已知: 那 2~3 秒保持一种已知世界朝向的姿势不动, 把
     MANUS 世界系与 PICO 世界系"对上表"。支持两种(--pose):
       forward(默认): 双手伸向正前(世界+X)、掌心朝下。此时解剖学腕系=世界系: 手指=+X, 手背=+Z。
       up:            双手竖起、手指朝上(+Z)、掌心朝脸(手背=+X)。竖起更好保持,
                      且手指方向靠重力, 不受"你以为的正前≠世界+X"的偏航误差影响。

原理(同时用两传感器; ego 简约定, 不做遥操 Q_CTRL_TO_WRIST):
  R_ctrl = 手柄原点世界朝向(LH→RH + Pico→Robot 轴)在静止窗口的平均
  R0     = 由 MANUS 手指几何(手指朝向 f, 手掌法向 n)映到世界(+X,+Z)的旋转
  T_calib = R_ctrl⁻¹ · R0
  使 wrist_pose = ctrl∘T_calib 在标定时=世界系, 之后随手柄走; 手指自洽。
  平移: 可视化 --show-origins 看偏差 cm, 手工改 calib 的 pos 即可。

用法:
  python3 calibrate_wrist.py calib_pico.jsonl calib_manus.jsonl -o config/calib_wrist.json
  # 可选: --win 1.5 静止窗口秒; --gate-ms 30; --dry-run 只打印不写文件
"""
from __future__ import annotations
import argparse, json, sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from align_pico_manus import load_manus, nearest            # noqa: E402
from export_dataset import (load_pico2, pico_to_world,       # noqa: E402
                            hand_local)
from pico_retarget import mat_to_quat, quat_conj, quat_mul, quat_normalize  # noqa: E402

# MANUS 25 关节里的关键索引(chain 顺序固定): 0=腕 1=Thumb_MCP 5=Index_MCP
# 10=Middle_MCP 15=Ring_MCP 20=Pinky_MCP
I_THUMB_MCP, I_INDEX_MCP, I_MIDDLE_MCP, I_PINKY_MCP = 1, 5, 10, 20
NS = 1_000_000


def unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v


def avg_quat(quats):
    """近似平均单位四元数(同一静止窗口): 统一半球后求均值。"""
    q0 = np.asarray(quats[0], float)
    acc = np.zeros(4)
    for q in quats:
        q = np.asarray(q, float)
        if np.dot(q, q0) < 0:
            q = -q
        acc += q
    return quat_normalize(list(acc / len(quats)))


def quat_ang_speed(qs):
    """相邻四元数夹角(rad), 用于找静止窗口。"""
    qs = [np.asarray(q, float) for q in qs]
    out = [0.0]
    for a, b in zip(qs[:-1], qs[1:]):
        d = abs(float(np.dot(unit(a), unit(b))))
        out.append(2 * np.arccos(np.clip(d, 0, 1)))
    return np.array(out)


def build_R0(joints, side, pose="forward"):
    """由手腕局部系的手指关节构造 R0: 局部->世界。

    pose=forward: 手指朝正前掌心朝下 (手指=+X, 手背=+Z, 掌心外法向=-Z);
    pose=up:      手指朝上掌心朝脸 (手指=+Z, 手背=+X, 掌心外法向=-X)。

    关键：旧版用「映射后拇指的世界 Y 符号」在 ±法向之间二选一，并假定
    forward/up 两种姿势下右手拇指都指 +Y。这对 up 不成立（掌心朝脸时右手
    拇指指 -Y），会把右手的 e3 选成掌心外法向而非手背 → 标定后表现为
    「真手心朝下、骨架手心朝上」（绕手指轴约 180°）。
    后改用 thumb×finger 消歧，但未考虑 MANUS 25 节点坐标系在左右手之间镜像，
    导致左手掌心/手背取反。现根据 side 选择 thumb×finger（右）或 finger×thumb（左），
    再取手背 = -掌心 作为 e3。
    """
    w = joints[0, :3]  # 腕根(局部≈0)
    f = unit(joints[I_MIDDLE_MCP] - w)                    # 手指朝向(掌骨方向)
    vi = joints[I_INDEX_MCP] - w
    vp = joints[I_PINKY_MCP] - w
    n = unit(np.cross(vi, vp))                            # 掌平面法向(符号待定)
    thumb = unit(joints[I_THUMB_MCP] - w)
    # 掌心外法向：MANUS 25 节点坐标系在左右手之间是镜像的。
    # 对右手，thumb × finger 指向掌心外；对左手则相反，须用 finger × thumb。
    # 据此与掌平面法向 n 对齐符号，得到稳定掌心法向，不再依赖世界 Y 符号。
    palm_dir = np.cross(thumb, f) if side == "right" else np.cross(f, thumb)
    palm = n if float(np.dot(n, palm_dir)) > 0 else -n
    back = -palm                                          # 文档约定 e3 = 手背

    if pose == "up":
        T1, T3 = np.array([0., 0., 1.]), np.array([1., 0., 0.])   # 手指→+Z, 手背→+X
    else:
        T1, T3 = np.array([1., 0., 0.]), np.array([0., 0., 1.])   # 手指→+X, 手背→+Z
    T2 = unit(np.cross(T3, T1))
    Tm = np.column_stack([T1, T2, T3])

    def make(nrm):
        e1 = unit(f)
        e3 = unit(nrm - np.dot(nrm, e1) * e1)            # 正交化到 f
        e2 = unit(np.cross(e3, e1))
        e3 = unit(np.cross(e1, e2))                       # 保证右手正交基
        # R0 = T·Eᵀ (E 列=e1,e2,e3), 使 R0@e1=T1, R0@e3=T3
        return Tm @ np.column_stack([e1, e2, e3]).T, e1, e3

    return make(back)


def collect(pico_path, manus_path, gate_ns):
    pico = load_pico2(Path(pico_path))
    manus = load_manus(Path(manus_path))
    keys = {s: [d["wall_ns"] for d in fr] for s, fr in manus.items()}
    rows = []
    for pf in pico:
        if pf["left_ctrl"] is None or pf["right_ctrl"] is None:
            continue
        lw = pico_to_world(pf["left_ctrl"]); rw = pico_to_world(pf["right_ctrl"])
        if np.dot(lw[0], lw[0]) < 1e-8 or np.dot(rw[0], rw[0]) < 1e-8:
            continue
        row = {"wall": pf["wall_ns"], "ts": pf["ts"],
               "lq": pico_to_world(pf["left_ctrl"])[1],
               "rq": pico_to_world(pf["right_ctrl"])[1],
               "lh": None, "rh": None}
        ok = True
        for side, tgt in (("left", "lh"), ("right", "rh")):
            fr = manus.get(side)
            if not fr:
                ok = False; break
            m = nearest(fr, keys[side], pf["wall_ns"])
            if m is None or abs(m["wall_ns"] - pf["wall_ns"]) > gate_ns or not m.get("nodes"):
                ok = False; break
            row[tgt] = hand_local(m["nodes"])
        if ok:
            rows.append(row)
    return rows


def find_still_window(rows, win_s):
    """找角速度最小、且真实时长≥win_s 的连续窗口(用干净的 ts 时钟)。"""
    ts = np.array([r["ts"] for r in rows], float)
    la = quat_ang_speed([r["lq"] for r in rows])
    ra = quat_ang_speed([r["rq"] for r in rows])
    act = la + ra
    win_ns = win_s * 1e9
    best = None
    for i in range(len(rows)):
        j = i
        while j < len(rows) and ts[j] - ts[i] < win_ns:
            j += 1
        if j >= len(rows) or j - i < 3:      # 未凑满 win_s 时长
            continue
        m = float(np.mean(act[i:j + 1]))
        if best is None or m < best[2]:
            best = (i, j + 1, m)
    if best is None:                          # 整段都比 win_s 短: 退回全段
        return 0, len(rows), float(np.mean(act))
    return best


def main():
    ap = argparse.ArgumentParser(description="解手腕外参 T_calib(基准姿态标定)")
    ap.add_argument("pico"); ap.add_argument("manus")
    ap.add_argument("-o", "--out", default="config/calib_wrist.json")
    ap.add_argument("--win", type=float, default=1.5, help="静止窗口秒")
    ap.add_argument("--pose", default="forward", choices=["forward", "up"],
                    help="标定姿态: forward=手指朝正前掌心朝下; up=手指朝上掌心朝脸")
    ap.add_argument("--gate-ms", type=float, default=30.0)
    ap.add_argument("--dry-run", action="store_true", help="只打印不写文件")
    args = ap.parse_args()

    rows = collect(args.pico, args.manus, int(args.gate_ms * NS))
    if len(rows) < 5:
        print(f"[calib] 有效帧太少({len(rows)}): 需头+双手柄+双手套同时有效。", file=sys.stderr)
        sys.exit(1)
    i, j, act = find_still_window(rows, args.win)
    dur = (rows[j - 1]["ts"] - rows[i]["ts"]) / 1e9
    print(f"[calib] 静止窗口: 帧[{i}:{j}] {j-i}帧 ≈{dur:.2f}s 平均角速度={act:.4f}rad/帧")
    if act > 0.02:
        print("[calib] ⚠ 该窗口仍较晃动, 结果可能不准; 请录一段更静止的标定动作。", file=sys.stderr)

    win = rows[i:j]
    pose_desc = "手指朝上掌心朝脸(手指=+Z,手背=+X)" if args.pose == "up" \
        else "掌心朝下手指朝前(手指=+X,手背=+Z)"
    tgt1, tgt3 = ([0, 0, 1], [1, 0, 0]) if args.pose == "up" else ([1, 0, 0], [0, 0, 1])
    out = {"_meta": {
        "note": f"T_calib: wrist_pose = ctrl_world∘T_calib (ego简约定,无Q180); 标定姿态={pose_desc}; pos=手柄局部系前/左/上(米)",
        "pose": args.pose,
        "source_pico": Path(args.pico).name, "source_manus": Path(args.manus).name,
        "window_frames": j - i, "window_s": round(dur, 3),
        "mean_ang_speed_rad_per_frame": round(act, 5)}}
    for side, qk, hk in (("left", "lq", "lh"), ("right", "rq", "rh")):
        q_ctrlQ = avg_quat([r[qk] for r in win])
        joints = np.mean([r[hk] for r in win], axis=0)
        R0, e1, e3 = build_R0(joints, side, pose=args.pose)
        q_R0 = mat_to_quat([list(R0[k]) for k in range(3)])
        q_calib = quat_normalize(quat_mul(quat_conj(q_ctrlQ), q_R0))
        out[side] = {"pos": [0.0, 0.0, 0.0], "quat": [round(x, 6) for x in q_calib]}
        # 自检: 修正后手腕系(=q_ctrlQ∘q_calib=R0) 把测得的手指朝向/掌法向映到目标轴
        from pico_retarget import quat_rotate
        chk = quat_normalize(quat_mul(q_ctrlQ, q_calib))
        fwd = quat_rotate(chk, list(e1))   # 应≈ tgt1
        up = quat_rotate(chk, list(e3))    # 应≈ tgt3
        print(f"[calib] {side}: T_calib quat={out[side]['quat']}")
        print(f"        自检 手指朝向→{[round(v,2) for v in fwd]}(应≈{tgt1}) "
              f"手背朝向→{[round(v,2) for v in up]}(应≈{tgt3})")

    if args.dry_run:
        print("[calib] --dry-run: 不写文件。JSON 预览:")
        print(json.dumps(out, ensure_ascii=False, indent=2))
        return
    op = Path(args.out); op.parent.mkdir(parents=True, exist_ok=True)
    op.write_text(json.dumps(out, ensure_ascii=False, indent=2))
    print(f"[calib] 写出 {op}")
    print(f"[calib] 用法: python3 export_dataset.py <pico> <manus> -o d.hdf5 --calib {op}")


if __name__ == "__main__":
    main()
