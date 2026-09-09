#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""video_offset.py — 自动估计「VST 视频」相对「姿态数据」的时间偏移。

原理：分别算两条「运动能量」时间序列——
  姿态: 每帧 头+双手柄 世界速度之和；
  视频: 相邻帧灰度差的均值(整体运动量)。
把两条归一化后做互相关，取相关最高的偏移即最佳 offset(秒)，可直接喂给
make_review_video.py --offset。

用法: python3 video_offset.py aligned_*.jsonl vst_*.mp4
"""
import json, sys
import numpy as np
import cv2

aligned_path, video_path = sys.argv[1], sys.argv[2]
GRID_HZ = 20.0
SEARCH = 8.0   # 搜索 ±8s

# ---------------- 姿态运动能量 ----------------
# ego 画面运动来自 头旋转 + 近景手运动。分别构造几种信号, 自动挑相关最强的。
def load_pose_signals(path):
    walls=[]; hpos=[]; hq=[]; lpos=[]; rpos=[]
    def gp(v): return np.array(v[0]) if v else np.array([np.nan]*3)
    with open(path) as f:
        for line in f:
            line=line.strip()
            if not line: continue
            try: r=json.loads(line)
            except: continue
            if "pico_wall_ns" not in r or not r.get("head"): continue
            walls.append(r["pico_wall_ns"]/1e9)
            hpos.append(r["head"][0]); hq.append(r["head"][1])
            lpos.append(gp(r.get("left_ctrl"))); rpos.append(gp(r.get("right_ctrl")))
    walls=np.array(walls); hpos=np.array(hpos); hq=np.array(hq)
    lpos=np.array(lpos); rpos=np.array(rpos)
    t=walls-walls[0]; dt=np.clip(np.diff(t),1e-3,None)
    hq=hq/np.clip(np.linalg.norm(hq,axis=1,keepdims=True),1e-9,None)
    dots=np.abs(np.sum(hq[1:]*hq[:-1],axis=1)).clip(0,1)
    ang=np.zeros(len(t)); ang[1:]=2*np.arccos(dots)/dt
    def spd(p):
        s=np.zeros(len(t)); d=np.linalg.norm(np.diff(p,axis=0),axis=1)/dt
        s[1:]=np.nan_to_num(d); return s
    hlin=spd(hpos); hands=np.fmax(spd(lpos),spd(rpos))
    return t, {"head_rot": ang, "head_rot+lin": ang+3*hlin, "hands": hands,
               "head+hands": ang+2*hands}

# ---------------- 视频运动能量 ----------------
def load_video_energy(path, step_hz=GRID_HZ):
    cap=cv2.VideoCapture(path)
    fps=cap.get(cv2.CAP_PROP_FPS) or 60.0
    stride=max(1,int(round(fps/step_hz)))
    t=[]; e=[]; prev=None; idx=0
    while True:
        ok=cap.grab()
        if not ok: break
        if idx%stride==0:
            ok,fr=cap.retrieve()
            if not ok: break
            g=cv2.cvtColor(cv2.resize(fr,(160,120)),cv2.COLOR_BGR2GRAY).astype(np.float32)
            if prev is not None:
                e.append(float(np.mean(np.abs(g-prev))))
                t.append(idx/fps)
            prev=g
        idx+=1
    cap.release()
    return np.array(t), np.array(e)

def resample(t,v,grid):
    return np.interp(grid, t, v, left=0, right=0)

def norm(x):
    x=x-x.mean()
    s=x.std()
    return x/s if s>1e-9 else x

print("[offset] 读取姿态运动能量(多信号)…")
tp,sigs=load_pose_signals(aligned_path)
print(f"  姿态时长≈{tp[-1]:.1f}s, 帧={len(tp)}")
print("[offset] 读取视频运动能量(抽帧)…")
tv,vv=load_video_energy(video_path)
print(f"  视频时长≈{tv[-1]:.1f}s, 采样点={len(tv)}")

dur=min(tp[-1], tv[-1])
grid=np.arange(0, dur, 1.0/GRID_HZ)
offs=np.arange(-SEARCH, SEARCH+1e-6, 0.05)
gv_cache=[norm(resample(tv+off, vv, grid)) for off in offs]

results={}
for name,sig in sigs.items():
    gp=norm(resample(tp,sig,grid))
    corrs=np.array([float(np.mean(gp*gv)) for gv in gv_cache])
    i=int(np.argmax(corrs))
    results[name]=(offs[i], corrs[i], corrs)

# 挑相关最强的信号
best_name=max(results, key=lambda k: results[k][1])
best_off, best_c, best_corrs = results[best_name]

print("\n===== 各信号估计 =====")
for name,(o,c,_) in sorted(results.items(), key=lambda x:-x[1][1]):
    print(f"  {name:14s} offset={o:+.2f}s  corr={c:.3f}")

print("\n===== 结论 =====")
print(f"采用信号「{best_name}」: 最佳 offset = {best_off:+.2f} s (corr={best_c:.3f})")
# 一致性: 各信号 offset 是否聚拢
all_offs=np.array([v[0] for v in results.values()])
spread=all_offs.max()-all_offs.min()
print(f"各信号 offset 一致性: 范围 {all_offs.min():+.2f}~{all_offs.max():+.2f}s (跨度{spread:.2f}s)")
if best_c<0.2 and spread>1.0:
    print("⚠ 相关弱且不一致: offset 不可靠, 建议目视微调, 或录制开头加个明显拍手动作再估。")
elif best_c<0.2:
    print("△ 相关弱但各信号一致, offset 大致可信, 建议 make_review_video 目视微调 ±0.3s。")
else:
    print("✓ offset 可信。")
print(f"\n用法: python3 make_review_video.py {aligned_path} {video_path} \\")
print(f"        -o review.mp4 --offset {best_off:.2f}")
