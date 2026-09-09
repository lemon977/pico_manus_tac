#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""analyze_quality.py — PICO+MANUS 采集数据体检（离线，只读现有 jsonl）。

结论导向：帧率/空帧/零位/坐标系正确性(Z上、头在手上方、右手系)/MANUS 手指活动度。
用法: python3 analyze_quality.py pico_*.jsonl manus_*.jsonl
"""
import json, sys
import numpy as np

sys.path.insert(0, ".")
from pico_retarget import convert_lh_to_rh, apply_pico_to_robot_axes, R_PICO_TO_ROBOT

def world_pos(node):
    """PICO 原始 pos/quat -> 世界系(机器人 X前Y左Z上) 位置。"""
    if not isinstance(node, dict):
        return None
    p = node.get("pos"); q = node.get("quat")
    if not (p and q and len(p) >= 3 and len(q) >= 4):
        return None
    (pp, _) = apply_pico_to_robot_axes(convert_lh_to_rh((p[:3], q[:4])))
    return np.array(pp)

def raw_pos(node):
    if not isinstance(node, dict):
        return None
    p = node.get("pos")
    return np.array(p[:3]) if p and len(p) >= 3 else None

def sec(ns): return ns/1e9

pico_path, manus_path = sys.argv[1], sys.argv[2]

print("#"*64)
print("# PICO 部分")
print("#"*64)

H=[]; Lw=[]; Rw=[]; ts=[]; wall=[]
head_null=lz=rz=0; n=0
Lheld=[]; Rheld=[]
with open(pico_path) as f:
    for line in f:
        try: r=json.loads(line)
        except: continue
        if r.get("functionName")!="Tracking": continue
        d=r.get("data") or {}
        n+=1
        if "timeStampNs" in d: ts.append(d["timeStampNs"])
        if r.get("recv_wall_ns"): wall.append(r["recv_wall_ns"])
        h=world_pos(d.get("Head"))
        if h is None: head_null+=1
        else: H.append(h)
        c=d.get("Controller") or {}
        lr=raw_pos(c.get("left")); rr=raw_pos(c.get("right"))
        # 零位=未握持
        if lr is None or np.dot(lr,lr)<1e-8: lz+=1
        else:
            wl=world_pos(c.get("left")); Lw.append(wl); Lheld.append(1)
        if rr is None or np.dot(rr,rr)<1e-8: rz+=1
        else:
            wr=world_pos(c.get("right")); Rw.append(wr); Rheld.append(1)

H=np.array(H); Lw=np.array(Lw); Rw=np.array(Rw)
print(f"跟踪帧={n}")
print(f"Head 空帧={head_null} ({100*head_null/n:.1f}%)")
print(f"左手柄 零位/未握持={lz} ({100*lz/n:.1f}%)  右手柄 零位={rz} ({100*rz/n:.1f}%)")

if len(ts)>1:
    ts=np.array(sorted(ts),float); dts=np.diff(ts)/1e6
    gaps=(dts>100).sum()
    print(f"\n[采样时钟 timeStampNs] 帧间隔ms 中位={np.median(dts):.2f} p95={np.percentile(dts,95):.2f} "
          f"等效≈{1000/np.median(dts):.0f}Hz  >100ms 的大间隙={gaps} 处")
    # 连续片段
    segs=1+ (dts>100).sum()
    print(f"  连续片段数(>100ms 处切段)={segs}")
if len(wall)>1:
    w=np.array(sorted(wall),float); dw=np.diff(w)/1e6
    print(f"[接收时钟 recv_wall]   帧间隔ms 中位={np.median(dw):.2f} p95={np.percentile(dw,95):.2f} (成簇属正常)")

print("\n[坐标系正确性] (世界系: X前 Y左 Z上)")
# 完整位置线性映射 = R_PICO_TO_ROBOT · diag(1,1,-1)   (后者=convert_lh_to_rh 的 z 取反)
M=np.array(R_PICO_TO_ROBOT) @ np.diag([1.0,1.0,-1.0])
det=np.linalg.det(M)
print(f"  完整轴变换(含LH→RH)行列式={det:+.1f} (应=+1 → 右手系正规旋转) {'OK✓' if abs(det-1)<1e-6 else '✗错误'}")
if len(H):
    print(f"  Head 世界位置范围: X[{H[:,0].min():+.2f},{H[:,0].max():+.2f}] "
          f"Y[{H[:,1].min():+.2f},{H[:,1].max():+.2f}] Z[{H[:,2].min():+.2f},{H[:,2].max():+.2f}]")
    head_span = np.ptp(H, axis=0)
    print(f"  Head 运动幅度 X={head_span[0]:.2f} Y={head_span[1]:.2f} Z={head_span[2]:.2f} m "
          f"({'头在动' if head_span.max()>0.05 else '⚠头几乎不动'})")
# 头在手上方？(Z_head > Z_ctrl)
if len(H) and len(Lw) and len(Rw):
    m=min(len(H),len(Lw),len(Rw))
    zc=np.vstack([Lw[:m,2],Rw[:m,2]]).mean(0)
    above=100*np.mean(H[:m,2]>zc)
    dzc=(H[:m,2]-zc)
    print(f"  头 Z 高于双手均值 的帧占比={above:.0f}%  (头-手 高度差 中位={np.median(dzc)*100:.0f}cm) "
          f"{'✓符合站姿' if above>70 else '⚠偏低,检查Z是否为上'}")
    # 手大多在身前 X>头 X ?
    xc=np.vstack([Lw[:m,0],Rw[:m,0]]).mean(0)
    front=100*np.mean(xc>H[:m,0])
    print(f"  手在头前方(X更大) 的帧占比={front:.0f}%")

print("\n"+"#"*64)
print("# MANUS 部分")
print("#"*64)
# 逐帧收集每只手 指尖到腕距离(检查手指是否活动)
FINGERS=("Thumb","Index","Middle","Ring","Pinky")
data={"left":{f:[] for f in FINGERS},"right":{f:[] for f in FINGERS}}
gid={"left":set(),"right":set()}
counts={"left":0,"right":0}
mwall={"left":[],"right":[]}
with open(manus_path) as f:
    for line in f:
        if not line.startswith("{"): continue
        try: o=json.loads(line)
        except: continue
        if o.get("type")!="manus_frame": continue
        side=o.get("side"); 
        if side not in data: continue
        counts[side]+=1
        gid[side].add(o.get("glove_id"))
        if o.get("recv_wall_ns"): mwall[side].append(o["recv_wall_ns"])
        nodes=np.array(o["nodes"],float)[:,:3]
        ch=o["chain_types"]; jt=o["joint_types"]
        wrist=nodes[0]
        for i,(c,j) in enumerate(zip(ch,jt)):
            if j=="TIP" and c in data[side]:
                data[side][c].append(np.linalg.norm(nodes[i]-wrist))
for side in ("left","right"):
    print(f"\n[{side}] 帧={counts[side]}  glove_id={gid[side]}")
    if len(mwall[side])>1:
        dw=np.diff(np.array(sorted(mwall[side]),float))/1e6
        print(f"  帧间隔ms 中位={np.median(dw):.2f} 等效≈{1000/np.median(dw):.0f}Hz")
    for fgr in FINGERS:
        v=np.array(data[side][fgr])
        if len(v)==0:
            print(f"  {fgr:7s} 无数据"); continue
        span=(v.max()-v.min())*100
        flag = "✓活动" if span>2.0 else "⚠几乎不动(可能没校准/传感器问题)"
        print(f"  {fgr:7s} 指尖-腕距离 {v.min()*100:5.1f}~{v.max()*100:5.1f}cm 变化幅度={span:4.1f}cm {flag}")
