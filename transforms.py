#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""transforms.py — 统一坐标转换层。

管理 pico_controller 里所有常用坐标系之间的转换，避免训练/部署/可视化
脚本里各自重复写矩阵公式。

支持的坐标系：
- pico_world: export_dataset 导出的世界系，RH X前Y左Z上，单位米
- head (neck_yaw_link): 头-relative，原点在头，X 前为头朝向
- robot_base: 机器人基座系（需外部标定 T_pico_world_to_base）
- wrist_local: MANUS 手关节的局部系（相对腕根）

用法示例：
    from transforms import world_to_head, head_to_world
    import h5py, json

    with h5py.File("data/export/vc1.hdf5") as f:
        head = f["head_pose"][:]
        lw   = f["left_wrist_pose"][:]
        lw_rel = world_to_head(lw, head)      # (T, 7)
        lw_back = head_to_world(lw_rel, head)   # 应 ≈ lw
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from pico_retarget import (
    compose_pose,
    convert_lh_to_rh,
    apply_pico_to_robot_axes,
    relative_pose,
    quat_to_mat,
    mat_to_quat,
)


# ------------------------------------------------------------------ pose 工具

def split_pose(pose: np.ndarray):
    """把 (..., 7) 拆成 pos (..., 3) + quat (..., 4)。"""
    pose = np.asarray(pose, float)
    return pose[..., :3], pose[..., 3:]


def merge_pose(pos: np.ndarray, quat: np.ndarray) -> np.ndarray:
    """把 pos (..., 3) + quat (..., 4) 拼回 (..., 7)。"""
    return np.concatenate([np.asarray(pos, float), np.asarray(quat, float)], axis=-1)


def pose_to_tuple(pose: np.ndarray):
    """把一帧 (7,) 拆成 ([x,y,z], [qx,qy,qz,qw])，用于复用 pico_retarget 函数。"""
    p = np.asarray(pose, float)
    return list(p[:3]), list(p[3:])


def tuple_to_pose(t) -> np.ndarray:
    """([x,y,z], [qx,qy,qz,qw]) -> np.ndarray(7,)"""
    return np.array(list(t[0]) + list(t[1]), float)


# ------------------------------------------------------------------ 批量转换

def _batch_transform(poses: np.ndarray, ref: np.ndarray | None, forward: bool) -> np.ndarray:
    """forward=True:  world -> ref^-1 * pose  （relative_pose）
       forward=False: ref -> ref * pose        （compose_pose）
    """
    poses = np.asarray(poses, float)
    single = poses.ndim == 1
    if single:
        poses = poses.reshape(1, -1)

    if ref is None:
        out = poses.copy()
    else:
        ref = np.asarray(ref, float)
        if ref.ndim == 1:
            ref = np.tile(ref, (len(poses), 1))
        out = np.zeros_like(poses)
        for i in range(len(poses)):
            if forward:
                t = relative_pose(pose_to_tuple(ref[i]), pose_to_tuple(poses[i]))
            else:
                t = compose_pose(pose_to_tuple(ref[i]), pose_to_tuple(poses[i]))
            out[i] = tuple_to_pose(t)

    return out[0] if single else out


def world_to_head(pose_world: np.ndarray, head_world: np.ndarray) -> np.ndarray:
    """pico_world -> neck_yaw_link（头-relative）。

    pose_world, head_world: (..., 7) 或 (7,)，后 4 维为 quat(x,y,z,w)。
    返回同 shape 的 relative pose。
    """
    return _batch_transform(pose_world, head_world, forward=True)


def head_to_world(pose_head: np.ndarray, head_world: np.ndarray) -> np.ndarray:
    """neck_yaw_link -> pico_world。"""
    return _batch_transform(pose_head, head_world, forward=False)


# ------------------------------------------------------------------ pico 原始 -> world

def pico_raw_to_world(raw_pose: np.ndarray) -> np.ndarray:
    """PICO 原始 (LH) -> pico_world (RH)。

    raw_pose: (..., 7) 或 (7,)
    """
    single = raw_pose.ndim == 1
    raw_pose = np.asarray(raw_pose, float)
    if single:
        raw_pose = raw_pose.reshape(1, -1)
    out = np.zeros_like(raw_pose)
    for i in range(len(raw_pose)):
        out[i] = tuple_to_pose(apply_pico_to_robot_axes(convert_lh_to_rh(pose_to_tuple(raw_pose[i]))))
    return out[0] if single else out


# ------------------------------------------------------------------ robot_base

def load_T_pico_world_to_base(path: str | Path) -> tuple[np.ndarray, np.ndarray] | None:
    """读取 {pos:[3], quat:[4]} 形式的 T_pico_world_to_base。

    返回 (pos, quat)，用于 world_to_robot_base / robot_base_to_world。
    """
    p = Path(path)
    if not p.is_file():
        return None
    obj = json.loads(p.read_text(encoding="utf-8"))
    pos = np.array(obj["pos"], float)
    quat = np.array(obj["quat"], float)
    return pos, quat


def world_to_robot_base(pose_world: np.ndarray,
                        T_pico_world_to_base: tuple[np.ndarray, np.ndarray]) -> np.ndarray:
    """pico_world -> robot_base。

    T_pico_world_to_base: (pos, quat)，表示 base 原点在 pico_world 里的位姿。
    等价于 relative_pose(T_pico_world_to_base, pose_world)。
    """
    base_pose = tuple_to_pose(T_pico_world_to_base)
    return _batch_transform(pose_world, base_pose, forward=True)


def robot_base_to_world(pose_base: np.ndarray,
                        T_pico_world_to_base: tuple[np.ndarray, np.ndarray]) -> np.ndarray:
    """robot_base -> pico_world。"""
    base_pose = tuple_to_pose(T_pico_world_to_base)
    return _batch_transform(pose_base, base_pose, forward=False)


# ------------------------------------------------------------------ MANUS 手局部

def manus_to_wrist_local(nodes: np.ndarray) -> np.ndarray:
    """MANUS 25 节点 (N,25,7) 或 (25,7) -> 腕局部系 (N,25,3) 或 (25,3)。

    只取位置，去掉腕根世界朝向。等价于 export_dataset.hand_local() 的批量版。
    """
    a = np.asarray(nodes, float)
    single = a.ndim == 2
    if single:
        a = a.reshape(1, *a.shape)
    wp = a[:, 0, :3]                       # (N, 3)
    rel = a[:, :, :3] - wp[:, None, :]     # (N, 25, 3)
    return rel[0] if single else rel


# ------------------------------------------------------------------ 标定辅助

def estimate_T_pico_world_to_base(head_world_poses: np.ndarray,
                                  base_marker_in_pico: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """极简标定：已知机器人 base 上某标记点在 pico_world 下的多次观测，求 base 原点。

    这里假设 base 的朝向与 pico_world 的 X 前 Y 左 Z 上近似对齐（例如机器人正对操作者）。
    若需要精确旋转，请用 AprilTag PnP。

    head_world_poses: (N,7)，采集者位于 base 正前方时的头部位姿
    base_marker_in_pico: (N,3)，同帧看到的 base 标记点（如胸前位置）
    返回 (pos, quat=identity)
    """
    marker = np.asarray(base_marker_in_pico, float)
    # 简单取中位数位置，朝向用identity
    pos = np.median(marker, axis=0)
    return pos, np.array([0.0, 0.0, 0.0, 1.0], float)
