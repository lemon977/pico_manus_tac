#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""manus_hdf5.py — 把 manus NDJSON 帧写成精简 HDF5（无 ROS/无第三方消息依赖）。

字段与本仓库 manus_jq_collection 的采集约定保持一致：
  manus/<side>/
    recv_wall_ns, recv_mono_ns, publish_time, glove_id, node_count
    poses            (N, MAX_NODES, 7)   [x,y,z,qx,qy,qz,qw]
    node_ids         (N, MAX_NODES)
    parent_ids       (N, MAX_NODES)
    joint_types_json (N,)  逐帧 JSON
    chain_types_json (N,)
    fingertip_xyz    (N, 5, 3)           thumb..pinky 的 Tip 节点
attrs: manus_coordinate=wrist_rooted_world_space_HandMotion_None, manus_units=meters
"""
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Dict, List

import h5py
import numpy as np

MAX_NODES = 32
POSE_DIM = 7
MAX_SENSORS = 5


class ManusJsonlToHdf5:
    def __init__(self, path, *, task_name: str = "manus_session") -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._f = h5py.File(self.path, "w")
        self._f.attrs["format_version"] = "1.0"
        self._f.attrs["task_name"] = task_name
        self._f.attrs["created_wall_ns"] = time.time_ns()
        self._f.attrs["manus_coordinate"] = "wrist_rooted_world_space_HandMotion_None"
        self._f.attrs["manus_units"] = "meters"
        self._f.attrs["source"] = "manus_ndjson_bridge"
        self._g = self._f.create_group("manus")
        self._sides: Dict[str, h5py.Group] = {}
        self._counts: Dict[str, int] = {}

    def _side_group(self, side: str) -> h5py.Group:
        if side in self._sides:
            return self._sides[side]
        g = self._g.create_group(side)
        dt = h5py.string_dtype(encoding="utf-8")
        g.create_dataset("recv_wall_ns", (0,), maxshape=(None,), dtype=np.int64, chunks=True)
        g.create_dataset("recv_mono_ns", (0,), maxshape=(None,), dtype=np.int64, chunks=True)
        g.create_dataset("recv_qpc_ns", (0,), maxshape=(None,), dtype=np.int64, chunks=True)
        g.create_dataset("publish_time", (0,), maxshape=(None,), dtype=np.uint64, chunks=True)
        g.create_dataset("glove_id", (0,), maxshape=(None,), dtype=np.int32, chunks=True)
        g.create_dataset("node_count", (0,), maxshape=(None,), dtype=np.int32, chunks=True)
        g.create_dataset("poses", (0, MAX_NODES, POSE_DIM),
                         maxshape=(None, MAX_NODES, POSE_DIM), dtype=np.float64,
                         chunks=(1, MAX_NODES, POSE_DIM))
        g.create_dataset("node_ids", (0, MAX_NODES), maxshape=(None, MAX_NODES),
                         dtype=np.int32, chunks=(1, MAX_NODES))
        g.create_dataset("parent_ids", (0, MAX_NODES), maxshape=(None, MAX_NODES),
                         dtype=np.int32, chunks=(1, MAX_NODES))
        g.create_dataset("joint_types_json", (0,), maxshape=(None,), dtype=dt, chunks=True)
        g.create_dataset("chain_types_json", (0,), maxshape=(None,), dtype=dt, chunks=True)
        g.create_dataset("fingertip_xyz", (0, MAX_SENSORS, 3),
                         maxshape=(None, MAX_SENSORS, 3), dtype=np.float64,
                         chunks=(1, MAX_SENSORS, 3))
        self._sides[side] = g
        self._counts[side] = 0
        return g

    @staticmethod
    def _append(ds: h5py.Dataset, row) -> None:
        n = ds.shape[0]
        ds.resize(n + 1, axis=0)
        ds[n] = row

    def append(self, obj: dict, fingertips: List[List[float]]) -> None:
        side = obj.get("side", "unknown")
        g = self._side_group(side)

        nodes = obj.get("nodes", []) or []
        nc = min(int(obj.get("node_count", len(nodes))), MAX_NODES)

        poses = np.zeros((MAX_NODES, POSE_DIM), dtype=np.float64)
        node_ids = np.full(MAX_NODES, -1, dtype=np.int32)
        parent_ids = np.full(MAX_NODES, -1, dtype=np.int32)
        raw_nids = obj.get("node_ids", []) or []
        raw_pids = obj.get("parent_ids", []) or []
        for i in range(min(nc, len(nodes))):
            p = nodes[i]
            poses[i, : len(p)] = p[:POSE_DIM]
        for i in range(min(nc, len(raw_nids))):
            node_ids[i] = int(raw_nids[i])
        for i in range(min(nc, len(raw_pids))):
            parent_ids[i] = int(raw_pids[i])

        ft = np.zeros((MAX_SENSORS, 3), dtype=np.float64)
        for i in range(min(MAX_SENSORS, len(fingertips))):
            ft[i] = fingertips[i][:3]

        self._append(g["recv_wall_ns"], np.int64(obj.get("recv_wall_ns", 0)))
        self._append(g["recv_mono_ns"], np.int64(obj.get("recv_mono_ns", 0)))
        self._append(g["recv_qpc_ns"], np.int64(obj.get("recv_qpc_ns", 0)))
        self._append(g["publish_time"], np.uint64(obj.get("publish_time", 0)))
        self._append(g["glove_id"], np.int32(obj.get("glove_id", 0)))
        self._append(g["node_count"], np.int32(nc))
        self._append(g["poses"], poses)
        self._append(g["node_ids"], node_ids)
        self._append(g["parent_ids"], parent_ids)
        self._append(g["joint_types_json"],
                     json.dumps(obj.get("joint_types", [])[:nc], ensure_ascii=False))
        self._append(g["chain_types_json"],
                     json.dumps(obj.get("chain_types", [])[:nc], ensure_ascii=False))
        self._append(g["fingertip_xyz"], ft)
        self._counts[side] = self._counts.get(side, 0) + 1

    def close(self) -> None:
        if self._f is None:
            return
        self._f.attrs["closed_wall_ns"] = time.time_ns()
        self._f.attrs["counts_json"] = json.dumps(self._counts)
        self._f.flush()
        self._f.close()
        self._f = None
