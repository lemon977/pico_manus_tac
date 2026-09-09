#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""manus_ndjson_source.py — 无 ROS 的 MANUS 实时源, 把手套数据喂给 ControllerVisualizer。

不依赖 ROS: 直接启动 manus_ndjson_bridge 子进程
(SDK Integrated 直连 dongle), 解析其 stdout 的 NDJSON, 每帧调用
visualizer.set_manus(side, nodes(N,7), parents, count)。node0 为腕根。

用途: 在 PICO 接收端(与 MANUS dongle 同机)上, 把 MANUS 手实时叠加到 PICO 手柄
参考系, 肉眼验证 "MANUS 腕 ↔ PICO 腕" 是否贴合, 并配合 --calib 标定外参。

注意: dongle 同一时刻只能被一个进程占用。用本源做实时叠加验证时, 不要同时再跑
manus_collector.py(它也会抢桥)。需要落盘时单独用 manus_collector.py。
"""
from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np

HERE = Path(__file__).resolve().parent
DEFAULT_BRIDGE = (
    HERE / "manus_ndjson_bridge" /
    ("manus_ndjson_bridge.exe" if os.name == "nt" else "manus_ndjson_bridge.out")
)


def frame_to_nodes(obj: dict):
    """NDJSON manus_frame -> (nodes (N,7), parents (N,), count)。"""
    nodes_in = obj.get("nodes") or []
    n = len(nodes_in)
    nodes = np.zeros((n, 7), dtype=float)
    for i, row in enumerate(nodes_in):
        for j in range(min(7, len(row))):
            nodes[i, j] = float(row[j])
    parents_in = obj.get("parent_ids") or obj.get("parents") or []
    parents = np.full(n, -1, dtype=np.int32)
    for i in range(min(n, len(parents_in))):
        try:
            parents[i] = int(parents_in[i])
        except (TypeError, ValueError):
            parents[i] = -1
    count = int(obj.get("node_count", n) or n)
    return nodes, parents, min(count, n)


class ManusNdjsonSource:
    """启动 MANUS 桥并把每帧交给 visualizer.set_manus。

    用法:
        src = ManusNdjsonSource(viz, bridge="manus_ndjson_bridge/manus_ndjson_bridge.out")
        src.start()
        ...
        src.stop()
    """

    def __init__(self, visualizer, bridge: Optional[str] = None,
                 hand_motion: Optional[str] = None, on_event=None):
        self.viz = visualizer
        self.bridge = Path(bridge or DEFAULT_BRIDGE)
        self.hand_motion = hand_motion
        self._on_event = on_event or (lambda m: print(m, flush=True))
        self._proc: Optional[subprocess.Popen] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self.frames = {"left": 0, "right": 0, "unknown": 0}

    def start(self) -> None:
        bridge = self.bridge.resolve()
        if not bridge.is_file():
            setup = ("powershell -ExecutionPolicy Bypass -File "
                     "scripts/setup_manus_bridge.ps1"
                     if os.name == "nt"
                     else "bash scripts/setup_manus_bridge.sh")
            raise RuntimeError(
                f"找不到 MANUS 桥可执行文件: {bridge}; 先运行 {setup} 编译。")
        env = dict(os.environ)
        if os.name == "nt":
            dll_dirs = [
                bridge.parent,
                bridge.parent / "ManusSDK" / "bin",
                bridge.parent / "ManusSDK" / "lib",
            ]
            existing = [str(path) for path in dll_dirs if path.is_dir()]
            if existing:
                env["PATH"] = os.pathsep.join(existing + [env.get("PATH", "")])
        else:
            libdir = bridge.parent / "ManusSDK" / "lib"
            if libdir.is_dir():
                env["LD_LIBRARY_PATH"] = (
                    f"{libdir}{os.pathsep}{env.get('LD_LIBRARY_PATH', '')}"
                )
        if self.hand_motion:
            env["MANUS_HAND_MOTION"] = self.hand_motion
        self._on_event(f"[manus] 启动桥: {bridge}")
        self._proc = subprocess.Popen(
            [str(bridge)], stdout=subprocess.PIPE, stderr=None,
            bufsize=1, text=True, env=env)
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _reader(self) -> None:
        assert self._proc is not None and self._proc.stdout is not None
        for line in self._proc.stdout:
            if self._stop.is_set():
                break
            line = line.strip()
            if not line or not line.startswith("{"):
                continue
            try:
                obj = json.loads(line)
            except Exception:  # noqa: BLE001
                continue
            if obj.get("type") != "manus_frame":
                continue
            side = str(obj.get("side", "unknown")).lower()
            if side not in ("left", "right"):
                # side 未知时按左手兜底, 便于至少看到叠加
                side = "left"
            nodes, parents, count = frame_to_nodes(obj)
            if count > 0:
                try:
                    self.viz.set_manus(side, nodes, parents, count)
                except Exception as e:  # noqa: BLE001
                    self._on_event(f"[manus] set_manus 失败: {e!r}")
                self.frames[side] = self.frames.get(side, 0) + 1
        self._stop.set()

    def stop(self) -> None:
        self._stop.set()
        if self._proc is not None and self._proc.poll() is None:
            try:
                self._proc.send_signal(signal.SIGINT)
                self._proc.wait(timeout=3)
            except Exception:  # noqa: BLE001
                try:
                    self._proc.kill()
                except Exception:  # noqa: BLE001
                    pass
        if self._thread is not None:
            self._thread.join(timeout=1.0)


if __name__ == "__main__":
    # 自测: 不连硬件, 只验证 frame_to_nodes 解析。
    demo = {"type": "manus_frame", "side": "left", "node_count": 2,
            "nodes": [[0, 0, 0, 0, 0, 0, 1], [0.03, 0, 0, 0, 0, 0, 1]],
            "parent_ids": [-1, 0]}
    nd, pa, c = frame_to_nodes(demo)
    assert nd.shape == (2, 7) and c == 2 and pa[1] == 0
    print("[selftest] frame_to_nodes OK", file=sys.stderr)
