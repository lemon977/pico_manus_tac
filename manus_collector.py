#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""manus_collector.py — 无 ROS 的 MANUS 手套采集器。

启动 manus_ndjson_bridge 子进程（SDK Integrated 直连 dongle），读取其 stdout 的
NDJSON，逐帧落盘 logs/manus_YYYYMMDD_HHMMSS.jsonl。每帧带 recv_qpc_ns /
recv_wall_ns；新会话用同机高精度单调钟对齐，旧会话统一回退墙钟。

用法：
  python3 manus_collector.py                 # 启动, 按 Enter 开始, Ctrl+C 停止保存
  python3 manus_collector.py --no-wait        # 立即开始录制
  python3 manus_collector.py --duration 60    # 录 60s 自动停止
  python3 manus_collector.py --hdf5           # 额外导出 HDF5（需要 h5py）

校准/设置经环境变量透传给桥：
  MANUS_CALIB_LEFT=config/xxxLeft.mcal MANUS_CALIB_RIGHT=... \
  MANUS_SETTINGS_DIR=... python3 manus_collector.py
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import queue
import signal
import subprocess
import sys
import threading
import time
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

from record_control import (
    ControlServer, MANUS_CONTROL_PORT,
    parse_hands, hands_label, parse_start_arg, HANDS_BOTH, HANDS_CHOICES,
    wait_queue_drained,
)
from session_layout import session_raw_dir

HERE = Path(__file__).resolve().parent
DEFAULT_BRIDGE = (
    HERE / "manus_ndjson_bridge" /
    ("manus_ndjson_bridge.exe" if os.name == "nt" else "manus_ndjson_bridge.out")
)


def _calibration_file_info(side: str) -> dict:
    """Capture immutable evidence for the calibration file given to the SDK."""
    env_key = f"MANUS_CALIB_{side.upper()}"
    raw_path = os.environ.get(env_key, "")
    path = Path(raw_path).resolve() if raw_path else None
    if path is None or not path.is_file():
        return {"side": side, "env": env_key, "path": raw_path, "exists": False}
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    stat = path.stat()
    return {
        "side": side,
        "env": env_key,
        "path": str(path),
        "file_name": path.name,
        "exists": True,
        "bytes": stat.st_size,
        "mtime_ns": stat.st_mtime_ns,
        "sha256": digest.hexdigest(),
    }

# 指尖 Tip 节点提取（thumb..pinky），与本仓库 manus_normalize 约定一致。
_JOINT_TIP = {"tip", "5"}
_CHAIN_FINGER_ORDER = (
    {"thumb", "fingerthumb", "5"},
    {"index", "fingerindex", "6"},
    {"middle", "fingermiddle", "7"},
    {"ring", "fingerring", "8"},
    {"pinky", "fingerpinky", "9"},
)


def fingertip_xyz(nodes: List[List[float]], joint_types: List[str], chain_types: List[str]):
    out = [[0.0, 0.0, 0.0] for _ in range(5)]
    n = min(len(nodes), len(joint_types), len(chain_types))
    for i in range(n):
        if str(joint_types[i] or "").strip().lower() not in _JOINT_TIP:
            continue
        chain = str(chain_types[i] or "").strip().lower()
        for fi, aliases in enumerate(_CHAIN_FINGER_ORDER):
            if chain in aliases:
                out[fi] = list(nodes[i][:3])
                break
    return out


class Collector:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.log_dir = Path(args.log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        # 常驻服务启动只负责连接/预热。目录必须等收到正式 START 后再创建，
        # 否则每次重启服务都会留下 data/sessions/<启动时间>/raw 空目录。
        if args.service:
            self.session = None
            self.jsonl_path = None
            self.manus_meta_path = None
            self.hdf5_path = None
        else:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.session = stamp
            sess_dir = session_raw_dir(self.log_dir, stamp)
            sess_dir.mkdir(parents=True, exist_ok=True)
            self.jsonl_path = sess_dir / "manus.jsonl"
            self.manus_meta_path = sess_dir / "manus.meta.json"
            self.hdf5_path = (sess_dir / "manus.hdf5") if args.hdf5 else None

        self._recording = threading.Event()
        self._stopping = False
        if not args.service and (args.no_wait or args.duration > 0):
            self._recording.set()
        self._stop = threading.Event()
        self._fp = None
        self._lock = threading.Lock()

        self._counts: Dict[str, int] = defaultdict(int)
        self._latest: Dict[str, dict] = {}
        self._latest_rx: Dict[str, float] = {}  # side -> wall sec
        self._calibration_files = {
            side: _calibration_file_info(side) for side in ("left", "right")
        }
        self._last_status = 0.0
        self._h5 = None
        self.writer_errors = 0
        self.max_write_backlog = 0
        self._write_queue = queue.Queue(maxsize=32768)
        self._writer_thread = threading.Thread(
            target=self._writer_loop, name="manus-jsonl-writer", daemon=True
        )
        self._writer_thread.start()
        self.default_hands = parse_hands(getattr(args, "hands", "both"))
        self.hands = self.default_hands

        self._proc: Optional[subprocess.Popen] = None

    def _writer_loop(self) -> None:
        """异步写 JSONL/HDF5，避免磁盘 I/O 反压 SDK stdout 读取。"""
        last_flush = time.monotonic()
        while True:
            fp, session, obj = self._write_queue.get()
            try:
                fp.write(json.dumps(obj, ensure_ascii=False) + "\n")
                h5 = self._ensure_h5()
                if h5 is not None:
                    h5.append(obj, fingertip_xyz(
                        obj.get("nodes", []),
                        obj.get("joint_types", []),
                        obj.get("chain_types", []),
                    ))
                side = obj.get("side", "unknown")
                with self._lock:
                    if session == self.session:
                        self._counts[side] += 1
                now = time.monotonic()
                if now - last_flush >= 0.5:
                    fp.flush()
                    last_flush = now
            except Exception as exc:  # noqa: BLE001
                with self._lock:
                    self.writer_errors += 1
                print(f"[collector] MANUS 异步写盘失败: {exc}", file=sys.stderr,
                      flush=True)
            finally:
                self._write_queue.task_done()

    # ------------------------------------------------------------------ bridge
    def _spawn_bridge(self) -> None:
        bridge = Path(self.args.bridge).resolve()
        if not bridge.is_file():
            setup = ("powershell -ExecutionPolicy Bypass -File "
                     "scripts/setup_manus_bridge.ps1"
                     if os.name == "nt"
                     else "bash scripts/setup_manus_bridge.sh")
            print(f"[collector] 找不到桥可执行文件: {bridge}\n"
                  f"           先运行 {setup} 编译。", file=sys.stderr)
            sys.exit(2)
        env = dict(os.environ)
        # 保证能加载 MANUS SDK 动态库。Linux 使用 .so 搜索路径；Windows
        # SDK 的 DLL 可能在 bridge 同目录或 ManusSDK/bin/lib 中。
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
        if self.args.hand_motion:
            env["MANUS_HAND_MOTION"] = self.args.hand_motion
        print(f"[collector] 启动桥: {bridge}", flush=True)
        self._proc = subprocess.Popen(
            [str(bridge)],
            stdout=subprocess.PIPE,
            stderr=None,  # SDK 日志直通到当前终端
            bufsize=1,
            text=True,
            env=env,
        )

    # ------------------------------------------------------------------ hdf5
    def _ensure_h5(self):
        if not self.args.hdf5 or self._h5 is not None:
            return self._h5
        try:
            import h5py  # noqa: F401
            import numpy as np  # noqa: F401
        except Exception as e:  # noqa: BLE001
            print(f"[collector] 无法导入 h5py/numpy，跳过 HDF5 导出: {e}", file=sys.stderr)
            self.args.hdf5 = False
            return None
        from manus_hdf5 import ManusJsonlToHdf5  # 延迟导入本地精简写入器
        self._h5 = ManusJsonlToHdf5(self.hdf5_path, task_name=self.args.task)
        return self._h5

    # ------------------------------------------------------------------ reader
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
            collector_qpc_ns = time.perf_counter_ns()
            if "recv_wall_ns" not in obj:
                obj["recv_wall_ns"] = time.time_ns()
            if not obj.get("recv_qpc_ns"):
                # 旧 C++ bridge 的 recv_mono_ns 来自 steady_clock；Windows/MSVC
                # 与 Python perf_counter_ns() 同为 QueryPerformanceCounter。
                obj["recv_qpc_ns"] = int(
                    obj.get("recv_mono_ns") or collector_qpc_ns
                )
            obj["collector_recv_qpc_ns"] = collector_qpc_ns
            side = obj.get("side", "unknown")
            self._latest[side] = obj
            self._latest_rx[side] = time.time()
            if self._recording.is_set():
                # 只落盘所选侧; 在线状态仍保留双手便于 status
                if side in ("left", "right") and side not in self.hands:
                    continue
                with self._lock:
                    if self._fp is not None:
                        try:
                            self._write_queue.put_nowait((self._fp, self.session, obj))
                            self.max_write_backlog = max(
                                self.max_write_backlog, self._write_queue.qsize()
                            )
                        except queue.Full:
                            self.writer_errors += 1
                            print("[collector] MANUS 写盘队列溢出；本会话将拒绝封存",
                                  file=sys.stderr, flush=True)
        self._stop.set()

    # ------------------------------------------------------------------ status
    def _print_status(self, force: bool = False) -> None:
        if self.args.print_hz <= 0 and not force:
            return
        now = time.time()
        if not force and (now - self._last_status) < (1.0 / max(self.args.print_hz, 0.1)):
            return
        self._last_status = now
        rec = "REC" if self._recording.is_set() else "待机"
        parts = [f"[{rec}] 已存 "]
        for side in ("left", "right", "unknown"):
            if side in self._latest or self._counts.get(side):
                obj = self._latest.get(side, {})
                nc = obj.get("node_count", 0)
                parts.append(f"{side}:{self._counts.get(side,0)}帧/{nc}节点 ")
        sys.stdout.write("\r" + "".join(parts) + "        ")
        sys.stdout.flush()

    # ------------------------------------------------------------------ service 控制
    def start_session(self, name: str, hands=None) -> str:
        h = parse_hands(hands) if hands is not None else self.default_hands
        with self._lock:
            if self._recording.is_set() or self._stopping:
                return f"ERR already recording session={self.session}"
            self.session = name
            self.hands = h
            sess_dir = session_raw_dir(self.log_dir, name)
            sess_dir.mkdir(parents=True, exist_ok=True)
            self.jsonl_path = sess_dir / "manus.jsonl"
            self.manus_meta_path = sess_dir / "manus.meta.json"
            self.hdf5_path = (sess_dir / "manus.hdf5") if self.args.hdf5 else None
            self._fp = open(self.jsonl_path, "w", encoding="utf-8")
            self._counts = defaultdict(int)
            self.writer_errors = 0
            self.max_write_backlog = 0
            self._recording.set()
            self._write_calibration_meta()
        hl = hands_label(h)
        return f"OK recording session={name} hands={hl} file={self.jsonl_path}"

    def stop_session(self) -> str:
        with self._lock:
            if not self._recording.is_set():
                return "ERR not recording"
            self._recording.clear()
            self._stopping = True
            path = self.jsonl_path
            session = self.session
        if not wait_queue_drained(self._write_queue, 20.0):
            with self._lock:
                self.writer_errors += 1
                fp = self._fp
            threading.Thread(
                target=self._finish_delayed_stop,
                args=(fp, session),
                name="manus-jsonl-delayed-stop",
                daemon=True,
            ).start()
            return (f"ERR writer_errors={self.writer_errors} stopped session={session} "
                    f"frames={sum(self._counts.values())} file={path}")
        with self._lock:
            n = sum(self._counts.values())
            if self._fp is not None:
                self._fp.flush()
                self._fp.close()
                self._fp = None
            errors = self.writer_errors
            self._stopping = False
        self._write_calibration_meta()
        if errors:
            return (f"ERR writer_errors={errors} stopped session={session} "
                    f"frames={n} file={path}")
        return f"OK stopped session={self.session} frames={n} file={path}"

    def _finish_delayed_stop(self, fp, session: str) -> None:
        self._write_queue.join()
        with self._lock:
            if self._fp is fp and fp is not None:
                try:
                    fp.flush()
                    fp.close()
                except OSError:
                    self.writer_errors += 1
                self._fp = None
            self._stopping = False
        self._write_calibration_meta()
        print(f"[collector] delayed writer finalization completed: session={session}",
              flush=True)

    def status(self) -> str:
        now = time.time()
        rec = "STOPPING" if self._stopping else ("REC" if self._recording.is_set() else "idle")
        sides = [s for s in ("left", "right") if s in self._latest]
        online = ",".join(sides) or "(none)"
        hl = hands_label(self.hands if self._recording.is_set() else self.default_hands)
        ages = []
        nodes = []
        calibrations = []
        for s in ("left", "right"):
            if s not in self._latest:
                ages.append(f"{s[0]}=-1")
                nodes.append(f"{s[0]}n=0")
                calibrations.append(f"{s[0]}=0")
                continue
            ages.append(f"{s[0]}={int(max(0.0, (now - self._latest_rx.get(s, 0)) * 1000))}")
            nodes.append(f"{s[0]}n={int(self._latest[s].get('node_count') or 0)}")
            calibrations.append(
                f"{s[0]}={1 if self._latest[s].get('calibration_applied') is True else 0}"
            )
        return (f"OK {rec} session={self.session} "
                f"frames={sum(self._counts.values())} hands={hl} gloves={online} "
                f"age_ms={','.join(ages)} nodes={','.join(nodes)} "
                f"calib={','.join(calibrations)} "
                f"writer_errors={self.writer_errors} "
                f"write_backlog={self._write_queue.qsize()}")

    def _write_calibration_meta(self) -> None:
        path = getattr(self, "manus_meta_path", None)
        if path is None:
            return
        doc = {
            "schema": "manus_capture_meta_v1",
            "session": self.session,
            "calibration_required": True,
            "calibration": {},
        }
        for side in ("left", "right"):
            evidence = dict(self._calibration_files[side])
            latest = self._latest.get(side, {})
            evidence["glove_id"] = latest.get("glove_id")
            evidence["sdk_applied"] = latest.get("calibration_applied") is True
            doc["calibration"][side] = evidence
        temp = path.with_suffix(path.suffix + ".tmp")
        temp.write_text(json.dumps(doc, ensure_ascii=False, indent=2), encoding="utf-8")
        temp.replace(path)

    def run_service(self, control_port: int) -> None:
        """常驻: 桥保持连接, 录制由控制端口 START/STOP 控制。"""
        self._spawn_bridge()
        reader = threading.Thread(target=self._reader, daemon=True)
        reader.start()

        def _handler(cmd, arg):
            if cmd == "PING":
                return "PONG"
            if cmd == "START":
                session, hands = parse_start_arg(
                    arg.strip() or datetime.now().strftime("%Y%m%d_%H%M%S"),
                    default_hands=self.default_hands)
                if not session:
                    session = datetime.now().strftime("%Y%m%d_%H%M%S")
                return self.start_session(session, hands=hands)
            if cmd == "STOP":
                return self.stop_session()
            if cmd == "SHUTDOWN":
                if self._recording.is_set() or self._stopping:
                    return "ERR recording_active"
                self._stop.set()
                return "OK shutting_down"
            if cmd == "STATUS":
                return self.status()
            return f"ERR unknown command {cmd}"

        control = ControlServer(control_port, _handler)
        control.start()

        def _sig(_s, _f):
            self._stop.set()
        signal.signal(signal.SIGINT, _sig)
        signal.signal(signal.SIGTERM, _sig)

        print(f"[collector] service 模式: 控制端口 {control_port}; "
              f"默认 hands={hands_label(self.default_hands)}; "
              f"录制用 python3 pico_record.py start (同一终端 Ctrl+C/Enter 停)。"
              f"等待手套数据…", flush=True)
        print("[collector] 坐标: MANUS 原生右手系 X前Y左Z上(腕根); "
              "与转换后的 PICO 同系。本窗口 Ctrl+C=停服务。", flush=True)
        while not self._stop.is_set():
            self._print_status()
            if self._proc is not None and self._proc.poll() is not None:
                print("\n[collector] 桥进程已退出。", flush=True)
                break
            time.sleep(0.05)
        control.stop()
        control.join(timeout=2.0)
        if self._recording.is_set():
            self.stop_session()
        self._shutdown()

    # ------------------------------------------------------------------ run
    def run(self) -> None:
        if self.args.service:
            self.run_service(self.args.control_port or MANUS_CONTROL_PORT)
            return
        self._spawn_bridge()
        with open(self.jsonl_path, "w", encoding="utf-8") as fp:
            self._fp = fp
            reader = threading.Thread(target=self._reader, daemon=True)
            reader.start()

            def _sig(_s, _f):
                self._stop.set()
            signal.signal(signal.SIGINT, _sig)
            signal.signal(signal.SIGTERM, _sig)

            if not self._recording.is_set():
                # 等待手套上线再提示开始。
                print("[collector] 等待手套数据…（连接中，Ctrl+C 取消）", flush=True)
                t0 = time.time()
                while not self._latest and not self._stop.is_set():
                    self._print_status()
                    time.sleep(0.1)
                    if time.time() - t0 > 0.5:
                        self._print_status()
                print("\n[collector] 手套已上线。按 Enter 开始录制…", flush=True)
                try:
                    input()
                except EOFError:
                    pass
                if not self._stop.is_set():
                    self._recording.set()
                    print(f"[collector] 开始录制 -> {self.jsonl_path}", flush=True)

            deadline = (time.time() + self.args.duration) if self.args.duration > 0 else None
            while not self._stop.is_set():
                self._print_status()
                if deadline is not None and time.time() >= deadline:
                    print("\n[collector] 到达设定时长，停止。", flush=True)
                    break
                if self._proc is not None and self._proc.poll() is not None:
                    print("\n[collector] 桥进程已退出。", flush=True)
                    break
                time.sleep(0.05)

            self._shutdown()

    def _shutdown(self) -> None:
        self._stop.set()
        self._recording.clear()
        if not wait_queue_drained(self._write_queue, 20.0):
            self.writer_errors += 1
            print("[collector] shutdown writer drain exceeded 20s",
                  file=sys.stderr, flush=True)
        with self._lock:
            if self._fp is not None:
                self._fp.flush()
        if self._proc is not None and self._proc.poll() is None:
            try:
                self._proc.send_signal(signal.SIGINT)
                self._proc.wait(timeout=3)
            except Exception:  # noqa: BLE001
                try:
                    self._proc.kill()
                except Exception:  # noqa: BLE001
                    pass
        if self._h5 is not None:
            self._h5.close()
        total = sum(self._counts.values())
        print(f"\n[collector] 结束。共 {total} 帧 -> {self.jsonl_path}"
              + (f"（+ {self.hdf5_path}）" if self.args.hdf5 else ""), flush=True)


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="无 ROS 的 MANUS 手套采集器")
    ap.add_argument("--bridge", default=str(DEFAULT_BRIDGE), help="桥可执行文件路径")
    ap.add_argument("--log-dir", default="data/sessions",
                    help="任务会话根目录 (默认 data/sessions; 每会话 raw/ 子目录)")
    ap.add_argument("--task", default="manus_session", help="任务名（写入 HDF5 attrs）")
    ap.add_argument("--duration", type=float, default=0.0, help="录制秒数；0 表示手动停止")
    ap.add_argument("--no-wait", action="store_true", help="不等待 Enter，立即录制")
    ap.add_argument("--hdf5", action="store_true", help="额外导出 HDF5（需要 h5py）")
    ap.add_argument("--print-hz", type=float, default=5.0, help="状态刷新频率；0 关闭")
    ap.add_argument("--hand-motion", default=None, choices=["none", "auto", "imu"],
                    help="覆盖 HandMotion（默认 none，与采集管线一致）")
    ap.add_argument("--service", action="store_true",
                    help="常驻服务模式: 桥连一次保持不断, 录制由 pico_record.py "
                         "START/STOP 控制")
    ap.add_argument("--control-port", type=int, default=None,
                    help=f"service 模式控制端口 (默认 {MANUS_CONTROL_PORT}, 仅监听本机)")
    ap.add_argument("--hands", default="both", choices=list(HANDS_CHOICES),
                    help="默认只录哪只手套: left / right / both。"
                         "service 下可被 pico_record.py start --hands 覆盖")
    return ap


def main() -> None:
    args = build_parser().parse_args()
    Collector(args).run()


if __name__ == "__main__":
    main()
