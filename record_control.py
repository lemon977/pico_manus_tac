#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""record_control.py — 采集服务的极简控制通道(本机 TCP 文本协议)。

常驻服务(pico_receiver / manus_collector / tactile_collector --service)内嵌 ControlServer,
录制客户端(pico_record.py)用 send_command 发指令:
  START <session>   开始录制到以 session 命名的文件, 回复 OK/ERR
  STOP              停止录制并关闭文件
  STATUS            返回一行状态(是否在线/是否录制/已录帧数)
  PING              返回 PONG(探活)

协议: 每条指令一行(\n 结尾), 服务回一行文本。仅监听 127.0.0.1, 不涉外网。

左右手选择(采集/可视化共用):
  START <session> [hands=left|right|both]
  CLI: --hands left|right|both   (l/r/all 也认)
上述 hands 只裁 PICO/MANUS；触觉 START 仅带 session，并始终保存左右两侧全部原始通道。
"""
from __future__ import annotations

import socket
import threading
import time
from typing import Callable, FrozenSet, Optional, Tuple

DEFAULT_HOST = "127.0.0.1"
PICO_CONTROL_PORT = 63910
MANUS_CONTROL_PORT = 63911
TACTILE_CONTROL_PORT = 63912

HANDS_BOTH: FrozenSet[str] = frozenset({"left", "right"})
HANDS_CHOICES = ("left", "right", "both", "l", "r", "all")


def wait_queue_drained(work_queue, timeout: float) -> bool:
    """Bounded Queue.join equivalent; never hang a STOP command forever."""
    deadline = time.monotonic() + max(0.0, float(timeout))
    condition = work_queue.all_tasks_done
    with condition:
        while work_queue.unfinished_tasks:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            condition.wait(remaining)
    return True


def parse_hands(spec) -> FrozenSet[str]:
    """把 left/right/both/l/r/all/None 解析成 frozenset({'left'}|{...}|both)。"""
    if spec is None or spec is True:
        return HANDS_BOTH
    if isinstance(spec, (set, frozenset, list, tuple)):
        out = set()
        for x in spec:
            out |= set(parse_hands(x))
        return frozenset(out) or HANDS_BOTH
    s = str(spec).strip().lower()
    if s in ("", "both", "all", "lr", "rl", "left,right", "right,left"):
        return HANDS_BOTH
    if s in ("left", "l", "lh", "左手"):
        return frozenset({"left"})
    if s in ("right", "r", "rh", "右手"):
        return frozenset({"right"})
    raise ValueError(f"无效 --hands={spec!r}, 请用 left / right / both")


def hands_label(hands: FrozenSet[str]) -> str:
    if hands >= HANDS_BOTH:
        return "both"
    if hands == frozenset({"left"}):
        return "left"
    if hands == frozenset({"right"}):
        return "right"
    return ",".join(sorted(hands)) or "none"


def parse_start_arg(arg: str, default_hands: Optional[FrozenSet[str]] = None
                    ) -> Tuple[str, FrozenSet[str]]:
    """解析 START 参数: 'session' 或 'session hands=left' / 'session --hands left'。

    返回 (session_name, hands)。未写 hands 时用 default_hands(缺省 both)。
    """
    default = default_hands if default_hands is not None else HANDS_BOTH
    tokens = (arg or "").split()
    if not tokens:
        return "", default
    session = tokens[0]
    hands = default
    i = 1
    while i < len(tokens):
        t = tokens[i]
        if t.startswith("hands="):
            hands = parse_hands(t.split("=", 1)[1])
            i += 1
        elif t in ("--hands", "-H") and i + 1 < len(tokens):
            hands = parse_hands(tokens[i + 1])
            i += 2
        else:
            # 兼容旧客户端把多余词粘在 session 后: 忽略未知 token
            i += 1
    return session, hands


def filter_pico_record(rec: dict, hands: FrozenSet[str]) -> dict:
    """按 hands 裁掉未选侧的 Controller / 手势 / retarget 腕字段(浅拷贝改写)。"""
    if hands >= HANDS_BOTH:
        return rec
    out = dict(rec)
    data = out.get("data")
    if isinstance(data, dict):
        data = dict(data)
        ctrl = data.get("Controller")
        if isinstance(ctrl, dict):
            ctrl = dict(ctrl)
            for side in ("left", "right"):
                if side not in hands:
                    ctrl.pop(side, None)
            data["Controller"] = ctrl
        # App 手势跟踪字段(若有)
        if "left" not in hands:
            data.pop("leftHand", None)
        if "right" not in hands:
            data.pop("rightHand", None)
        out["data"] = data
    rt = out.get("retarget")
    if isinstance(rt, dict):
        rt = dict(rt)
        if "left" not in hands:
            rt.pop("left_wrist_rel", None)
        if "right" not in hands:
            rt.pop("right_wrist_rel", None)
        out["retarget"] = rt
    out["hands"] = hands_label(hands)
    return out


def send_command(port: int, line: str, host: str = DEFAULT_HOST, timeout: float = 5.0) -> str:
    """向服务发送一行指令并返回一行回复; 连接失败抛异常。"""
    with socket.create_connection((host, port), timeout=timeout) as s:
        s.settimeout(timeout)
        s.sendall((line.strip() + "\n").encode("utf-8"))
        buf = bytearray()
        while b"\n" not in buf:
            chunk = s.recv(4096)
            if not chunk:
                break
            buf += chunk
    return buf.decode("utf-8", "replace").strip()


class ControlServer(threading.Thread):
    """在后台线程监听控制端口, 把每条 (cmd, arg) 交给 handler 处理并回其返回值。

    handler(cmd: str, arg: str) -> str
    """

    def __init__(self, port: int, handler: Callable[[str, str], str],
                 host: str = DEFAULT_HOST):
        super().__init__(daemon=True)
        self.port = port
        self.host = host
        self.handler = handler
        self._sock = None
        self._stop_event = threading.Event()
        self._ready = threading.Event()
        self._bind_error: Optional[BaseException] = None

    def run(self) -> None:
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._sock.bind((self.host, self.port))
            self._sock.listen(4)
            self._sock.settimeout(1.0)
        except BaseException as exc:
            self._bind_error = exc
            self._ready.set()
            if self._sock is not None:
                try:
                    self._sock.close()
                except OSError:
                    pass
            raise
        self._ready.set()
        while not self._stop_event.is_set():
            try:
                conn, _ = self._sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with conn:
                try:
                    conn.settimeout(5.0)
                    data = b""
                    while b"\n" not in data:
                        chunk = conn.recv(4096)
                        if not chunk:
                            break
                        data += chunk
                    line = data.decode("utf-8", "replace").strip()
                    if not line:
                        continue
                    parts = line.split(None, 1)
                    cmd = parts[0].upper()
                    arg = parts[1] if len(parts) > 1 else ""
                    try:
                        reply = self.handler(cmd, arg)
                    except Exception as e:  # noqa: BLE001
                        reply = f"ERR {e!r}"
                    conn.sendall(((reply or "OK") + "\n").encode("utf-8"))
                except OSError:
                    pass

    def wait_ready(self, timeout: float = 5.0) -> None:
        """等待监听 socket 完成 bind；失败或超时则抛出 RuntimeError。"""
        if not self._ready.wait(timeout):
            raise RuntimeError(
                f"control server bind timeout: {self.host}:{self.port}"
            )
        if self._bind_error is not None:
            raise RuntimeError(
                f"control server bind failed: {self.host}:{self.port}: "
                f"{self._bind_error}"
            ) from self._bind_error

    def stop(self) -> None:
        self._stop_event.set()
        try:
            if self._sock is not None:
                self._sock.close()
        except OSError:
            pass
