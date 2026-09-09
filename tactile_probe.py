#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双路触觉硬件只读探测与稳定性测试。

默认自动发现 VID:PID=1A86:7523 的 CH340 串口，并把它们标记为本次进程内的
``candidate_1`` / ``candidate_2``。candidate 标签和 COM/tty 名称都不是设备身份，
也不代表左右手。

本脚本只允许发送四个读取功能码：0x06、0x07、0x0C、0x20。它不会修改设备
地址、增益、标定或其他持久化配置，也不会落盘采集数据。
"""
from __future__ import annotations

import argparse
import math
import os
import re
import signal
import statistics
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, List, Optional, Sequence, Set, Tuple

from tactile_protocol import (
    BAUD_RATE,
    DEFAULT_ADDRESS,
    FUNC_CELLMAP,
    FUNC_COLS,
    FUNC_FSR,
    FUNC_ROWS,
    READ_ONLY_FUNCTIONS,
    Frame,
    FrameStreamParser,
    GridConfig,
    ParserStats,
    PayloadError,
    decode_fsr,
    encode_frame,
    parse_dimension,
    parse_grid,
)


CH340_VID = 0x1A86
CH340_PID = 0x7523
EXPECTED_ROWS = 24
EXPECTED_COLS = 16
EXPECTED_ACTIVE = 369
MAX_LATENCY_SAMPLES = 10_000


class ProbeError(RuntimeError):
    """探测、串口或设备响应错误。"""


class ResponseTimeout(ProbeError):
    """设备未在期限内返回期望响应。"""


class ProbeStopped(ProbeError):
    """主线程要求正常停止当前探测。"""


@dataclass(frozen=True)
class PortCandidate:
    label: str
    device: str
    description: str = ""
    hwid: str = ""
    vid: Optional[int] = None
    pid: Optional[int] = None
    serial_number: Optional[str] = None
    location: Optional[str] = None

    @property
    def usb_id(self) -> str:
        if self.vid is None or self.pid is None:
            return "unknown"
        return "{:04X}:{:04X}".format(self.vid, self.pid)


@dataclass
class ProbeStats:
    attempts: int = 0
    valid_frames: int = 0
    timeouts: int = 0
    address_errors: int = 0
    function_errors: int = 0
    payload_errors: int = 0
    io_errors: int = 0
    missed_deadlines: int = 0
    first_frame_ns: int = 0
    last_frame_ns: int = 0
    value_count: int = 0
    value_sum: int = 0
    value_min: Optional[int] = None
    value_max: Optional[int] = None
    latencies_ms: Deque[float] = field(
        default_factory=lambda: deque(maxlen=MAX_LATENCY_SAMPLES)
    )
    payload_lengths: Set[int] = field(default_factory=set)
    frame_lengths: Set[int] = field(default_factory=set)


@dataclass(frozen=True)
class StatsSnapshot:
    attempts: int
    valid_frames: int
    timeouts: int
    address_errors: int
    function_errors: int
    payload_errors: int
    io_errors: int
    missed_deadlines: int
    first_frame_ns: int
    last_frame_ns: int
    value_count: int
    value_sum: int
    value_min: Optional[int]
    value_max: Optional[int]
    latencies_ms: Tuple[float, ...]
    payload_lengths: Tuple[int, ...]
    frame_lengths: Tuple[int, ...]

    @property
    def fps(self) -> float:
        if self.valid_frames < 2 or self.last_frame_ns <= self.first_frame_ns:
            return 0.0
        elapsed = (self.last_frame_ns - self.first_frame_ns) / 1_000_000_000.0
        return (self.valid_frames - 1) / elapsed if elapsed > 0 else 0.0

    @property
    def value_mean(self) -> float:
        return self.value_sum / self.value_count if self.value_count else float("nan")


def _load_pyserial() -> Tuple[Any, Any]:
    try:
        import serial  # type: ignore
        from serial.tools import list_ports  # type: ignore
    except ImportError as exc:
        raise ProbeError(
            "缺少 pyserial。请在实际采集 Python 环境执行: "
            "python3 -m pip install pyserial"
        ) from exc
    return serial, list_ports


def _natural_port_key(value: str) -> Tuple[Any, ...]:
    parts = re.split(r"(\d+)", str(value).lower())
    return tuple(int(part) if part.isdigit() else part for part in parts)


def _candidate_from_info(label: str, info: Any) -> PortCandidate:
    return PortCandidate(
        label=label,
        device=str(getattr(info, "device", "")),
        description=str(getattr(info, "description", "") or ""),
        hwid=str(getattr(info, "hwid", "") or ""),
        vid=getattr(info, "vid", None),
        pid=getattr(info, "pid", None),
        serial_number=getattr(info, "serial_number", None),
        location=getattr(info, "location", None),
    )


def discover_candidates(
    list_ports_module: Any,
    explicit_ports: Sequence[str],
) -> Tuple[List[PortCandidate], List[PortCandidate]]:
    """返回 (CH340候选, 全部串口)；显式端口仍需后续协议探测。"""
    infos = list(list_ports_module.comports())
    infos.sort(key=lambda item: _natural_port_key(getattr(item, "device", "")))
    all_ports = [
        _candidate_from_info("port_{}".format(index + 1), info)
        for index, info in enumerate(infos)
    ]

    if explicit_ports:
        devices = [str(port).strip() for port in explicit_ports if str(port).strip()]
        if len(devices) != len(set(device.lower() for device in devices)):
            raise ProbeError("--port 包含重复串口")
        by_name: Dict[str, Any] = {
            str(getattr(info, "device", "")).lower(): info for info in infos
        }
        selected: List[PortCandidate] = []
        for index, device in enumerate(devices):
            info = by_name.get(device.lower())
            if info is None:
                selected.append(PortCandidate(label="candidate_{}".format(index + 1), device=device))
            else:
                selected.append(_candidate_from_info("candidate_{}".format(index + 1), info))
        return selected, all_ports

    matches = [
        info
        for info in infos
        if getattr(info, "vid", None) == CH340_VID
        and getattr(info, "pid", None) == CH340_PID
    ]
    candidates = [
        _candidate_from_info("candidate_{}".format(index + 1), info)
        for index, info in enumerate(matches)
    ]
    return candidates, all_ports


def _format_optional(value: Any) -> str:
    return str(value) if value not in (None, "") else "-"


def print_port_table(ports: Sequence[PortCandidate], title: str) -> None:
    print(title)
    if not ports:
        print("  (none)")
        return
    for port in ports:
        print(
            "  {:<12} port={} usb={} serial={} location={} desc={}".format(
                port.label,
                port.device,
                port.usb_id,
                _format_optional(port.serial_number),
                _format_optional(port.location),
                port.description or "-",
            )
        )


class ProbeWorker(threading.Thread):
    """一个串口对应一个独占线程；同端口始终只有一个待响应请求。"""

    def __init__(
        self,
        candidate: PortCandidate,
        serial_module: Any,
        address: int,
        rate_hz: float,
        response_timeout_ms: int,
        start_event: threading.Event,
        stop_event: threading.Event,
    ) -> None:
        # daemon 仅是串口驱动卡死时的最后退路；正常路径仍会 join 并 close。
        super().__init__(name="tactile-{}".format(candidate.label), daemon=True)
        self.candidate = candidate
        self.serial_module = serial_module
        self.address = address
        self.rate_hz = rate_hz
        self.response_timeout_ms = response_timeout_ms
        self.start_event = start_event
        self.stop_event = stop_event
        self.ready_event = threading.Event()
        self.grid: Optional[GridConfig] = None
        self.error: Optional[str] = None
        self.parser = FrameStreamParser()
        self.stats = ProbeStats()
        self._stats_lock = threading.Lock()
        self._parser_lock = threading.Lock()
        self._serial = None

    def snapshot(self) -> StatsSnapshot:
        with self._stats_lock:
            stats = self.stats
            return StatsSnapshot(
                attempts=stats.attempts,
                valid_frames=stats.valid_frames,
                timeouts=stats.timeouts,
                address_errors=stats.address_errors,
                function_errors=stats.function_errors,
                payload_errors=stats.payload_errors,
                io_errors=stats.io_errors,
                missed_deadlines=stats.missed_deadlines,
                first_frame_ns=stats.first_frame_ns,
                last_frame_ns=stats.last_frame_ns,
                value_count=stats.value_count,
                value_sum=stats.value_sum,
                value_min=stats.value_min,
                value_max=stats.value_max,
                latencies_ms=tuple(stats.latencies_ms),
                payload_lengths=tuple(sorted(stats.payload_lengths)),
                frame_lengths=tuple(sorted(stats.frame_lengths)),
            )

    def parser_snapshot(self) -> ParserStats:
        with self._parser_lock:
            stats = self.parser.stats
            return ParserStats(
                frames=stats.frames,
                discarded_bytes=stats.discarded_bytes,
                length_errors=stats.length_errors,
                tail_errors=stats.tail_errors,
                crc_errors=stats.crc_errors,
                buffer_overflows=stats.buffer_overflows,
            )

    def _open_serial(self) -> Any:
        serial = self.serial_module
        port = serial.Serial(
            port=None,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=min(0.02, self.response_timeout_ms / 1000.0),
            write_timeout=1.0,
        )
        self._serial = port
        try:
            port.port = self.candidate.device
            if os.name != "nt" and hasattr(port, "exclusive"):
                port.exclusive = True
            port.dtr = False
            port.rts = False
            port.open()
            port.reset_input_buffer()
            port.reset_output_buffer()
        except BaseException:
            try:
                port.close()
            except Exception:
                pass
            self._serial = None
            raise
        return port

    def interrupt_io(self) -> None:
        """尝试从主线程取消底层 I/O，只用于 join 超时后的安全退出。"""
        port = self._serial
        if port is None:
            return
        for method_name in ("cancel_read", "cancel_write"):
            method = getattr(port, method_name, None)
            if callable(method):
                try:
                    method()
                except Exception:
                    pass
        try:
            port.close()
        except Exception:
            pass

    def _transact(self, function: int) -> Tuple[Frame, int, int]:
        if function not in READ_ONLY_FUNCTIONS:
            raise ProbeError("安全拒绝：功能码 0x{:02X} 不在只读白名单".format(function))
        if self.stop_event.is_set():
            raise ProbeStopped("探测已停止")
        if self._serial is None:
            raise ProbeError("串口尚未打开")

        request = encode_frame(self.address, function)
        sent_ns = time.monotonic_ns()
        written = self._serial.write(request)
        if written != len(request):
            raise ProbeError("串口短写: {} / {} bytes".format(written, len(request)))

        deadline_ns = sent_ns + self.response_timeout_ms * 1_000_000
        while not self.stop_event.is_set() and time.monotonic_ns() < deadline_ns:
            waiting = int(getattr(self._serial, "in_waiting", 0) or 0)
            chunk = self._serial.read(min(max(waiting, 1), 4096))
            if not chunk:
                continue
            with self._parser_lock:
                frames = self.parser.feed(chunk)
            for frame in frames:
                if frame.address != self.address:
                    with self._stats_lock:
                        self.stats.address_errors += 1
                    continue
                if frame.function != function:
                    with self._stats_lock:
                        self.stats.function_errors += 1
                    continue
                return frame, sent_ns, time.monotonic_ns()

        if self.stop_event.is_set():
            raise ProbeStopped("探测已停止")
        with self._parser_lock:
            self.parser.reset()
        try:
            self._serial.reset_input_buffer()
        except Exception:
            pass
        raise ResponseTimeout(
            "{} 等待 address=0x{:02X}, function=0x{:02X} 超时".format(
                self.candidate.device, self.address, function
            )
        )

    def _query_grid(self) -> GridConfig:
        row_frame, _, _ = self._transact(FUNC_ROWS)
        rows = parse_dimension(row_frame, self.address, FUNC_ROWS)
        col_frame, _, _ = self._transact(FUNC_COLS)
        cols = parse_dimension(col_frame, self.address, FUNC_COLS)
        map_frame, _, _ = self._transact(FUNC_CELLMAP)
        return parse_grid(map_frame, self.address, rows, cols)

    def _record_sample(self, frame: Frame, sent_ns: int, received_ns: int) -> None:
        if self.grid is None:
            raise ProbeError("缺少 grid 配置")
        sample = decode_fsr(frame, self.address, self.grid)
        values = sample.wire_values
        with self._stats_lock:
            stats = self.stats
            stats.valid_frames += 1
            if stats.first_frame_ns == 0:
                stats.first_frame_ns = received_ns
            stats.last_frame_ns = received_ns
            stats.latencies_ms.append((received_ns - sent_ns) / 1_000_000.0)
            stats.payload_lengths.add(len(frame.payload))
            stats.frame_lengths.add(len(frame.raw))
            if values:
                local_min = min(values)
                local_max = max(values)
                stats.value_min = local_min if stats.value_min is None else min(stats.value_min, local_min)
                stats.value_max = local_max if stats.value_max is None else max(stats.value_max, local_max)
                stats.value_sum += sum(values)
                stats.value_count += len(values)

    def _sample_loop(self) -> None:
        period_ns = max(1, int(1_000_000_000.0 / self.rate_hz))
        next_request_ns = time.monotonic_ns()
        while not self.stop_event.is_set():
            now_ns = time.monotonic_ns()
            if now_ns < next_request_ns:
                # Event.wait 的短超时在 Windows 上会被系统定时粒度拖长；使用
                # Python 高分辨率 sleep 才能稳定验证60Hz。最多多等一个周期后退出。
                time.sleep((next_request_ns - now_ns) / 1_000_000_000.0)
                continue

            with self._stats_lock:
                self.stats.attempts += 1
            try:
                frame, sent_ns, received_ns = self._transact(FUNC_FSR)
                self._record_sample(frame, sent_ns, received_ns)
            except ProbeStopped:
                break
            except ResponseTimeout:
                with self._stats_lock:
                    self.stats.timeouts += 1
            except PayloadError:
                with self._stats_lock:
                    self.stats.payload_errors += 1
            except Exception:
                with self._stats_lock:
                    self.stats.io_errors += 1
                raise

            next_request_ns += period_ns
            now_ns = time.monotonic_ns()
            if next_request_ns < now_ns:
                # 轻微迟到不再主动丢弃整个周期；保持绝对相位，在单请求/单响应
                # 约束下有界追赶。只有真正落后整周期才计 missed。
                missed = int((now_ns - next_request_ns) // period_ns)
                if missed:
                    with self._stats_lock:
                        self.stats.missed_deadlines += missed
                    next_request_ns += missed * period_ns

    def run(self) -> None:
        try:
            self._open_serial()
            self.grid = self._query_grid()
            if (
                self.grid.rows != EXPECTED_ROWS
                or self.grid.cols != EXPECTED_COLS
                or self.grid.active_count != EXPECTED_ACTIVE
            ):
                raise ProbeError(
                    "{} 网格不符合当前硬件规格: {}x{}, active={}".format(
                        self.candidate.device,
                        self.grid.rows,
                        self.grid.cols,
                        self.grid.active_count,
                    )
                )
            self.ready_event.set()
            while not self.start_event.wait(0.1):
                if self.stop_event.is_set():
                    return
            if not self.stop_event.is_set():
                self._sample_loop()
        except Exception as exc:
            self.error = "{}: {}".format(type(exc).__name__, exc)
            self.stop_event.set()
        finally:
            self.ready_event.set()
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass


def _percentile(values: Sequence[float], fraction: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, int(math.ceil(fraction * len(ordered))) - 1))
    return ordered[index]


def _number(value: float, digits: int = 2) -> str:
    return "-" if not math.isfinite(value) else format(value, ",.{}f".format(digits))


def print_progress(workers: Sequence[ProbeWorker]) -> None:
    parts = []
    for worker in workers:
        stats = worker.snapshot()
        parser_stats = worker.parser_snapshot()
        parts.append(
            "{}({}) valid={} fps={:.1f} timeout={} crc={}".format(
                worker.candidate.label,
                worker.candidate.device,
                stats.valid_frames,
                stats.fps,
                stats.timeouts,
                parser_stats.crc_errors,
            )
        )
    print("[progress] " + " | ".join(parts), flush=True)


def worker_passed(worker: ProbeWorker, duration: float, rate_hz: float) -> Tuple[bool, List[str]]:
    stats = worker.snapshot()
    parser_stats = worker.parser_snapshot()
    reasons: List[str] = []
    if worker.error:
        reasons.append(worker.error)
    if worker.grid is None:
        reasons.append("缺少 grid")
    if duration > 0:
        minimum = max(1, int(duration * rate_hz * 0.90))
        if stats.valid_frames < minimum:
            reasons.append("有效帧不足: {} < {}".format(stats.valid_frames, minimum))
    elif stats.valid_frames < 1:
        reasons.append("没有有效帧")
    if stats.timeouts:
        reasons.append("timeouts={}".format(stats.timeouts))
    if stats.address_errors:
        reasons.append("address_errors={}".format(stats.address_errors))
    if stats.function_errors:
        reasons.append("function_errors={}".format(stats.function_errors))
    if stats.payload_errors:
        reasons.append("payload_errors={}".format(stats.payload_errors))
    if stats.io_errors:
        reasons.append("io_errors={}".format(stats.io_errors))
    if stats.valid_frames >= 2 and stats.fps < rate_hz * 0.90:
        reasons.append("实测帧率不足: {:.2f} < {:.2f} Hz".format(stats.fps, rate_hz * 0.90))
    missed_limit = max(3, int(math.ceil(stats.attempts * 0.05)))
    if stats.missed_deadlines > missed_limit:
        reasons.append(
            "missed_deadlines={} > {}".format(stats.missed_deadlines, missed_limit)
        )
    if parser_stats.length_errors or parser_stats.tail_errors or parser_stats.crc_errors:
        reasons.append(
            "parser length/tail/crc={}/{}/{}".format(
                parser_stats.length_errors, parser_stats.tail_errors, parser_stats.crc_errors
            )
        )
    if parser_stats.discarded_bytes:
        reasons.append("parser discarded_bytes={}".format(parser_stats.discarded_bytes))
    if parser_stats.buffer_overflows:
        reasons.append("parser buffer_overflows={}".format(parser_stats.buffer_overflows))
    if stats.payload_lengths and stats.payload_lengths != (EXPECTED_ACTIVE * 2,):
        reasons.append("payload_lengths={}".format(stats.payload_lengths))
    if stats.frame_lengths and stats.frame_lengths != (EXPECTED_ACTIVE * 2 + 10,):
        reasons.append("frame_lengths={}".format(stats.frame_lengths))
    return not reasons, reasons


def print_summary(worker: ProbeWorker, duration: float, rate_hz: float) -> bool:
    stats = worker.snapshot()
    parser_stats = worker.parser_snapshot()
    passed, reasons = worker_passed(worker, duration, rate_hz)
    latency_median = statistics.median(stats.latencies_ms) if stats.latencies_ms else float("nan")
    latency_p95 = _percentile(stats.latencies_ms, 0.95)
    latency_max = max(stats.latencies_ms) if stats.latencies_ms else float("nan")
    grid = worker.grid

    print("\n[{}] {}  {}".format(worker.candidate.label, worker.candidate.device, "PASS" if passed else "FAIL"))
    if grid is not None:
        print("  grid={}x{} active={}".format(grid.rows, grid.cols, grid.active_count))
    print(
        "  attempts={} valid={} fps={:.2f} timeouts={} missed_deadlines={}".format(
            stats.attempts,
            stats.valid_frames,
            stats.fps,
            stats.timeouts,
            stats.missed_deadlines,
        )
    )
    print(
        "  latency_ms median={} p95={} max={}".format(
            _number(latency_median), _number(latency_p95), _number(latency_max)
        )
    )
    print(
        "  values min={} max={} mean={} payload_lengths={} frame_lengths={}".format(
            _format_optional(stats.value_min),
            _format_optional(stats.value_max),
            _number(stats.value_mean, 3),
            list(stats.payload_lengths),
            list(stats.frame_lengths),
        )
    )
    print(
        "  parser frames={} discarded={} length_errors={} tail_errors={} crc_errors={} overflows={}".format(
            parser_stats.frames,
            parser_stats.discarded_bytes,
            parser_stats.length_errors,
            parser_stats.tail_errors,
            parser_stats.crc_errors,
            parser_stats.buffer_overflows,
        )
    )
    if reasons:
        print("  reasons=" + "; ".join(reasons))
    return passed


def parse_hex_byte(value: str) -> int:
    text = str(value).strip().lower()
    if text.startswith("0x"):
        text = text[2:]
    try:
        parsed = int(text, 16)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("地址必须是十六进制字节，例如 0A") from exc
    if parsed < 0 or parsed > 0xFF:
        raise argparse.ArgumentTypeError("地址必须在 00..FF")
    return parsed


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="CH340 触觉设备的只读双路探测与稳定性测试（不判断左右）"
    )
    parser.add_argument(
        "--port",
        action="append",
        default=[],
        metavar="PORT",
        help="显式串口，可重复；省略时自动匹配 VID:PID=1A86:7523",
    )
    parser.add_argument("--list", action="store_true", help="只列出串口，不打开设备")
    parser.add_argument(
        "--expect-devices",
        type=int,
        default=2,
        help="要求的设备数，默认 2；数量不符则拒绝采集",
    )
    parser.add_argument("--duration", type=float, default=10.0, help="采集秒数，默认 10；0 表示直到 Ctrl+C")
    parser.add_argument("--rate-hz", type=float, default=60.0, help="每路请求频率，默认 60 Hz")
    parser.add_argument(
        "--response-timeout-ms",
        type=int,
        default=200,
        help="单次响应超时，默认 200 ms",
    )
    parser.add_argument("--progress-hz", type=float, default=1.0, help="进度打印频率；0 表示关闭")
    parser.add_argument(
        "--address",
        type=parse_hex_byte,
        default=DEFAULT_ADDRESS,
        help="只读查询地址（十六进制），默认 0A",
    )
    return parser


def _validate_args(args: argparse.Namespace) -> None:
    if args.expect_devices < 1:
        raise ProbeError("--expect-devices 必须 >= 1")
    if not math.isfinite(args.duration) or args.duration < 0:
        raise ProbeError("--duration 必须 >= 0")
    if not math.isfinite(args.rate_hz) or args.rate_hz <= 0 or args.rate_hz > 200:
        raise ProbeError("--rate-hz 必须在 (0, 200]")
    if args.response_timeout_ms < 10 or args.response_timeout_ms > 5000:
        raise ProbeError("--response-timeout-ms 必须在 10..5000")
    if not math.isfinite(args.progress_hz) or args.progress_hz < 0 or args.progress_hz > 20:
        raise ProbeError("--progress-hz 必须在 0..20")


def run_probe(args: argparse.Namespace) -> int:
    _validate_args(args)
    serial, list_ports = _load_pyserial()
    candidates, all_ports = discover_candidates(list_ports, args.port)

    if args.list:
        print_port_table(all_ports, "全部串口：")
        print_port_table(candidates, "选中的 CH340/显式候选：")
        return 0

    if len(candidates) != args.expect_devices:
        print_port_table(all_ports, "当前串口：")
        raise ProbeError(
            "候选设备数为 {}，但 --expect-devices={}；拒绝打开串口".format(
                len(candidates), args.expect_devices
            )
        )

    print("注意：candidate 标签只在本次进程有效，不代表左右手，也不能持久化。")
    print_port_table(candidates, "准备探测：")
    print(
        "只读白名单=06,07,0C,20 address=0x{:02X} baud={} rate={:.1f}Hz duration={}s".format(
            args.address, BAUD_RATE, args.rate_hz, args.duration
        )
    )

    start_event = threading.Event()
    stop_event = threading.Event()
    workers = [
        ProbeWorker(
            candidate=candidate,
            serial_module=serial,
            address=args.address,
            rate_hz=args.rate_hz,
            response_timeout_ms=args.response_timeout_ms,
            start_event=start_event,
            stop_event=stop_event,
        )
        for candidate in candidates
    ]
    started_workers: List[ProbeWorker] = []
    stuck: List[str] = []

    def request_stop(_signum: int, _frame: Any) -> None:
        stop_event.set()

    old_sigint = signal.signal(signal.SIGINT, request_stop)
    old_sigterm = None
    if hasattr(signal, "SIGTERM"):
        old_sigterm = signal.signal(signal.SIGTERM, request_stop)

    try:
        for worker in workers:
            try:
                worker.start()
            except RuntimeError as exc:
                raise ProbeError(
                    "无法启动 {} 的探测线程".format(worker.candidate.device)
                ) from exc
            started_workers.append(worker)

        init_deadline = time.monotonic() + 10.0
        for worker in workers:
            remaining = init_deadline - time.monotonic()
            if remaining > 0:
                worker.ready_event.wait(remaining)

        init_errors = [worker for worker in workers if worker.error or worker.grid is None]
        if init_errors:
            stop_event.set()
            start_event.set()
            for worker in started_workers:
                worker.join(3.0)
            for worker in init_errors:
                print(
                    "[init FAIL] {} {}: {}".format(
                        worker.candidate.label,
                        worker.candidate.device,
                        worker.error or "初始化超时",
                    ),
                    file=sys.stderr,
                )
            return 1

        for worker in workers:
            assert worker.grid is not None
            print(
                "[ready] {} {} grid={}x{} active={}".format(
                    worker.candidate.label,
                    worker.candidate.device,
                    worker.grid.rows,
                    worker.grid.cols,
                    worker.grid.active_count,
                )
            )

        run_start = time.monotonic()
        start_event.set()
        next_progress = run_start
        progress_period = 1.0 / args.progress_hz if args.progress_hz > 0 else None
        while not stop_event.is_set():
            now = time.monotonic()
            if args.duration > 0 and now - run_start >= args.duration:
                stop_event.set()
                break
            if progress_period is not None and now >= next_progress:
                print_progress(workers)
                next_progress = now + progress_period
            if any(worker.error for worker in workers):
                stop_event.set()
                break
            time.sleep(0.05)

    except KeyboardInterrupt:
        stop_event.set()
    finally:
        stop_event.set()
        start_event.set()
        for worker in started_workers:
            worker.join(5.0)
        still_running = [worker for worker in started_workers if worker.is_alive()]
        for worker in still_running:
            worker.interrupt_io()
        for worker in still_running:
            worker.join(2.0)
        stuck = [worker.candidate.device for worker in started_workers if worker.is_alive()]
        if stuck:
            print("[FAIL] 线程未按时退出: {}".format(", ".join(stuck)), file=sys.stderr)
        signal.signal(signal.SIGINT, old_sigint)
        if old_sigterm is not None:
            signal.signal(signal.SIGTERM, old_sigterm)

    if stuck:
        return 1

    passed = [print_summary(worker, args.duration, args.rate_hz) for worker in workers]
    overall = all(passed)
    print("\nOVERALL {}".format("PASS" if overall else "FAIL"))
    return 0 if overall else 1


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return run_probe(args)
    except ProbeError as exc:
        print("[probe] {}".format(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
