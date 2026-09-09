#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""独立的双手触觉原始数据采集器。

本模块只保存设备按 wire-active 顺序返回的全部原始值。它不做五指分组、死点
过滤、24x16 矩阵展开、4x7 掩码、基线校正或空间方向解释。

启动时必须在同一个 :class:`tactile_pairing.PairingSession` 内完成“左手按压识别、
右手由两路候选排除绑定”的会话级配对，之后持续持有原串口。单次响应超时或解析
器丢弃字节会记录为可恢复告警；短窗口内重复达到阈值、任一路断开、协议/CRC错误
或订阅读取落后才会锁存故障。采集器不会自动重开串口，也不会按 COM 口复用旧左右
身份。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import signal
import sys
import tempfile
import threading
import time
import uuid
from collections import deque
from contextlib import ExitStack
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

from record_control import ControlServer, TACTILE_CONTROL_PORT
from session_layout import session_raw_dir
from tactile_pairing import (
    DEFAULT_CONFIG_PATH,
    FrameGapError,
    MANUAL_PAIRING_RESULT_METHOD,
    PairedSubscriptions,
    PairingConfig,
    PairingError,
    PairingRejected,
    PairingResult,
    PairingSession,
    ReaderHealth,
    TimedFsrFrame,
    TRANSIENT_HEALTH_FIELDS,
    _load_pyserial,
    configured_manus,
    grid_cellmap_bytes,
    grid_cellmap_sha256,
    load_pairing_config,
    pair_by_press,
    require_raw_capture_ready,
)
from tactile_probe import discover_candidates, parse_hex_byte, print_port_table
from tactile_protocol import DEFAULT_ADDRESS, FUNC_FSR
from tactile_layout import (
    TACTILE_LAYOUT_SCHEMA,
    TACTILE_LAYOUT_SOURCE,
    finger_region_metadata,
)


HERE = Path(__file__).resolve().parent
DEFAULT_LOG_DIR = HERE / "data" / "sessions"

FRAME_SCHEMA = "pico_tactile_wire_frame_v1"
META_SCHEMA = "pico_tactile_raw_meta_v1"
SIDE_NAMES = ("left", "right")
DEFAULT_TRANSIENT_INCIDENT_LIMIT = 3
DEFAULT_TRANSIENT_WINDOW_S = 5.0

STATE_PAIRED_IDLE = "PAIRED_IDLE"
STATE_RECORDING = "RECORDING_RAW"
STATE_STOPPING = "STOPPING_RAW"
STATE_FAULT = "FAULT_LATCHED"
STATE_SHUTDOWN = "SHUTDOWN"

_WINDOWS_FORBIDDEN = frozenset('<>:"/\\|?*')
_STATE_TOKEN_RE = re.compile(r"[^A-Za-z0-9_.-]+")


class CollectorError(RuntimeError):
    """采集器状态、写盘或控制命令错误。"""


class TransientHealthTracker:
    """把偶发 timeout/parser resync 记为告警，重复达到阈值才升级故障。"""

    def __init__(self, incident_limit: int, window_s: float) -> None:
        if (
            isinstance(incident_limit, bool)
            or not isinstance(incident_limit, int)
            or incident_limit < 1
            or incident_limit > 100
        ):
            raise CollectorError("transient_incident_limit 必须在1..100")
        if not math.isfinite(window_s) or window_s <= 0 or window_s > 60.0:
            raise CollectorError("transient_window_s 必须在(0,60]")
        self.incident_limit = int(incident_limit)
        self.window_s = float(window_s)
        self._lock = threading.Lock()
        self._events = {side: deque() for side in SIDE_NAMES}
        self._incidents = {side: 0 for side in SIDE_NAMES}
        self._counters = {
            side: {name: 0 for name in TRANSIENT_HEALTH_FIELDS}
            for side in SIDE_NAMES
        }
        self._last_event: Optional[Dict[str, Any]] = None

    def _prune_locked(self, side: str, now_mono: float) -> None:
        cutoff = now_mono - self.window_s
        events = self._events[side]
        while events and events[0] < cutoff:
            events.popleft()

    def observe(
        self,
        side: str,
        health_delta: ReaderHealth,
        now_mono: Optional[float] = None,
    ) -> Optional[Dict[str, Any]]:
        normalized = str(side).lower()
        if normalized not in SIDE_NAMES:
            raise CollectorError("未知 transient health side={}".format(side))
        values = {
            name: max(0, int(getattr(health_delta, name)))
            for name in TRANSIENT_HEALTH_FIELDS
        }
        incident_count = max(
            values["timeouts"],
            int(values["parser_discarded"] > 0),
        )
        if incident_count <= 0:
            return None
        now = time.monotonic() if now_mono is None else float(now_mono)
        with self._lock:
            self._prune_locked(normalized, now)
            self._events[normalized].extend(now for _ in range(incident_count))
            self._incidents[normalized] += incident_count
            for name, value in values.items():
                self._counters[normalized][name] += value
            active = len(self._events[normalized])
            event = {
                "side": normalized,
                "wall_ns": time.time_ns(),
                "incidents": int(incident_count),
                "active_window_incidents": int(active),
                "timeouts": int(values["timeouts"]),
                "parser_discarded": int(values["parser_discarded"]),
                "fault_threshold_reached": bool(active >= self.incident_limit),
            }
            self._last_event = dict(event)
            return event

    def snapshot(self, now_mono: Optional[float] = None) -> Dict[str, Any]:
        now = time.monotonic() if now_mono is None else float(now_mono)
        with self._lock:
            for side in SIDE_NAMES:
                self._prune_locked(side, now)
            return {
                "incident_limit": int(self.incident_limit),
                "window_s": float(self.window_s),
                "incident_count_by_side": dict(self._incidents),
                "active_window_incidents_by_side": {
                    side: len(self._events[side]) for side in SIDE_NAMES
                },
                "counters_by_side": {
                    side: dict(self._counters[side]) for side in SIDE_NAMES
                },
                "last_event": (
                    dict(self._last_event) if self._last_event is not None else None
                ),
            }


def _json_bytes(document: Mapping[str, Any], pretty: bool) -> bytes:
    if pretty:
        text = json.dumps(
            document,
            ensure_ascii=False,
            allow_nan=False,
            indent=2,
            sort_keys=True,
        )
    else:
        text = json.dumps(
            document,
            ensure_ascii=False,
            allow_nan=False,
            separators=(",", ":"),
            sort_keys=False,
        )
    return (text + "\n").encode("utf-8")


def _fsync_directory(path: Path) -> None:
    if os.name == "nt":
        return
    descriptor = os.open(str(path), os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _atomic_write_json(path: Path, document: Mapping[str, Any]) -> None:
    """在同目录原子提交 JSON，并回读验证提交内容。"""
    target = Path(path)
    if target.is_symlink():
        raise CollectorError("拒绝写入符号链接: {}".format(target))
    raw = _json_bytes(document, pretty=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=".{}-".format(target.name),
        suffix=".tmp",
        dir=str(target.parent),
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            written = stream.write(raw)
            if written != len(raw):
                raise CollectorError("meta 短写: {} / {} bytes".format(written, len(raw)))
            stream.flush()
            os.fsync(stream.fileno())
        with temporary.open("rb") as stream:
            reread = stream.read(len(raw) + 1)
        if reread != raw:
            raise CollectorError("meta 临时文件回读不一致")
        os.replace(str(temporary), str(target))
        with target.open("rb") as stream:
            committed = stream.read(len(raw) + 1)
        if committed != raw:
            raise CollectorError("meta 提交后回读不一致")
        _fsync_directory(target.parent)
    finally:
        if temporary.exists():
            try:
                temporary.unlink()
            except OSError:
                pass


def _json_file_matches(path: Path, document: Mapping[str, Any]) -> bool:
    """判断目标是否已经精确提交；用于识别 replace 后的校验/fsync 异常。"""
    target = Path(path)
    if target.is_symlink():
        return False
    expected = _json_bytes(document, pretty=True)
    try:
        with target.open("rb") as stream:
            return stream.read(len(expected) + 1) == expected
    except OSError:
        return False


def _commit_json(path: Path, document: Mapping[str, Any]) -> None:
    """提交 JSON；若异常发生在 replace 之后，以目标精确内容作为提交结论。"""
    try:
        _atomic_write_json(path, document)
    except Exception:
        if _json_file_matches(path, document):
            # 目标内容可见不等于 rename 的目录项已持久；Unix 上必须重试目录 fsync。
            _fsync_directory(Path(path).parent)
            return
        raise


def _health_dict(health: Any) -> Dict[str, int]:
    return {
        str(name): int(getattr(health, name))
        for name in getattr(health, "__dataclass_fields__", {})
    }


def _validate_session_name(value: str) -> str:
    name = str(value or "").strip()
    if not name:
        raise CollectorError("session 名不能为空")
    if len(name) > 128:
        raise CollectorError("session 名不能超过128个字符")
    if name in (".", "..") or name.endswith((" ", ".")):
        raise CollectorError("session 名不安全: {!r}".format(name))
    if any(ord(char) < 32 or char in _WINDOWS_FORBIDDEN for char in name):
        raise CollectorError("session 名含路径分隔符、控制字符或非法字符")
    return name


def _fault_token(message: str) -> str:
    token = _STATE_TOKEN_RE.sub("_", str(message)).strip("_")
    return (token or "fault")[:120]


def _pairing_prompt(phase: str, message: str) -> None:
    print("\n[{}] {}".format(phase, message), flush=True)
    try:
        input("准备好后按 Enter；看到“开始采集”后执行上述动作: ")
    except EOFError as exc:
        raise PairingError("交互输入已关闭") from exc
    if phase.endswith("_press"):
        print("  3...", flush=True)
        time.sleep(0.5)
        print("  2...", flush=True)
        time.sleep(0.5)
        print("  1...", flush=True)
        time.sleep(0.5)
    print("  开始采集。", flush=True)


class TactileCollector:
    """消费配对会话的两路订阅，并由唯一 writer 线程落盘。"""

    def __init__(
        self,
        session: PairingSession,
        pairing: PairingResult,
        config: PairingConfig,
        streams: PairedSubscriptions,
        log_dir: Path = DEFAULT_LOG_DIR,
        batch_frames: int = 128,
        flush_interval_s: float = 0.25,
        fsync_interval_s: float = 1.0,
        stop_timeout_s: float = 5.0,
        transient_incident_limit: int = DEFAULT_TRANSIENT_INCIDENT_LIMIT,
        transient_window_s: float = DEFAULT_TRANSIENT_WINDOW_S,
    ) -> None:
        if isinstance(batch_frames, bool) or not isinstance(batch_frames, int):
            raise CollectorError("batch_frames 必须是整数")
        if batch_frames < 1 or batch_frames > 2048:
            raise CollectorError("batch_frames 必须在1..2048")
        for name, value, maximum in (
            ("flush_interval_s", flush_interval_s, 60.0),
            ("fsync_interval_s", fsync_interval_s, 60.0),
            ("stop_timeout_s", stop_timeout_s, 60.0),
        ):
            if not math.isfinite(value) or value <= 0 or value > maximum:
                raise CollectorError("{} 必须在(0,{}]".format(name, maximum))
        if fsync_interval_s < flush_interval_s:
            raise CollectorError("fsync_interval_s 不能小于 flush_interval_s")
        if set(streams.by_side.keys()) != set(SIDE_NAMES):
            raise CollectorError("订阅必须且只能包含 left/right")

        self.session = session
        self.pairing = pairing
        self.config = config
        self.streams = streams
        self.log_dir = Path(log_dir).resolve()
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.batch_frames = batch_frames
        self.flush_interval_s = float(flush_interval_s)
        self.fsync_interval_s = float(fsync_interval_s)
        self.stop_timeout_s = float(stop_timeout_s)
        self._transient_health = TransientHealthTracker(
            transient_incident_limit,
            transient_window_s,
        )
        self._transient_start = self._transient_health.snapshot()

        self.collector_instance_id = uuid.uuid4().hex
        self.pairing_session_id = uuid.uuid4().hex
        self._bindings = {binding.side: binding for binding in pairing.bindings}
        if set(self._bindings.keys()) != set(SIDE_NAMES):
            raise CollectorError("配对结果必须且只能包含 left/right")

        self._condition = threading.Condition(threading.RLock())
        # START/STOP/shutdown 只能有一个生命周期操作的 owner。
        self._lifecycle_lock = threading.RLock()
        self._state = STATE_PAIRED_IDLE
        self._fault_reason: Optional[str] = None
        self._service_shutdown = threading.Event()
        self._shutdown = threading.Event()
        self._writer_ready = threading.Event()
        self._writer_errors = 0
        self._gap_frames = 0

        self._session_name = "-"
        self._capture_id = ""
        self._session_dir: Optional[Path] = None
        self._partial_path: Optional[Path] = None
        self._final_path: Optional[Path] = None
        self._meta_path: Optional[Path] = None
        self._fp = None
        self._meta: Optional[Dict[str, Any]] = None
        self._start_wall_ns = 0
        self._start_mono_ns = 0
        self._stop_wall_ns = 0
        self._stop_mono_ns = 0
        self._record_seq = 0
        self._records_total = 0
        self._frames_by_side = {"left": 0, "right": 0}
        self._first_recv_wall_ns = {"left": 0, "right": 0}
        self._last_recv_wall_ns = {"left": 0, "right": 0}
        self._last_stream_seq_written = {"left": -1, "right": -1}
        self._last_processed_stream_seq = {"left": -1, "right": -1}
        self._last_processed_mono_ns = {"left": 0, "right": 0}
        self._data_bytes = 0
        self._data_hash = hashlib.sha256()
        self._last_flush_mono = 0.0
        self._last_fsync_mono = 0.0
        self._durable_records = 0
        self._health_start: Dict[str, Any] = {}

        self._writer = threading.Thread(
            target=self._writer_loop,
            name="tactile-raw-writer",
            daemon=True,
        )
        self._writer.start()
        if not self._writer_ready.wait(2.0) or not self._writer.is_alive():
            raise CollectorError("触觉 writer 线程未启动")

    @property
    def state(self) -> str:
        with self._condition:
            return self._state

    @property
    def fault_reason(self) -> Optional[str]:
        with self._condition:
            return self._fault_reason

    def _safe_paths(self, session_name: str) -> Tuple[Path, Path, Path, Path]:
        name = _validate_session_name(session_name)
        session_dir = session_raw_dir(self.log_dir, name)
        log_root = self.log_dir.resolve()
        session_dir.parent.mkdir(parents=True, exist_ok=True)
        try:
            session_dir.parent.resolve().relative_to(log_root)
        except ValueError as exc:
            raise CollectorError("session 父目录逃逸日志根目录: {}".format(
                session_dir.parent)) from exc
        session_dir.mkdir(parents=True, exist_ok=True)
        resolved_dir = session_dir.resolve()
        try:
            resolved_dir.relative_to(log_root)
        except ValueError as exc:
            raise CollectorError("session 目录逃逸日志根目录: {}".format(resolved_dir))
        if resolved_dir == log_root:
            raise CollectorError("session 目录不能等于日志根目录")
        partial = resolved_dir / "tactile.jsonl.partial"
        final = resolved_dir / "tactile.jsonl"
        meta = resolved_dir / "tactile.meta.json"
        for target in (partial, final, meta):
            if target.exists() or target.is_symlink():
                raise CollectorError("拒绝覆盖已有触觉文件: {}".format(target))
        return resolved_dir, partial, final, meta

    def _reader_for_side(self, side: str) -> Any:
        return self.session.reader_for_side(self.pairing, side)

    def _assignment_basis(self, side: str) -> str:
        if self.pairing.method != MANUAL_PAIRING_RESULT_METHOD:
            return "unknown"
        return (
            "activity_verified"
            if side == "left"
            else "remaining_candidate_by_exclusion"
        )

    def _stream_meta(self, side: str) -> Dict[str, Any]:
        binding = self._bindings[side]
        reader = self._reader_for_side(side)
        grid = reader.grid
        if grid is None:
            raise CollectorError("{} grid 不存在".format(side))
        profile = self.config.profiles[binding.physical_profile]
        candidate = reader.candidate
        return {
            "stream_id": binding.stream_id,
            "candidate_label": binding.candidate_label,
            "configured_expected_manus_glove_id": str(binding.manus_glove_id),
            "manus_identity_status": "configured_expectation_only",
            "assignment_basis": self._assignment_basis(side),
            "observed_transport": {
                "port": binding.port_at_pairing,
                "usb_vid_pid": candidate.usb_id,
                "usb_serial": candidate.serial_number,
                "usb_location": candidate.location,
                "description": candidate.description,
                "identity": False,
            },
            "protocol": {
                "address": int(self.session.address),
                "function": int(FUNC_FSR),
                "payload_bytes": int(grid.active_count * 2),
                "requested_rate_hz": float(self.session.rate_hz),
            },
            "wire_layout": {
                "rows": int(grid.rows),
                "cols": int(grid.cols),
                "active_count": int(grid.active_count),
                "order": "row-major over set bits in cellmap",
                "wire_payload_encoding": "signed_int16_le",
                "json_value_type": "integer",
                "cellmap_hex": grid_cellmap_bytes(grid).hex(),
                "cellmap_sha256": grid_cellmap_sha256(grid),
            },
            "physical_interpretation": {
                "profile": binding.physical_profile,
                "kind": str(profile["kind"]),
                "mask_verified": bool(profile["mask_verified"]),
                "post_decode_invalid_cells": [
                    [int(cell[0]), int(cell[1])]
                    for cell in profile["post_decode_invalid_cells"]
                ],
                "finger_groups_verified": True,
                "finger_groups_status": "hardware_and_captured_pressure_verified",
                "finger_mapping_schema": TACTILE_LAYOUT_SCHEMA,
                "finger_mapping_source": TACTILE_LAYOUT_SOURCE,
                "finger_mapping": finger_region_metadata(),
                "dead_channels_verified": bool(profile["mask_verified"]),
                "intra_finger_orientation": "physical (4,8): across_finger x tip_to_base",
                "applied_to_wire_values": False,
            },
        }

    def _pairing_evidence(self) -> Sequence[Dict[str, Any]]:
        return [
            {
                "requested_side": item.requested_side,
                "candidate_label": item.candidate_label,
                "observed_port": item.port,
                "baseline_frames": int(item.baseline_frames),
                "press_frames": int(item.press_frames),
                "baseline_score": float(item.baseline_score),
                "raw_press_score": float(item.raw_press_score),
                "score": float(item.score),
                "active_frame_count": int(item.active_frame_count),
                "peak_active_cells": int(item.peak_active_cells),
            }
            for item in self.pairing.evidence
        ]

    def _scoring_meta(self) -> Dict[str, Any]:
        scoring = getattr(self.pairing, "scoring", None)
        if scoring is None:
            return {
                "recorded": False,
                "algorithm": "median_mad_topk_v1",
                "parameters": None,
            }
        return {
            "recorded": True,
            "algorithm": "median_mad_topk_v1",
            "parameters": {
                name: getattr(scoring, name)
                for name in scoring.__dataclass_fields__
            },
        }

    def _new_meta(self, session_name: str) -> Dict[str, Any]:
        return {
            "schema": META_SCHEMA,
            "session": session_name,
            "capture_id": self._capture_id,
            "collector_instance_id": self.collector_instance_id,
            "state": "recording",
            "complete": False,
            "data_file": "tactile.jsonl.partial",
            "final_data_file": "tactile.jsonl",
            "commit_protocol": {
                "valid_capture_marker": "complete=true and capture_valid=true",
                "partial_or_faulted_is_not_training_data": True,
            },
            "frame_schema": FRAME_SCHEMA,
            "requested_sides": ["left", "right"],
            "requested_rate_hz_per_side": float(self.session.rate_hz),
            "start_wall_ns": self._start_wall_ns,
            "start_mono_ns": self._start_mono_ns,
            "capture_scope": "tactile_only",
            "tactile_raw_capture_ready_at_start": True,
            "capture_valid": False,
            "formal_multimodal_recording_ready": False,
            "manus_live_verified": False,
            "manus_continuous_lease": False,
            "values_semantics": "wire_active_order_unmasked",
            "subscription": {
                "start_wall_ns": int(self.streams.start_wall_ns),
                "start_mono_ns": int(self.streams.start_mono_ns),
                "continuity_policy": "per-side stream_seq must be contiguous",
                "gap_policy": "latch_fault_and_keep_partial",
            },
            "transient_health_policy": {
                "fields": list(TRANSIENT_HEALTH_FIELDS),
                "action": "warn_and_continue_until_window_threshold",
                "incident_limit": int(self._transient_health.incident_limit),
                "window_s": float(self._transient_health.window_s),
                "capture_valid_with_warnings": True,
            },
            "clocks": {
                "align_key": "recv_qpc_ns preferred; recv_wall_ns legacy fallback",
                "recv_wall_ns": "capture-host Python time.time_ns() after accepted full frame",
                "recv_mono_ns": "capture-process monotonic clock",
                "recv_qpc_ns": "capture-host QueryPerformanceCounter/perf_counter clock",
                "request_mono_ns": "capture-process monotonic clock immediately before request",
                "sensor_clock_present": False,
            },
            "pairing": {
                "pairing_session_id": self.pairing_session_id,
                "method": self.pairing.method,
                "paired_wall_ns": int(self.pairing.paired_wall_ns),
                "config_sha256": self.pairing.config_sha256,
                "manus_source": self.pairing.manus_source,
                "manus_identity_status": "configured_expectation_only",
                "manus_live_verified": False,
                "transport_is_identity": False,
                "assignment_policy": {
                    "positive_identified_side": "left",
                    "left": "activity_verified",
                    "right": "remaining_candidate_by_exclusion",
                    "candidate_count": 2,
                },
                "valid_until": "collector_process_and_original_open_serial_handles_only",
                "evidence": self._pairing_evidence(),
                "activity_scoring": self._scoring_meta(),
            },
            "streams": {
                "left": self._stream_meta("left"),
                "right": self._stream_meta("right"),
            },
            "writer": {
                "encoding": "utf-8",
                "line_ending": "LF",
                "batch_frames": self.batch_frames,
                "flush_interval_ms": int(round(self.flush_interval_s * 1000.0)),
                "fsync_interval_ms": int(round(self.fsync_interval_s * 1000.0)),
                "exclusive_create": True,
            },
            "health_counter_semantics": {
                "buffer_drops": "reader ring overwrites; not a capture loss unless subscription_gap_frames is nonzero",
                "missed_deadlines": "host request schedule slips; accepted frames retain measured timestamps",
                "subscription_gap_frames": "confirmed frames missed by this collector; must be zero for a complete capture",
                "timeouts": "transient warning; repeated incidents within the configured window latch a fault",
                "parser_discarded": "transient resync byte count; recorded with warnings and escalated by incident threshold",
            },
            "health_at_start": dict(self._health_start),
            "summary": None,
        }

    def _current_health(self) -> Dict[str, Any]:
        return {
            side: _health_dict(self._reader_for_side(side).health_snapshot())
            for side in SIDE_NAMES
        }

    def _health_delta(self, end: Mapping[str, Any]) -> Dict[str, Any]:
        result: Dict[str, Any] = {}
        for side in SIDE_NAMES:
            start_values = self._health_start.get(side, {})
            end_values = end.get(side, {})
            result[side] = {
                name: max(0, int(end_values.get(name, 0)) - int(start_values.get(name, 0)))
                for name in end_values
            }
        return result

    def _summary_locked(self, complete: bool, reason: str) -> Dict[str, Any]:
        try:
            health_end = self._current_health()
        except Exception as exc:  # noqa: BLE001 - fault metadata must still be writable
            health_end = {"error": "{}: {}".format(type(exc).__name__, exc)}
        health_delta = (
            self._health_delta(health_end)
            if all(side in health_end for side in SIDE_NAMES)
            else {}
        )
        actual_rate_hz = {}
        for side in SIDE_NAMES:
            count = int(self._frames_by_side[side])
            first = int(self._first_recv_wall_ns[side])
            last = int(self._last_recv_wall_ns[side])
            actual_rate_hz[side] = (
                round((count - 1) * 1e9 / (last - first), 6)
                if count > 1 and last > first else 0.0
            )
        transient_end = self._transient_health.snapshot()
        transient_by_side = {}
        transient_count = 0
        for side in SIDE_NAMES:
            start_incidents = int(
                self._transient_start["incident_count_by_side"].get(side, 0)
            )
            end_incidents = int(
                transient_end["incident_count_by_side"].get(side, 0)
            )
            counters = {}
            for name in TRANSIENT_HEALTH_FIELDS:
                start_value = int(
                    self._transient_start["counters_by_side"].get(side, {}).get(name, 0)
                )
                end_value = int(
                    transient_end["counters_by_side"].get(side, {}).get(name, 0)
                )
                counters[name] = max(0, end_value - start_value)
            incidents = max(0, end_incidents - start_incidents)
            transient_count += incidents
            transient_by_side[side] = {
                "incidents": incidents,
                **counters,
            }
        transient_warnings = {
            "incident_count": int(transient_count),
            "by_side": transient_by_side,
            "last_event": (
                transient_end["last_event"] if transient_count else None
            ),
        }
        return {
            "complete": bool(complete),
            "reason": str(reason),
            "quality_status": "warning" if transient_count else "ok",
            "transient_health_warnings": transient_warnings,
            "stop_wall_ns": int(self._stop_wall_ns or time.time_ns()),
            "stop_mono_ns": int(self._stop_mono_ns or time.monotonic_ns()),
            "records_total": int(self._records_total),
            "frames_by_side": dict(self._frames_by_side),
            "requested_rate_hz_per_side": float(self.session.rate_hz),
            "actual_rate_hz_by_side": actual_rate_hz,
            "actual_recorded_sides": [
                side for side in SIDE_NAMES if self._frames_by_side[side] > 0
            ],
            "first_recv_wall_ns": dict(self._first_recv_wall_ns),
            "last_recv_wall_ns": dict(self._last_recv_wall_ns),
            "last_stream_seq_written": dict(self._last_stream_seq_written),
            "data_bytes": int(self._data_bytes),
            "data_sha256": self._data_hash.hexdigest(),
            "durable_records": int(self._durable_records),
            "subscription_gap_frames": int(self._gap_frames),
            "writer_errors": int(self._writer_errors),
            "health_at_end": health_end,
            "health_delta": health_delta,
        }

    def _flush_locked(self, durable: bool) -> None:
        if self._fp is None:
            return
        self._fp.flush()
        self._last_flush_mono = time.monotonic()
        if durable:
            os.fsync(self._fp.fileno())
            self._last_fsync_mono = self._last_flush_mono
            self._durable_records = self._records_total

    def _write_frame_locked(self, side: str, frame: TimedFsrFrame) -> None:
        values = frame.wire_values
        if len(values) != 369:
            raise CollectorError("{} frame 通道数不是369: {}".format(side, len(values)))
        if any(
            isinstance(value, bool)
            or not isinstance(value, int)
            or value < -32768
            or value > 32767
            for value in values
        ):
            raise CollectorError("{} frame 含非 signed-int16 值".format(side))
        if frame.stream_seq <= self._last_stream_seq_written[side]:
            raise CollectorError("{} stream_seq 未严格递增".format(side))
        if frame.request_mono_ns and frame.request_mono_ns > frame.recv_mono_ns:
            raise CollectorError("{} request_mono_ns 晚于 recv_mono_ns".format(side))
        record = {
            "schema": FRAME_SCHEMA,
            "type": "tactile_frame",
            "record_seq": self._record_seq,
            "side": side,
            "stream_id": self._bindings[side].stream_id,
            "stream_seq": int(frame.stream_seq),
            "request_mono_ns": int(frame.request_mono_ns),
            "recv_mono_ns": int(frame.recv_mono_ns),
            "recv_qpc_ns": int(frame.recv_qpc_ns),
            "recv_wall_ns": int(frame.recv_wall_ns),
            "wire_values": list(values),
        }
        raw = _json_bytes(record, pretty=False)
        if self._fp is None:
            raise CollectorError("数据文件未打开")
        written = self._fp.write(raw)
        if written != len(raw):
            raise CollectorError("触觉 JSONL 短写: {} / {} bytes".format(written, len(raw)))
        self._data_hash.update(raw)
        self._data_bytes += len(raw)
        self._record_seq += 1
        self._records_total += 1
        self._frames_by_side[side] += 1
        if not self._first_recv_wall_ns[side]:
            self._first_recv_wall_ns[side] = int(frame.recv_wall_ns)
        self._last_recv_wall_ns[side] = int(frame.recv_wall_ns)
        self._last_stream_seq_written[side] = int(frame.stream_seq)

        now = time.monotonic()
        if (
            self._records_total % 64 == 0
            or now - self._last_flush_mono >= self.flush_interval_s
        ):
            self._flush_locked(durable=False)
        if now - self._last_fsync_mono >= self.fsync_interval_s:
            self._flush_locked(durable=True)

    def _process_frame(self, side: str, frame: TimedFsrFrame) -> None:
        with self._condition:
            previous = self._last_processed_stream_seq[side]
            if frame.stream_seq <= previous:
                raise CollectorError(
                    "{} subscription stream_seq 未严格递增: {} <= {}".format(
                        side, frame.stream_seq, previous
                    )
                )
            self._last_processed_stream_seq[side] = int(frame.stream_seq)
            self._last_processed_mono_ns[side] = int(frame.recv_mono_ns)
            should_write = (
                self._state in (STATE_RECORDING, STATE_STOPPING)
                and frame.recv_mono_ns >= self._start_mono_ns
                and (not self._stop_mono_ns or frame.recv_mono_ns < self._stop_mono_ns)
            )
            if should_write:
                self._write_frame_locked(side, frame)
            self._condition.notify_all()

    def _latch_fault(self, reason: str, gap_frames: int = 0) -> None:
        with self._condition:
            if self._state in (STATE_FAULT, STATE_SHUTDOWN):
                return
            capture_was_active = self._state in (STATE_RECORDING, STATE_STOPPING)
            self._fault_reason = str(reason)
            self._gap_frames += max(0, int(gap_frames))
            self._writer_errors += 1
            self._stop_wall_ns = self._stop_wall_ns or time.time_ns()
            self._stop_mono_ns = self._stop_mono_ns or time.monotonic_ns()
            if self._fp is not None:
                try:
                    self._flush_locked(durable=True)
                except Exception as exc:  # noqa: BLE001
                    self._fault_reason += "; flush={}: {}".format(type(exc).__name__, exc)
                try:
                    self._fp.close()
                except Exception as exc:  # noqa: BLE001
                    self._fault_reason += "; close={}: {}".format(type(exc).__name__, exc)
                self._fp = None
            if capture_was_active and self._meta is not None and self._meta_path is not None:
                try:
                    failed_meta = dict(self._meta)
                    failed_meta["state"] = "faulted"
                    failed_meta["complete"] = False
                    failed_meta["capture_valid"] = False
                    if self._final_path is not None and self._final_path.exists():
                        failed_meta["data_file"] = self._final_path.name
                    else:
                        failed_meta["data_file"] = (
                            self._partial_path.name
                            if self._partial_path is not None
                            else None
                        )
                    failed_meta["stop_wall_ns"] = self._stop_wall_ns
                    failed_meta["stop_mono_ns"] = self._stop_mono_ns
                    failed_meta["fault"] = self._fault_reason
                    failed_meta["summary"] = self._summary_locked(False, self._fault_reason)
                    _commit_json(self._meta_path, failed_meta)
                    self._meta = failed_meta
                except Exception as exc:  # noqa: BLE001
                    self._fault_reason += "; meta={}: {}".format(type(exc).__name__, exc)
            self._state = STATE_FAULT
            self._condition.notify_all()

    def _writer_loop(self) -> None:
        self._writer_ready.set()
        while not self._shutdown.is_set():
            made_progress = False
            for side in SIDE_NAMES:
                if self._shutdown.is_set():
                    break
                subscription = self.streams.by_side[side]
                try:
                    batch = subscription.read_batch(
                        max_frames=self.batch_frames,
                        timeout_s=0.1,
                    )
                    warning = self._transient_health.observe(side, batch.health_delta)
                    if warning is not None:
                        print(
                            "[tactile] transient warning side={} timeouts={} "
                            "parser_discarded={} active_window={}/{}".format(
                                side,
                                warning["timeouts"],
                                warning["parser_discarded"],
                                warning["active_window_incidents"],
                                self._transient_health.incident_limit,
                            ),
                            flush=True,
                        )
                        if warning["fault_threshold_reached"]:
                            self._latch_fault(
                                "{} transient health incidents {}/{} within {:.1f}s; "
                                "timeouts={} parser_discarded={}".format(
                                    side,
                                    warning["active_window_incidents"],
                                    self._transient_health.incident_limit,
                                    self._transient_health.window_s,
                                    warning["timeouts"],
                                    warning["parser_discarded"],
                                )
                            )
                            return
                    frames = batch.frames
                    if frames:
                        made_progress = True
                    for frame in frames:
                        self._process_frame(side, frame)
                except FrameGapError as exc:
                    self._latch_fault(str(exc), getattr(exc, "dropped_frames", 0))
                    return
                except PairingError as exc:
                    if not self._shutdown.is_set():
                        self._latch_fault(str(exc))
                    return
                except Exception as exc:  # noqa: BLE001
                    if not self._shutdown.is_set():
                        self._latch_fault("{}: {}".format(type(exc).__name__, exc))
                    return
            if not made_progress:
                self._shutdown.wait(0.01)

    def start_session(self, session_name: str) -> str:
        with self._lifecycle_lock:
            return self._start_session_locked(session_name)

    def _start_session_locked(self, session_name: str) -> str:
        try:
            name = _validate_session_name(session_name)
        except CollectorError as exc:
            return "ERR {}".format(_fault_token(str(exc)))
        with self._condition:
            if self._state == STATE_FAULT:
                return "ERR fault_latched reason={}".format(_fault_token(self._fault_reason or "fault"))
            if self._state != STATE_PAIRED_IDLE:
                return "ERR state={}".format(self._state)
            if not self._writer.is_alive():
                self._latch_fault("writer_thread_not_alive")
                return "ERR fault_latched reason=writer_thread_not_alive"

        try:
            require_raw_capture_ready(self.session, self.pairing, self.config)
            session_dir, partial, final, meta_path = self._safe_paths(name)
        except (PairingError, CollectorError, OSError) as exc:
            return "ERR {}".format(_fault_token(str(exc)))

        opened = None
        try:
            opened = partial.open("xb", buffering=1024 * 1024)
            with self._condition:
                if self._state != STATE_PAIRED_IDLE:
                    raise CollectorError("START 期间状态改变为 {}".format(self._state))
                self._session_name = name
                self._capture_id = uuid.uuid4().hex
                self._session_dir = session_dir
                self._partial_path = partial
                self._final_path = final
                self._meta_path = meta_path
                self._fp = opened
                opened = None
                self._start_wall_ns = time.time_ns()
                self._start_mono_ns = time.monotonic_ns()
                self._stop_wall_ns = 0
                self._stop_mono_ns = 0
                self._record_seq = 0
                self._records_total = 0
                self._frames_by_side = {"left": 0, "right": 0}
                self._first_recv_wall_ns = {"left": 0, "right": 0}
                self._last_recv_wall_ns = {"left": 0, "right": 0}
                self._last_stream_seq_written = {"left": -1, "right": -1}
                self._data_bytes = 0
                self._data_hash = hashlib.sha256()
                self._last_flush_mono = time.monotonic()
                self._last_fsync_mono = self._last_flush_mono
                self._durable_records = 0
                self._health_start = self._current_health()
                self._transient_start = self._transient_health.snapshot()
                self._meta = self._new_meta(name)
                # 状态先在同一把 Condition 锁内切换；若 meta 提交失败，异常路径
                # 会锁存故障并保留/补写 faulted meta，不会产生孤儿 recording meta。
                self._state = STATE_RECORDING
                _commit_json(meta_path, self._meta)
                self._condition.notify_all()
            return (
                "OK tactile_raw_recording capture_scope=tactile_only pipeline_ready=0 "
                "session={} file={} meta={}".format(name, final, meta_path)
            )
        except Exception as exc:  # noqa: BLE001
            if opened is not None:
                try:
                    opened.close()
                except Exception:
                    pass
            with self._condition:
                state_after_failure = self._state
            if state_after_failure == STATE_RECORDING:
                self._latch_fault(
                    "START commit {}: {}".format(type(exc).__name__, exc)
                )
                return "ERR fault_latched reason={}".format(
                    _fault_token(self._fault_reason or "start_commit_failed")
                )
            if state_after_failure == STATE_FAULT:
                return "ERR fault_latched reason={}".format(
                    _fault_token(self._fault_reason or "start_failed")
                )
            with self._condition:
                owned_fp = self._fp
                self._fp = None
                if owned_fp is not None:
                    try:
                        owned_fp.close()
                    except Exception:
                        pass
                self._meta = None
                self._meta_path = None
                self._partial_path = None
                self._final_path = None
                self._session_dir = None
            try:
                if partial.exists() and partial.stat().st_size == 0:
                    partial.unlink()
            except OSError:
                pass
            try:
                if session_dir.exists() and not any(session_dir.iterdir()):
                    session_dir.rmdir()
            except OSError:
                pass
            return "ERR {}".format(_fault_token("{}: {}".format(type(exc).__name__, exc)))

    def _finalize_complete_locked(self) -> None:
        if self._fp is None or self._partial_path is None or self._final_path is None:
            raise CollectorError("STOP 时数据文件状态不完整")
        self._flush_locked(durable=True)
        self._fp.close()
        self._fp = None
        os.replace(str(self._partial_path), str(self._final_path))
        _fsync_directory(self._final_path.parent)
        if self._meta is None or self._meta_path is None:
            raise CollectorError("STOP 时 meta 状态不完整")
        completed = dict(self._meta)
        completed["state"] = "complete"
        completed["complete"] = True
        completed["capture_valid"] = True
        completed["data_file"] = self._final_path.name
        completed["stop_wall_ns"] = self._stop_wall_ns
        completed["stop_mono_ns"] = self._stop_mono_ns
        completed["summary"] = self._summary_locked(True, "clean_stop")
        _commit_json(self._meta_path, completed)
        self._meta = completed

    def stop_session(self) -> str:
        with self._lifecycle_lock:
            return self._stop_session_locked()

    def _stop_session_locked(self) -> str:
        with self._condition:
            if self._state == STATE_FAULT:
                return "ERR fault_latched reason={}".format(_fault_token(self._fault_reason or "fault"))
            if self._state != STATE_RECORDING:
                return "ERR not_recording state={}".format(self._state)
            self._state = STATE_STOPPING
            self._stop_wall_ns = time.time_ns()
            self._stop_mono_ns = time.monotonic_ns()
            deadline = time.monotonic() + self.stop_timeout_s
            while self._state == STATE_STOPPING:
                if all(
                    self._last_processed_mono_ns[side] >= self._stop_mono_ns
                    for side in SIDE_NAMES
                ):
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    self._latch_fault("STOP drain timeout")
                    return "ERR fault_latched reason=stop_drain_timeout"
                self._condition.wait(min(0.1, remaining))
            if self._state == STATE_FAULT:
                return "ERR fault_latched reason={}".format(_fault_token(self._fault_reason or "fault"))
            empty_sides = [
                side for side in SIDE_NAMES if self._frames_by_side[side] == 0
            ]
            if empty_sides:
                self._latch_fault(
                    "no recorded frames for side(s): {}".format(",".join(empty_sides))
                )
                return "ERR fault_latched reason=no_recorded_frames"
            try:
                self._finalize_complete_locked()
            except Exception as exc:  # noqa: BLE001
                self._latch_fault("STOP finalize {}: {}".format(type(exc).__name__, exc))
                return "ERR fault_latched reason={}".format(_fault_token(self._fault_reason or "fault"))
            frames = dict(self._frames_by_side)
            data_bytes = self._data_bytes
            digest = self._data_hash.hexdigest()
            session_name = self._session_name
            warning_count = int(
                (self._meta or {}).get("summary", {})
                .get("transient_health_warnings", {})
                .get("incident_count", 0)
            )
            # 已完成会话的 meta 已原子封存；idle 期间后续设备故障不得改写它。
            self._meta = None
            self._meta_path = None
            self._partial_path = None
            self._final_path = None
            self._session_dir = None
            self._state = STATE_PAIRED_IDLE
            self._condition.notify_all()
            return (
                "OK tactile_raw_stopped capture_scope=tactile_only capture_valid=1 "
                "pipeline_ready=0 session={} frames=l={},r={} bytes={} sha256={} warnings={}".format(
                    session_name,
                    frames["left"],
                    frames["right"],
                    data_bytes,
                    digest,
                    warning_count,
                )
            )

    def status(self) -> str:
        with self._condition:
            state = self._state
            ages = {}
            reader_error = 0
            for side in SIDE_NAMES:
                try:
                    reader = self._reader_for_side(side)
                    ages[side] = int(reader.latest_frame_age_ms())
                    if reader.error or not reader.is_alive():
                        reader_error += 1
                except Exception:
                    ages[side] = -1
                    reader_error += 1
            raw_stream_valid = int(
                state in (STATE_PAIRED_IDLE, STATE_RECORDING, STATE_STOPPING)
                and reader_error == 0
            )
            pairing_valid = int(
                state in (STATE_PAIRED_IDLE, STATE_RECORDING, STATE_STOPPING)
                and reader_error == 0
            )
            start_ready = int(state == STATE_PAIRED_IDLE and reader_error == 0)
            mask_verified = int(
                all(binding.mask_verified for binding in self._bindings.values())
            )
            fsync_age_ms = (
                int(max(0.0, (time.monotonic() - self._last_fsync_mono) * 1000.0))
                if self._last_fsync_mono
                else -1
            )
            fault = _fault_token(self._fault_reason or "none")
            transient = self._transient_health.snapshot()
            warning_left = int(transient["incident_count_by_side"]["left"])
            warning_right = int(transient["incident_count_by_side"]["right"])
            warning_active = sum(
                int(value)
                for value in transient["active_window_incidents_by_side"].values()
            )
            last_warning = transient["last_event"]
            last_warning_token = _fault_token(
                "none" if last_warning is None else "{}_timeouts_{}_parser_discarded_{}".format(
                    last_warning["side"],
                    last_warning["timeouts"],
                    last_warning["parser_discarded"],
                )
            )
            return (
                "OK state={} session={} paired={} raw_stream_valid={} start_ready={} "
                "pipeline_ready=0 "
                "capture_scope=tactile_only frames=l={},r={} age_ms=l={},r={} "
                "durable_records={} fsync_age_ms={} gaps={} errors={} warnings={} "
                "warning_l={} warning_r={} warning_active={} warning_last={} "
                "manus_live_verified=0 mask_verified={} fault={}".format(
                    state,
                    self._session_name,
                    pairing_valid,
                    raw_stream_valid,
                    start_ready,
                    self._frames_by_side["left"],
                    self._frames_by_side["right"],
                    ages["left"],
                    ages["right"],
                    self._durable_records,
                    fsync_age_ms,
                    self._gap_frames,
                    self._writer_errors + reader_error,
                    warning_left + warning_right,
                    warning_left,
                    warning_right,
                    warning_active,
                    last_warning_token,
                    mask_verified,
                    fault,
                )
            )

    def handle_command(self, command: str, argument: str) -> str:
        cmd = str(command).upper()
        arg = str(argument or "")
        if cmd == "PING":
            return "PONG"
        if cmd == "STATUS":
            return self.status()
        if cmd == "START":
            if not arg.strip() or len(arg.split()) != 1:
                return "ERR START_requires_one_safe_session_name"
            return self.start_session(arg.strip())
        if cmd == "STOP":
            if arg.strip():
                return "ERR STOP_takes_no_arguments"
            return self.stop_session()
        if cmd == "SHUTDOWN":
            if arg.strip():
                return "ERR SHUTDOWN_takes_no_arguments"
            if self.state == STATE_RECORDING:
                return "ERR recording_active"
            self.request_service_shutdown()
            return "OK shutting_down"
        return "ERR unknown_command_{}".format(_fault_token(cmd))

    def request_service_shutdown(self) -> None:
        """通知 service 主循环退出；实际封存和资源释放由上下文退出路径完成。"""
        self._service_shutdown.set()

    def run_service(self, control_port: int) -> None:
        control = ControlServer(control_port, self.handle_command)
        control.start()
        control.wait_ready(5.0)
        print(
            "[tactile] service 控制端口 {}; state={}；仅触觉原始采集，pipeline_ready=0".format(
                control_port, self._state
            ),
            flush=True,
        )
        try:
            while not self._service_shutdown.wait(0.25):
                if self.state == STATE_FAULT:
                    time.sleep(0.25)
        finally:
            control.stop()
            control.join(self.stop_timeout_s + 2.0)
            if control.is_alive():
                raise CollectorError("control_server_handler_not_stopped")

    def shutdown(self) -> Tuple[str, ...]:
        failures = []
        # 先让 service 控制循环退出，但保持 writer 工作，确保活动录制能 clean STOP。
        self._service_shutdown.set()
        lifecycle_acquired = self._lifecycle_lock.acquire(
            timeout=self.stop_timeout_s + 3.0
        )
        if not lifecycle_acquired:
            failures.append("lifecycle_operation_not_stopped")
            # 另一个 START/STOP 仍拥有生命周期；不能在 owner 背后拆订阅或 writer。
            # 调用方可在该操作结束后重试 shutdown。
            return tuple(failures)
        try:
            if lifecycle_acquired:
                condition_acquired = self._condition.acquire(timeout=1.0)
                if condition_acquired:
                    try:
                        state = self._state
                        # 保持 Condition 到 STOP 切换为 STOPPING；RLock/Condition.wait
                        # 会正确释放递归层级，不给 writer 插入一次长 I/O 的竞态窗口。
                        if state == STATE_RECORDING:
                            reply = self._stop_session_locked()
                            if not reply.startswith("OK"):
                                failures.append(reply)
                        elif state == STATE_STOPPING:
                            # lifecycle_lock 由 STOP owner 持有；正常不应到达这里。
                            failures.append("unexpected_stopping_without_owner")
                    finally:
                        self._condition.release()
                else:
                    failures.append("collector_state_lock_timeout")

            self._shutdown.set()
            close = getattr(self.streams, "close", None)
            if callable(close):
                try:
                    close()
                except Exception as exc:  # noqa: BLE001
                    failures.append("subscriptions={}: {}".format(type(exc).__name__, exc))
            else:
                for subscription in self.streams.by_side.values():
                    try:
                        subscription.close()
                    except Exception as exc:  # noqa: BLE001
                        failures.append("subscription={}: {}".format(type(exc).__name__, exc))

            self._writer.join(3.0)
            if self._writer.is_alive():
                failures.append("writer_thread_not_stopped")

            if lifecycle_acquired:
                # 即使 writer 未按时退出，也只做有界锁获取；绝不在 shutdown 无界卡住。
                condition_acquired = self._condition.acquire(timeout=1.0)
                if not condition_acquired:
                    failures.append("collector_finalize_lock_timeout")
                else:
                    try:
                        if self._state in (STATE_RECORDING, STATE_STOPPING):
                            self._latch_fault("shutdown_without_clean_stop")
                        if self._fp is not None:
                            try:
                                self._flush_locked(durable=True)
                                self._fp.close()
                            except Exception as exc:  # noqa: BLE001
                                failures.append(
                                    "file_close={}: {}".format(type(exc).__name__, exc)
                                )
                            self._fp = None
                        self._state = STATE_SHUTDOWN
                        self._condition.notify_all()
                    finally:
                        self._condition.release()
        finally:
            if lifecycle_acquired:
                self._lifecycle_lock.release()
        return tuple(failures)

    def __enter__(self) -> "TactileCollector":
        return self

    def __exit__(self, exc_type: Any, _exc: Any, _traceback: Any) -> None:
        failures = self.shutdown()
        if failures and exc_type is None:
            raise CollectorError("; ".join(failures))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="双手触觉原始369通道采集器")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH))
    parser.add_argument("--log-dir", default=str(DEFAULT_LOG_DIR))
    parser.add_argument("--port", action="append", default=[], help="显式候选串口，可重复两次")
    parser.add_argument("--address", type=parse_hex_byte, default=DEFAULT_ADDRESS)
    parser.add_argument("--rate-hz", type=float, default=60.0)
    parser.add_argument("--response-timeout-ms", type=int, default=200)
    parser.add_argument("--baseline-seconds", type=float, default=2.0)
    parser.add_argument("--press-seconds", type=float, default=4.0)
    parser.add_argument("--batch-frames", type=int, default=128)
    parser.add_argument("--flush-interval", type=float, default=0.25)
    parser.add_argument("--fsync-interval", type=float, default=1.0)
    parser.add_argument("--stop-timeout", type=float, default=5.0)
    parser.add_argument(
        "--transient-incident-limit",
        type=int,
        default=DEFAULT_TRANSIENT_INCIDENT_LIMIT,
        help="短窗口内 timeout/parser_discarded 告警达到此次数才锁存",
    )
    parser.add_argument(
        "--transient-window-seconds",
        type=float,
        default=DEFAULT_TRANSIENT_WINDOW_S,
        help="瞬时告警升级为锁存故障的滚动窗口秒数",
    )
    parser.add_argument("--service", action="store_true")
    parser.add_argument("--control-port", type=int, default=TACTILE_CONTROL_PORT)
    parser.add_argument("--session", help="非service模式会话名；默认时间戳")
    parser.add_argument("--duration", type=float, default=0.0, help="非service模式录制秒数；0表示按Enter停止")
    return parser


def run(args: argparse.Namespace) -> int:
    if not math.isfinite(args.duration) or args.duration < 0 or args.duration > 86400:
        raise CollectorError("--duration 必须在0..86400秒")
    if args.control_port < 1 or args.control_port > 65535:
        raise CollectorError("--control-port 必须在1..65535")
    config = load_pairing_config(Path(args.config))
    manus = configured_manus(config)
    serial_module, list_ports = _load_pyserial()
    candidates, all_ports = discover_candidates(list_ports, args.port)
    if len(candidates) != 2:
        print_port_table(all_ports, "当前串口：")
        raise CollectorError("触觉候选必须恰好为2，实际 {}".format(len(candidates)))

    print("触觉候选仅作为本进程会话对象，COM/USB位置不是左右身份：", flush=True)
    print_port_table(candidates, "准备打开：")
    with PairingSession.open(
        candidates,
        serial_module,
        address=args.address,
        rate_hz=args.rate_hz,
        response_timeout_ms=args.response_timeout_ms,
    ) as session:
        pairing = pair_by_press(
            session,
            config,
            manus,
            _pairing_prompt,
            baseline_seconds=args.baseline_seconds,
            press_seconds=args.press_seconds,
        )
        require_raw_capture_ready(session, pairing, config)
        streams = session.subscribe_paired(pairing)
        collector_context = TactileCollector(
            session,
            pairing,
            config,
            streams,
            log_dir=Path(args.log_dir),
            batch_frames=args.batch_frames,
            flush_interval_s=args.flush_interval,
            fsync_interval_s=args.fsync_interval,
            stop_timeout_s=args.stop_timeout,
            transient_incident_limit=args.transient_incident_limit,
            transient_window_s=args.transient_window_seconds,
        )
        previous_handlers = {}
        with ExitStack() as stack:
            if args.service:
                def request_shutdown(_signal_number: int, _frame: Any) -> None:
                    collector_context.request_service_shutdown()

                for signal_number in (signal.SIGINT, signal.SIGTERM):
                    previous_handlers[signal_number] = signal.getsignal(signal_number)
                    signal.signal(signal_number, request_shutdown)

                def restore_signal_handlers() -> None:
                    for signal_number, previous in previous_handlers.items():
                        signal.signal(signal_number, previous)

                # ExitStack 按后进先出执行：先完整 shutdown collector，再恢复默认信号处理。
                stack.callback(restore_signal_handlers)

            collector = stack.enter_context(collector_context)
            print(
                "[tactile] 左手按压识别、右手排除绑定通过；raw_stream_valid=1, "
                "start_ready=1, pipeline_ready=0",
                flush=True,
            )
            if args.service:
                collector.run_service(args.control_port)
                return 0

            session_name = args.session or datetime.now().strftime("%Y%m%d_%H%M%S")
            reply = collector.start_session(session_name)
            print("[tactile] " + reply, flush=True)
            if not reply.startswith("OK"):
                return 2
            if args.duration > 0:
                deadline = time.monotonic() + args.duration
                while time.monotonic() < deadline:
                    if collector.state == STATE_FAULT:
                        raise CollectorError(collector.fault_reason or "collector fault")
                    time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))
            else:
                try:
                    input("正在录制全部369个原始值；按 Enter 停止: ")
                except EOFError:
                    pass
            reply = collector.stop_session()
            print("[tactile] " + reply, flush=True)
            return 0 if reply.startswith("OK") else 3


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return run(args)
    except (CollectorError, PairingRejected, PairingError, OSError) as exc:
        print("[tactile] {}".format(exc), file=sys.stderr, flush=True)
        return 2
    except KeyboardInterrupt:
        print("\n[tactile] 用户中止", file=sys.stderr, flush=True)
        return 130


if __name__ == "__main__":
    sys.exit(main())
