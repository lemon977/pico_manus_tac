#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""触觉设备与 MANUS 左右手的会话级手动配对。

触觉控制器目前没有不可变 UID，CH340 也没有 USB serial。因此本模块永远不会把
COM/tty、USB location 或 candidate 顺序当作永久身份。默认流程要求每次进程启动后：

1. 同时打开两只触觉设备并验证协议、24x16/369 格点和只读采样；
2. 采集双手松开的基线，再提示用户只按左手任意触觉区域；
3. 把活动明确的一路绑定为左手，把剩余唯一一路按排除法绑定为右手；
4. 把本次进程的临时 tactile stream 绑定到配置中的 MANUS uint32 glove_id。

配对成功后 ``PairingSession`` 仍持有原串口，后续采集器应复用同一个 session，避免
关闭重开后端口顺序变化。standalone CLI 只用于检查/配对验证，退出时会释放串口，
不会落盘触觉原始帧，也不会把临时端口映射写回永久配置。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import queue
import re
import signal
import statistics
import subprocess
import sys
import tempfile
import threading
import time
import uuid
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any, Callable, Deque, Dict, List, Mapping, Optional, Sequence, Set, Tuple

from tactile_probe import (
    CH340_PID,
    CH340_VID,
    EXPECTED_ACTIVE,
    EXPECTED_COLS,
    EXPECTED_ROWS,
    PortCandidate,
    discover_candidates,
    parse_hex_byte,
    print_port_table,
)
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
    PayloadError,
    decode_fsr,
    encode_frame,
    parse_dimension,
    parse_grid,
)
from tactile_layout import physical_invalid_grid_coordinates


HERE = Path(__file__).resolve().parent
DEFAULT_CONFIG_PATH = HERE / "config" / "tactile_pairing.json"
DEFAULT_MANUS_BRIDGE = HERE / "manus_ndjson_bridge" / "manus_ndjson_bridge.out"

CONFIG_SCHEMA = "pico_tactile_pairing_v1"
PAIRING_METHOD = "manual_left_press_right_by_exclusion_each_start"
MANUAL_PAIRING_RESULT_METHOD = "manual_left_press_right_by_exclusion"
SIDE_NAMES = ("left", "right")
MAX_CONFIG_BYTES = 64 * 1024
MAX_UINT32 = 0xFFFFFFFF
MAX_SAMPLE_BUFFER = 2048
MAX_CAPTURE_FRAMES = 10_000
READ_CHUNK_BYTES = 4096
HEALTH_ERROR_FIELDS = (
    "timeouts",
    "address_errors",
    "function_errors",
    "payload_errors",
    "io_errors",
    "parser_discarded",
    "parser_length_errors",
    "parser_tail_errors",
    "parser_crc_errors",
    "parser_overflows",
)
TRANSIENT_HEALTH_FIELDS = (
    "timeouts",
    "parser_discarded",
)
FATAL_SUBSCRIPTION_HEALTH_FIELDS = tuple(
    name for name in HEALTH_ERROR_FIELDS if name not in TRANSIENT_HEALTH_FIELDS
)

_PROFILE_NAME_RE = re.compile(r"^[a-z0-9][a-z0-9_]{0,63}$")
_GLOVE_ID_RE = re.compile(r"^[1-9][0-9]{0,9}$")
_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


class PairingError(RuntimeError):
    """配对环境、配置、串口或 MANUS 身份错误。"""


class ConfigError(PairingError):
    """永久配对配置缺失或不符合严格 schema。"""


class ResponseTimeout(PairingError):
    """触觉设备没有在期限内返回期望响应。"""


class PairingStopped(PairingError):
    """配对流程被要求正常停止。"""


class FrameGapError(PairingError):
    """订阅游标落后于有界环形缓冲，至少一帧已经不可恢复。"""

    def __init__(
        self,
        side: str,
        stream_id: str,
        expected_sequence: int,
        oldest_available_sequence: int,
    ) -> None:
        self.side = str(side)
        self.stream_id = str(stream_id)
        self.expected_sequence = int(expected_sequence)
        self.oldest_available_sequence = int(oldest_available_sequence)
        self.dropped_frames = max(
            0, self.oldest_available_sequence - self.expected_sequence
        )
        super().__init__(
            "{} tactile stream {} 丢失 {} 帧: expected_seq={}, oldest_available={}".format(
                self.side,
                self.stream_id,
                self.dropped_frames,
                self.expected_sequence,
                self.oldest_available_sequence,
            )
        )


class PairingRejected(PairingError):
    """证据不足、歧义或设备不健康，因此拒绝猜测左右手。"""

    def __init__(
        self,
        code: str,
        message: str,
        evidence: Sequence["ActivityEvidence"] = (),
    ) -> None:
        super().__init__(message)
        self.code = str(code)
        self.evidence = tuple(evidence)


@dataclass(frozen=True)
class PairingConfig:
    path: Path
    sha256: str
    document: Mapping[str, Any]
    manus_ids: Mapping[str, int]
    side_profiles: Mapping[str, str]
    profiles: Mapping[str, Mapping[str, Any]]


@dataclass(frozen=True)
class ObservedManus:
    identities: Mapping[str, int]
    frame_counts: Mapping[str, int]
    source: str
    observed_wall_ns: int
    live_snapshot: bool
    continuous_lease: bool = False
    service_instance_id: Optional[str] = None
    unknown_frames: int = 0
    unresolved_unknown_ids: Tuple[int, ...] = ()


@dataclass(frozen=True)
class TimedFsrFrame:
    recv_mono_ns: int
    recv_wall_ns: int
    wire_values: Tuple[int, ...]
    # 默认值保持现有 ``TimedFsrFrame(mono, wall, values)`` 三参数构造兼容。
    stream_seq: int = -1
    request_mono_ns: int = 0
    # Windows: Python perf_counter_ns() 使用 QPC，跨 Python/C++ 采集进程可比较。
    # recv_mono_ns 仍保留给进程内门禁（Windows 上为 GetTickCount64）。
    recv_qpc_ns: int = 0


@dataclass(frozen=True)
class ReaderHealth:
    valid_frames: int = 0
    timeouts: int = 0
    address_errors: int = 0
    function_errors: int = 0
    payload_errors: int = 0
    io_errors: int = 0
    missed_deadlines: int = 0
    buffer_drops: int = 0
    parser_discarded: int = 0
    parser_length_errors: int = 0
    parser_tail_errors: int = 0
    parser_crc_errors: int = 0
    parser_overflows: int = 0

    def subtract(self, earlier: "ReaderHealth") -> "ReaderHealth":
        values: Dict[str, int] = {}
        for name in self.__dataclass_fields__:
            values[name] = max(0, int(getattr(self, name)) - int(getattr(earlier, name)))
        return ReaderHealth(**values)


@dataclass(frozen=True)
class FrameBatch:
    """一个 side/stream 的连续、无缺口原始触觉帧批次。"""

    side: str
    stream_id: str
    frames: Tuple[TimedFsrFrame, ...]
    next_sequence: int
    health_delta: ReaderHealth


@dataclass(frozen=True)
class WindowCapture:
    start_mono_ns: int
    end_mono_ns: int
    duration_s: float
    frames: Mapping[str, Tuple[TimedFsrFrame, ...]]
    health_delta: Mapping[str, ReaderHealth]


@dataclass(frozen=True)
class ActivityEvidence:
    requested_side: str
    candidate_label: str
    port: str
    baseline_frames: int
    press_frames: int
    baseline_score: float
    raw_press_score: float
    score: float
    active_frame_count: int
    peak_active_cells: int


@dataclass(frozen=True)
class SideBinding:
    side: str
    stream_id: str
    candidate_label: str
    port_at_pairing: str
    manus_glove_id: int
    physical_profile: str
    mask_verified: bool


@dataclass(frozen=True)
class PairingResult:
    method: str
    paired_wall_ns: int
    config_sha256: str
    manus_source: str
    manus_observed_wall_ns: int
    manus_live_snapshot: bool
    bindings: Tuple[SideBinding, SideBinding]
    evidence: Tuple[ActivityEvidence, ...]
    scoring: Optional["ScoringConfig"] = None


def _strict_bindings_by_side(result: PairingResult) -> Dict[str, SideBinding]:
    if len(result.bindings) != len(SIDE_NAMES):
        raise PairingError("配对结果必须且只能包含 left/right 两个 binding")
    bindings: Dict[str, SideBinding] = {}
    for binding in result.bindings:
        if binding.side not in SIDE_NAMES:
            raise PairingError("配对结果包含未知 side={}".format(binding.side))
        if binding.side in bindings:
            raise PairingError("配对结果包含重复 side={}".format(binding.side))
        if not isinstance(binding.candidate_label, str) or not binding.candidate_label:
            raise PairingError("{} candidate_label 无效".format(binding.side))
        if not isinstance(binding.stream_id, str) or not binding.stream_id:
            raise PairingError("{} stream_id 无效".format(binding.side))
        bindings[binding.side] = binding
    if set(bindings) != set(SIDE_NAMES):
        raise PairingError("配对结果必须且只能包含 left/right")
    if bindings["left"].candidate_label == bindings["right"].candidate_label:
        raise PairingError("左右 binding 不能使用同一个 candidate")
    if bindings["left"].stream_id == bindings["right"].stream_id:
        raise PairingError("左右 binding 不能使用同一个 stream_id")
    return bindings


@dataclass(frozen=True)
class ScoringConfig:
    top_k: int = 16
    noise_floor: float = 1.0
    sigma_multiplier: float = 6.0
    min_frame_score: float = 10.0
    min_winner_score: float = 20.0
    max_non_target_score: float = 20.0
    min_ratio: float = 3.0
    min_margin: float = 10.0
    # 默认请求 60 Hz 时要求约 0.4 秒有效响应；同时只占 4 秒人工窗口的 10%，
    # 给操作者留出按下、调整接触位置和传感器静态回落的时间。
    min_active_frames: int = 24
    min_peak_active_cells: int = 2
    min_active_ratio: float = 0.10
    capture_ratio: float = 0.80

    def __post_init__(self) -> None:
        for name, maximum in (
            ("top_k", EXPECTED_ACTIVE),
            ("min_active_frames", MAX_CAPTURE_FRAMES),
            ("min_peak_active_cells", EXPECTED_ACTIVE),
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError("{} 必须是真整数".format(name))
            if value < 1 or value > maximum:
                raise ValueError("{} 必须在 1..{}".format(name, maximum))
        for name in (
            "noise_floor",
            "sigma_multiplier",
            "min_frame_score",
            "min_winner_score",
            "max_non_target_score",
            "min_ratio",
            "min_margin",
            "min_active_ratio",
            "capture_ratio",
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError("{} 必须是有限数值".format(name))
            if not math.isfinite(float(value)):
                raise ValueError("{} 必须是有限数值".format(name))
        if self.noise_floor <= 0:
            raise ValueError("noise_floor 必须 > 0")
        if self.sigma_multiplier < 0:
            raise ValueError("sigma_multiplier 必须 >= 0")
        if (
            self.min_frame_score < 0
            or self.min_winner_score <= 0
            or self.max_non_target_score <= 0
            or self.min_margin < 0
        ):
            raise ValueError(
                "评分阈值不能为负，min_winner_score/max_non_target_score 必须 > 0"
            )
        if self.max_non_target_score > self.min_winner_score:
            raise ValueError("max_non_target_score 不能大于 min_winner_score")
        if self.min_ratio <= 1:
            raise ValueError("min_ratio 必须 > 1")
        if self.min_active_ratio <= 0 or self.min_active_ratio > 1:
            raise ValueError("min_active_ratio 必须在 (0,1]")
        if self.capture_ratio <= 0 or self.capture_ratio > 1:
            raise ValueError("capture_ratio 必须在 (0,1]")


@dataclass
class _ReaderCounters:
    valid_frames: int = 0
    timeouts: int = 0
    address_errors: int = 0
    function_errors: int = 0
    payload_errors: int = 0
    io_errors: int = 0
    missed_deadlines: int = 0
    buffer_drops: int = 0


def _reject_duplicate_keys(pairs: Sequence[Tuple[str, Any]]) -> Dict[str, Any]:
    result: Dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ConfigError("配置包含重复字段: {!r}".format(key))
        result[key] = value
    return result


def _reject_json_constant(value: str) -> Any:
    raise ConfigError("配置不允许 NaN/Infinity: {}".format(value))


def _require_object(value: Any, context: str) -> Mapping[str, Any]:
    if not isinstance(value, dict):
        raise ConfigError("{} 必须是 JSON object".format(context))
    return value


def _require_exact_keys(
    value: Mapping[str, Any],
    required: Set[str],
    optional: Set[str],
    context: str,
) -> None:
    keys = set(value.keys())
    missing = sorted(required - keys)
    unknown = sorted(keys - required - optional)
    if missing:
        raise ConfigError("{} 缺少字段: {}".format(context, ", ".join(missing)))
    if unknown:
        raise ConfigError("{} 包含未知字段: {}".format(context, ", ".join(unknown)))


def _require_true_int(value: Any, context: str, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigError("{} 必须是整数".format(context))
    if value < minimum or value > maximum:
        raise ConfigError("{} 必须在 {}..{}".format(context, minimum, maximum))
    return value


def _parse_glove_id(value: Any, context: str) -> int:
    if not isinstance(value, str) or not _GLOVE_ID_RE.fullmatch(value):
        raise ConfigError("{} 必须是 uint32 十进制字符串".format(context))
    parsed = int(value, 10)
    if parsed < 1 or parsed > MAX_UINT32:
        raise ConfigError("{} 超出 uint32 范围".format(context))
    return parsed


def _validate_shape(value: Any, expected: Tuple[int, int], context: str) -> None:
    if not isinstance(value, list) or len(value) != 2:
        raise ConfigError("{} 必须是两个整数".format(context))
    actual = tuple(
        _require_true_int(item, "{}[{}]".format(context, index), 1, 64)
        for index, item in enumerate(value)
    )
    if actual != expected:
        raise ConfigError("{} 必须是 {}".format(context, list(expected)))


def _validate_profile(name: str, value: Any) -> Mapping[str, Any]:
    if not _PROFILE_NAME_RE.fullmatch(name):
        raise ConfigError("无效 profile 名称: {!r}".format(name))
    profile = _require_object(value, "profiles.{}".format(name))
    _require_exact_keys(
        profile,
        {
            "kind",
            "wire_grid",
            "physical_finger_count",
            "nominal_fingertip_shape",
            "effective_non_thumb_shape",
            "non_thumb_dead_physical_column",
            "physical_active_count",
            "palm_present",
            "mask_verified",
            "post_decode_invalid_cells",
            "note",
        },
        set(),
        "profiles.{}".format(name),
    )
    if profile["kind"] != "five_fingertip_arrays_4x8_four_4x7":
        raise ConfigError("profiles.{}.kind 不受支持".format(name))
    if _require_true_int(
        profile["physical_finger_count"],
        "profiles.{}.physical_finger_count".format(name),
        1,
        5,
    ) != 5:
        raise ConfigError("profiles.{}.physical_finger_count 必须为 5".format(name))
    _validate_shape(
        profile["nominal_fingertip_shape"],
        (4, 8),
        "profiles.{}.nominal_fingertip_shape".format(name),
    )
    _validate_shape(
        profile["effective_non_thumb_shape"],
        (4, 7),
        "profiles.{}.effective_non_thumb_shape".format(name),
    )
    if _require_true_int(
        profile["non_thumb_dead_physical_column"],
        "profiles.{}.non_thumb_dead_physical_column".format(name),
        0,
        7,
    ) != 7:
        raise ConfigError(
            "profiles.{}.non_thumb_dead_physical_column 必须为 7".format(name)
        )
    if _require_true_int(
        profile["physical_active_count"],
        "profiles.{}.physical_active_count".format(name),
        1,
        160,
    ) != 144:
        raise ConfigError("profiles.{}.physical_active_count 必须为 144".format(name))
    if profile["palm_present"] is not False:
        raise ConfigError("profiles.{}.palm_present 必须为 false".format(name))
    if not isinstance(profile["mask_verified"], bool):
        raise ConfigError("profiles.{}.mask_verified 必须是 bool".format(name))
    if not isinstance(profile["note"], str) or not profile["note"].strip():
        raise ConfigError("profiles.{}.note 不能为空".format(name))

    grid = _require_object(profile["wire_grid"], "profiles.{}.wire_grid".format(name))
    _require_exact_keys(
        grid,
        {"rows", "cols", "active_count", "cellmap_sha256"},
        set(),
        "profiles.{}.wire_grid".format(name),
    )
    if _require_true_int(grid["rows"], "wire_grid.rows", 1, 32) != EXPECTED_ROWS:
        raise ConfigError("wire_grid.rows 当前必须为 {}".format(EXPECTED_ROWS))
    if _require_true_int(grid["cols"], "wire_grid.cols", 1, 32) != EXPECTED_COLS:
        raise ConfigError("wire_grid.cols 当前必须为 {}".format(EXPECTED_COLS))
    if (
        _require_true_int(grid["active_count"], "wire_grid.active_count", 1, 1024)
        != EXPECTED_ACTIVE
    ):
        raise ConfigError("wire_grid.active_count 当前必须为 {}".format(EXPECTED_ACTIVE))
    digest = grid["cellmap_sha256"]
    if not isinstance(digest, str) or not _SHA256_RE.fullmatch(digest):
        raise ConfigError("wire_grid.cellmap_sha256 必须是64位小写十六进制")

    invalid = profile["post_decode_invalid_cells"]
    if not isinstance(invalid, list):
        raise ConfigError("profiles.{}.post_decode_invalid_cells 必须是数组".format(name))
    cells: Set[Tuple[int, int]] = set()
    for index, cell in enumerate(invalid):
        if not isinstance(cell, list) or len(cell) != 2:
            raise ConfigError("post_decode_invalid_cells[{}] 必须是 [row,col]".format(index))
        row = _require_true_int(cell[0], "invalid row", 0, EXPECTED_ROWS - 1)
        col = _require_true_int(cell[1], "invalid col", 0, EXPECTED_COLS - 1)
        if (row, col) in cells:
            raise ConfigError("post_decode_invalid_cells 含重复坐标 {}".format([row, col]))
        cells.add((row, col))
    if profile["mask_verified"]:
        if len(cells) != 16:
            raise ConfigError("已验证的4x7 profile 必须恰有16个后解码无效点")
        expected_cells = set(physical_invalid_grid_coordinates())
        if cells != expected_cells:
            raise ConfigError(
                "已验证4x7掩码必须精确覆盖四个非拇指上方实物阵列靠近指根的16点"
            )
    elif cells:
        raise ConfigError("mask_verified=false 时不得提前应用 post_decode_invalid_cells")
    return profile


def validate_config_document(document: Any) -> Tuple[Dict[str, int], Dict[str, str], Dict[str, Mapping[str, Any]]]:
    root = _require_object(document, "root")
    _require_exact_keys(
        root,
        {"schema", "pairing_method", "sides", "profiles"},
        {"_meta"},
        "root",
    )
    if root["schema"] != CONFIG_SCHEMA:
        raise ConfigError("不支持的 schema: {!r}".format(root["schema"]))
    if root["pairing_method"] != PAIRING_METHOD:
        raise ConfigError("pairing_method 必须为 {}".format(PAIRING_METHOD))
    if "_meta" in root:
        _require_object(root["_meta"], "_meta")

    profiles_obj = _require_object(root["profiles"], "profiles")
    if not profiles_obj:
        raise ConfigError("profiles 不能为空")
    profiles: Dict[str, Mapping[str, Any]] = {
        name: _validate_profile(name, value) for name, value in profiles_obj.items()
    }

    sides_obj = _require_object(root["sides"], "sides")
    if set(sides_obj.keys()) != set(SIDE_NAMES):
        raise ConfigError("sides 必须且只能包含 left/right")
    manus_ids: Dict[str, int] = {}
    side_profiles: Dict[str, str] = {}
    for side in SIDE_NAMES:
        item = _require_object(sides_obj[side], "sides.{}".format(side))
        _require_exact_keys(
            item,
            {"manus_glove_id", "manus_expected_side", "physical_profile"},
            set(),
            "sides.{}".format(side),
        )
        if item["manus_expected_side"] != side:
            raise ConfigError("sides.{}.manus_expected_side 必须为 {}".format(side, side))
        manus_ids[side] = _parse_glove_id(
            item["manus_glove_id"], "sides.{}.manus_glove_id".format(side)
        )
        profile_name = item["physical_profile"]
        if not isinstance(profile_name, str) or profile_name not in profiles:
            raise ConfigError("sides.{}.physical_profile 引用了不存在的 profile".format(side))
        side_profiles[side] = profile_name
    if manus_ids["left"] == manus_ids["right"]:
        raise ConfigError("左右 MANUS glove_id 不能相同")
    return manus_ids, side_profiles, profiles


def _deep_freeze(value: Any) -> Any:
    if isinstance(value, dict):
        return MappingProxyType({key: _deep_freeze(item) for key, item in value.items()})
    if isinstance(value, list):
        return tuple(_deep_freeze(item) for item in value)
    return value


def load_pairing_config(path: Path) -> PairingConfig:
    source = Path(path).resolve()
    try:
        size = source.stat().st_size
    except OSError as exc:
        raise ConfigError("无法读取配对配置 {}: {}".format(source, exc)) from exc
    if size < 2 or size > MAX_CONFIG_BYTES:
        raise ConfigError("配置大小必须在 2..{} 字节".format(MAX_CONFIG_BYTES))
    try:
        raw = source.read_bytes()
        if len(raw) < 2 or len(raw) > MAX_CONFIG_BYTES:
            raise ConfigError("配置实际读取大小必须在 2..{} 字节".format(MAX_CONFIG_BYTES))
        text = raw.decode("utf-8", errors="strict")
        document = json.loads(
            text,
            object_pairs_hook=_reject_duplicate_keys,
            parse_constant=_reject_json_constant,
        )
    except ConfigError:
        raise
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ConfigError("配置解析失败 {}: {}".format(source, exc)) from exc
    manus_ids, side_profiles, profiles = validate_config_document(document)
    frozen_document = _deep_freeze(document)
    return PairingConfig(
        path=source,
        sha256=hashlib.sha256(raw).hexdigest(),
        document=frozen_document,
        manus_ids=MappingProxyType(dict(manus_ids)),
        side_profiles=MappingProxyType(dict(side_profiles)),
        profiles=frozen_document["profiles"],
    )


def atomic_write_pairing_config(document: Mapping[str, Any], path: Path) -> PairingConfig:
    """显式配置工具使用的同目录原子写入；动态按压配对不会调用它。"""
    validate_config_document(document)
    try:
        encoded = (
            json.dumps(
                document,
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
                allow_nan=False,
            )
            + "\n"
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise ConfigError("配置无法序列化: {}".format(exc)) from exc
    if len(encoded) > MAX_CONFIG_BYTES:
        raise ConfigError("配置序列化后超过 {} 字节".format(MAX_CONFIG_BYTES))

    requested = Path(path).absolute()
    requested.parent.mkdir(parents=True, exist_ok=True)
    parent = requested.parent.resolve()
    target = parent / requested.name
    if target.is_symlink():
        raise ConfigError("拒绝通过符号链接写入配对配置: {}".format(target))
    fd, temp_name = tempfile.mkstemp(
        prefix=".tactile_pairing.", suffix=".tmp", dir=str(target.parent)
    )
    temp_path = Path(temp_name)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        load_pairing_config(temp_path)
        os.replace(str(temp_path), str(target))
        if os.name != "nt":
            directory_fd = os.open(str(target.parent), os.O_RDONLY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
    finally:
        if temp_path.exists():
            temp_path.unlink()
    return load_pairing_config(target)


def _coerce_observed_glove_id(value: Any, context: str) -> int:
    if isinstance(value, bool):
        raise PairingError("{} 不是有效 uint32 glove_id".format(context))
    if isinstance(value, int):
        parsed = value
    elif isinstance(value, str) and _GLOVE_ID_RE.fullmatch(value):
        parsed = int(value, 10)
    else:
        raise PairingError("{} 不是有效 uint32 glove_id".format(context))
    if parsed < 1 or parsed > MAX_UINT32:
        raise PairingError("{} 超出 uint32 范围".format(context))
    return parsed


class _ManusAccumulator:
    def __init__(self) -> None:
        self.ids: Dict[str, Set[int]] = {"left": set(), "right": set()}
        self.counts: Dict[str, int] = {"left": 0, "right": 0}
        self.unknown_frames = 0
        self.unknown_ids: Set[int] = set()

    def add(self, obj: Mapping[str, Any]) -> bool:
        if obj.get("type") != "manus_frame":
            return False
        side = str(obj.get("side", "unknown")).lower()
        if side not in SIDE_NAMES:
            self.unknown_frames += 1
            value = obj.get("glove_id")
            if value is None:
                raise PairingError("MANUS unknown-side frame is missing glove_id")
            self.unknown_ids.add(
                _coerce_observed_glove_id(value, "MANUS unknown-side glove_id")
            )
            return True
        glove_id = _coerce_observed_glove_id(obj.get("glove_id"), "MANUS glove_id")
        self.ids[side].add(glove_id)
        self.counts[side] += 1
        return True

    def ready(self, minimum_frames_per_side: int) -> bool:
        return all(self.counts[side] >= minimum_frames_per_side for side in SIDE_NAMES)

    def finish(
        self,
        source: str,
        observed_wall_ns: int,
        live_snapshot: bool,
    ) -> ObservedManus:
        identities: Dict[str, int] = {}
        for side in SIDE_NAMES:
            if len(self.ids[side]) != 1:
                raise PairingError(
                    "MANUS {} 必须恰有一个 glove_id，实际 {}".format(
                        side, sorted(self.ids[side])
                    )
                )
            identities[side] = next(iter(self.ids[side]))
        if identities["left"] == identities["right"]:
            raise PairingError("同一个 MANUS glove_id 同时上报为 left/right")
        resolved_ids = set(identities.values())
        unresolved = tuple(sorted(self.unknown_ids - resolved_ids))
        if unresolved:
            raise PairingError(
                "MANUS 仍存在无法解析 side 的额外 glove_id: {}".format(list(unresolved))
            )
        return ObservedManus(
            identities=MappingProxyType(dict(identities)),
            frame_counts=MappingProxyType(dict(self.counts)),
            source=source,
            observed_wall_ns=observed_wall_ns,
            live_snapshot=live_snapshot,
            continuous_lease=False,
            unknown_frames=self.unknown_frames,
            unresolved_unknown_ids=unresolved,
        )


def configured_manus(config: PairingConfig) -> ObservedManus:
    return ObservedManus(
        identities=MappingProxyType(dict(config.manus_ids)),
        frame_counts=MappingProxyType({"left": 0, "right": 0}),
        source="config_only",
        observed_wall_ns=0,
        live_snapshot=False,
    )


def discover_manus_from_ndjson(path: Path, max_frames: int = 2000) -> ObservedManus:
    source = Path(path).resolve()
    if max_frames < 2:
        raise PairingError("max_frames 必须 >= 2")
    accumulator = _ManusAccumulator()
    seen_frames = 0
    try:
        with source.open("r", encoding="utf-8", errors="strict") as stream:
            for line_number, line in enumerate(stream, 1):
                stripped = line.strip()
                if not stripped or not stripped.startswith("{"):
                    continue
                try:
                    obj = json.loads(
                        stripped,
                        object_pairs_hook=_reject_duplicate_keys,
                        parse_constant=_reject_json_constant,
                    )
                except (json.JSONDecodeError, ConfigError) as exc:
                    raise PairingError(
                        "MANUS NDJSON 第 {} 行无效: {}".format(line_number, exc)
                    ) from exc
                if not isinstance(obj, dict):
                    continue
                if accumulator.add(obj):
                    seen_frames += 1
                    if seen_frames >= max_frames:
                        break
    except PairingError:
        raise
    except (OSError, UnicodeError) as exc:
        raise PairingError("无法读取 MANUS NDJSON {}: {}".format(source, exc)) from exc
    if seen_frames == 0:
        raise PairingError("MANUS NDJSON 中没有 manus_frame")
    return accumulator.finish(
        "historical_ndjson:{}".format(source),
        observed_wall_ns=0,
        live_snapshot=False,
    )


def discover_manus_snapshot_from_bridge(
    bridge_path: Path,
    timeout_s: float = 15.0,
    minimum_frames_per_side: int = 5,
) -> ObservedManus:
    bridge = Path(bridge_path).resolve()
    if not bridge.is_file():
        raise PairingError("找不到 MANUS bridge: {}".format(bridge))
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise PairingError("MANUS bridge timeout 必须 > 0")

    env = dict(os.environ)
    # 身份快照不得触发 bridge 的可选校准下发副作用。
    env.pop("MANUS_CALIB_LEFT", None)
    env.pop("MANUS_CALIB_RIGHT", None)
    libdir = bridge.parent / "ManusSDK" / "lib"
    if libdir.is_dir():
        env["LD_LIBRARY_PATH"] = "{}:{}".format(
            libdir, env.get("LD_LIBRARY_PATH", "")
        )
    try:
        process = subprocess.Popen(
            [str(bridge)],
            stdout=subprocess.PIPE,
            stderr=None,
            bufsize=1,
            text=True,
            encoding="utf-8",
            errors="replace",
            env=env,
        )
    except OSError as exc:
        raise PairingError("无法启动 MANUS bridge {}: {}".format(bridge, exc)) from exc

    lines: "queue.Queue[Optional[str]]" = queue.Queue()

    def reader() -> None:
        try:
            assert process.stdout is not None
            for line in process.stdout:
                lines.put(line)
        finally:
            lines.put(None)

    thread = threading.Thread(target=reader, name="manus-identity-reader", daemon=True)
    thread.start()
    accumulator = _ManusAccumulator()
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline and not accumulator.ready(minimum_frames_per_side):
            remaining = max(0.01, deadline - time.monotonic())
            try:
                line = lines.get(timeout=min(0.25, remaining))
            except queue.Empty:
                if process.poll() is not None:
                    break
                continue
            if line is None:
                break
            stripped = line.strip()
            if not stripped.startswith("{"):
                continue
            try:
                obj = json.loads(stripped)
            except json.JSONDecodeError:
                continue
            if isinstance(obj, dict):
                accumulator.add(obj)
        if not accumulator.ready(minimum_frames_per_side):
            raise PairingError(
                "MANUS bridge 在 {:.1f}s 内未同时得到左右手有效帧；counts={}".format(
                    timeout_s, accumulator.counts
                )
            )
        return accumulator.finish(
            "diagnostic_bridge_snapshot:{}".format(bridge),
            observed_wall_ns=time.time_ns(),
            live_snapshot=True,
        )
    finally:
        if process.poll() is None:
            try:
                if os.name == "nt":
                    process.terminate()
                else:
                    process.send_signal(signal.SIGINT)
                process.wait(timeout=3.0)
            except Exception:
                try:
                    process.kill()
                    process.wait(timeout=1.0)
                except Exception:
                    pass
        if process.stdout is not None:
            try:
                process.stdout.close()
            except Exception:
                pass
        thread.join(timeout=1.0)


def validate_manus_against_config(observed: ObservedManus, config: PairingConfig) -> None:
    if set(observed.identities.keys()) != set(SIDE_NAMES):
        raise PairingError("MANUS 身份集合必须且只能包含 left/right")
    if observed.unresolved_unknown_ids:
        raise PairingError(
            "MANUS 仍有 unresolved glove_id: {}".format(
                list(observed.unresolved_unknown_ids)
            )
        )
    for side in SIDE_NAMES:
        actual = observed.identities.get(side)
        expected = config.manus_ids[side]
        if actual != expected:
            raise PairingError(
                "MANUS {} glove_id 不符: expected={}, observed={}".format(
                    side, expected, actual
                )
            )
    if observed.identities["left"] == observed.identities["right"]:
        raise PairingError("MANUS 左右 glove_id 重复")


def _load_pyserial() -> Tuple[Any, Any]:
    try:
        import serial  # type: ignore
        from serial.tools import list_ports  # type: ignore
    except ImportError as exc:
        raise PairingError(
            "缺少 pyserial。请在实际采集 Python 环境执行: python3 -m pip install pyserial"
        ) from exc
    return serial, list_ports


def grid_cellmap_bytes(grid: GridConfig) -> bytes:
    return b"".join(mask.to_bytes(4, byteorder="little", signed=False) for mask in grid.row_masks)


def grid_cellmap_sha256(grid: GridConfig) -> str:
    return hashlib.sha256(grid_cellmap_bytes(grid)).hexdigest()


class TactileReader(threading.Thread):
    """一只触觉设备的持续只读采样线程。"""

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
        super().__init__(name="pairing-{}".format(candidate.label), daemon=True)
        self.candidate = candidate
        self.serial_module = serial_module
        self.address = address
        self.rate_hz = rate_hz
        self.response_timeout_ms = response_timeout_ms
        self.start_event = start_event
        self.stop_event = stop_event
        self.ready_event = threading.Event()
        self.stream_id = uuid.uuid4().hex
        self.grid: Optional[GridConfig] = None
        self.error: Optional[str] = None
        self.parser = FrameStreamParser()
        self._serial = None
        self._samples: Deque[TimedFsrFrame] = deque(maxlen=MAX_SAMPLE_BUFFER)
        self._samples_lock = threading.Lock()
        self._samples_changed = threading.Condition(self._samples_lock)
        self._next_stream_seq = 0
        self._notification_seq = 0
        self._active_capture_token: Optional[str] = None
        self._active_capture_frames: List[TimedFsrFrame] = []
        self._health = _ReaderCounters()
        self._health_lock = threading.Lock()
        self._parser_lock = threading.Lock()

    def _increment(self, name: str, amount: int = 1) -> None:
        with self._health_lock:
            setattr(self._health, name, int(getattr(self._health, name)) + int(amount))
        # 错误/超时没有新帧可触发 notify；主动唤醒订阅者让其及时 fail-closed。
        if name in HEALTH_ERROR_FIELDS:
            self.notify_subscribers()

    def notify_subscribers(self) -> None:
        with self._samples_changed:
            self._notification_seq += 1
            self._samples_changed.notify_all()

    def health_snapshot(self) -> ReaderHealth:
        with self._health_lock:
            counters = _ReaderCounters(**self._health.__dict__)
        with self._parser_lock:
            parser_stats = self.parser.stats
            return ReaderHealth(
                valid_frames=counters.valid_frames,
                timeouts=counters.timeouts,
                address_errors=counters.address_errors,
                function_errors=counters.function_errors,
                payload_errors=counters.payload_errors,
                io_errors=counters.io_errors,
                missed_deadlines=counters.missed_deadlines,
                buffer_drops=counters.buffer_drops,
                parser_discarded=parser_stats.discarded_bytes,
                parser_length_errors=parser_stats.length_errors,
                parser_tail_errors=parser_stats.tail_errors,
                parser_crc_errors=parser_stats.crc_errors,
                parser_overflows=parser_stats.buffer_overflows,
            )

    def frames_between(self, start_ns: int, end_ns: int) -> Tuple[TimedFsrFrame, ...]:
        with self._samples_changed:
            return tuple(
                frame
                for frame in self._samples
                if start_ns <= frame.recv_mono_ns < end_ns
            )

    def begin_capture(self, token: str) -> None:
        with self._samples_changed:
            if self._active_capture_token is not None:
                raise PairingError("{} 已有活动采样窗口".format(self.candidate.label))
            self._active_capture_token = token
            self._active_capture_frames = []

    def end_capture(
        self, token: str, start_ns: int, end_ns: int
    ) -> Tuple[TimedFsrFrame, ...]:
        with self._samples_changed:
            if self._active_capture_token != token:
                raise PairingError("{} 采样窗口 token 不一致".format(self.candidate.label))
            captured = self._active_capture_frames
            self._active_capture_token = None
            self._active_capture_frames = []
        # 过滤/构造 tuple 放在热路径锁外，避免长窗口阻塞串口 append。
        return tuple(
            frame
            for frame in captured
            if start_ns <= frame.recv_mono_ns < end_ns
        )

    def cancel_capture(self, token: str) -> None:
        with self._samples_changed:
            if self._active_capture_token == token:
                self._active_capture_token = None
                self._active_capture_frames = []

    def latest_frame_age_ms(self) -> float:
        with self._samples_changed:
            if not self._samples:
                return float("inf")
            latest_ns = self._samples[-1].recv_mono_ns
        return max(0.0, (time.monotonic_ns() - latest_ns) / 1_000_000.0)

    def subscription_sequence_at(self, boundary_mono_ns: int) -> int:
        """返回共同时间边界处的首个 seq；调用后到达的新帧不会被漏掉。"""
        with self._samples_changed:
            for frame in self._samples:
                if frame.recv_mono_ns >= boundary_mono_ns:
                    return frame.stream_seq
            return self._next_stream_seq

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

    def _transact_timed(self, function: int) -> Tuple[Frame, int]:
        if function not in READ_ONLY_FUNCTIONS:
            raise PairingError("拒绝非只读功能码 0x{:02X}".format(function))
        if self.stop_event.is_set():
            raise PairingStopped("配对已停止")
        if self._serial is None:
            raise PairingError("串口尚未打开")
        request = encode_frame(self.address, function)
        request_mono_ns = time.monotonic_ns()
        written = self._serial.write(request)
        if written != len(request):
            raise PairingError("串口短写: {} / {} bytes".format(written, len(request)))

        deadline_ns = time.monotonic_ns() + self.response_timeout_ms * 1_000_000
        while not self.stop_event.is_set() and time.monotonic_ns() < deadline_ns:
            waiting = int(getattr(self._serial, "in_waiting", 0) or 0)
            chunk = self._serial.read(min(max(waiting, 1), READ_CHUNK_BYTES))
            if not chunk:
                continue
            with self._parser_lock:
                frames = self.parser.feed(chunk)
            for frame in frames:
                if frame.address != self.address:
                    self._increment("address_errors")
                    continue
                if frame.function != function:
                    self._increment("function_errors")
                    continue
                return frame, request_mono_ns
        if self.stop_event.is_set():
            raise PairingStopped("配对已停止")
        with self._parser_lock:
            self.parser.reset()
        try:
            self._serial.reset_input_buffer()
        except Exception:
            pass
        self._increment("timeouts")
        raise ResponseTimeout(
            "{} 等待 address=0x{:02X}, function=0x{:02X} 超时".format(
                self.candidate.device, self.address, function
            )
        )

    def _transact(self, function: int) -> Frame:
        frame, _request_mono_ns = self._transact_timed(function)
        return frame

    def _query_grid(self) -> GridConfig:
        rows = parse_dimension(self._transact(FUNC_ROWS), self.address, FUNC_ROWS)
        cols = parse_dimension(self._transact(FUNC_COLS), self.address, FUNC_COLS)
        return parse_grid(self._transact(FUNC_CELLMAP), self.address, rows, cols)

    def _append_sample(self, frame: TimedFsrFrame) -> None:
        overwritten = False
        with self._samples_changed:
            stream_seq = self._next_stream_seq
            self._next_stream_seq += 1
            sequenced = TimedFsrFrame(
                recv_mono_ns=frame.recv_mono_ns,
                recv_wall_ns=frame.recv_wall_ns,
                wire_values=frame.wire_values,
                stream_seq=stream_seq,
                request_mono_ns=frame.request_mono_ns,
                recv_qpc_ns=frame.recv_qpc_ns,
            )
            overwritten = len(self._samples) == self._samples.maxlen
            self._samples.append(sequenced)
            if self._active_capture_token is not None:
                self._active_capture_frames.append(sequenced)
            self._notification_seq += 1
            self._samples_changed.notify_all()
        if overwritten:
            # 这是全局 ring overwrite 遥测；只有订阅 cursor 落后才算该订阅丢帧。
            self._increment("buffer_drops")
        self._increment("valid_frames")

    def _sample_loop(self) -> None:
        assert self.grid is not None
        period_ns = max(1, int(1_000_000_000.0 / self.rate_hz))
        next_request_ns = time.monotonic_ns()
        while not self.stop_event.is_set():
            now_ns = time.monotonic_ns()
            if now_ns < next_request_ns:
                # Python 3.11+ 在 Windows 上的 time.sleep 使用高分辨率 waitable timer；
                # threading.Event.wait 的毫秒级等待会被约15.6ms系统粒度拖慢，60Hz
                # 请求因此退化到约30Hz。最大等待不超过一个采样周期，退出延迟可控。
                time.sleep((next_request_ns - now_ns) / 1_000_000_000.0)
                continue
            try:
                response, request_mono_ns = self._transact_timed(FUNC_FSR)
                sample = decode_fsr(response, self.address, self.grid)
                recv_mono_ns = time.monotonic_ns()
                recv_qpc_ns = time.perf_counter_ns()
                self._append_sample(
                    TimedFsrFrame(
                        recv_mono_ns=recv_mono_ns,
                        recv_wall_ns=time.time_ns(),
                        wire_values=sample.wire_values,
                        request_mono_ns=request_mono_ns,
                        recv_qpc_ns=recv_qpc_ns,
                    )
                )
            except PairingStopped:
                break
            except ResponseTimeout:
                pass
            except PayloadError:
                self._increment("payload_errors")
            except Exception:
                self._increment("io_errors")
                raise

            next_request_ns += period_ns
            now_ns = time.monotonic_ns()
            if next_request_ns < now_ns:
                # 轻微晚于目标时刻时直接进入下一次单请求/单响应，不再人为额外
                # 跳过一个完整周期。只有落后超过整周期才计 missed 并推进相位；
                # 始终没有并发请求，也不会积累无界追赶突发。
                missed = int((now_ns - next_request_ns) // period_ns)
                if missed:
                    self._increment("missed_deadlines", missed)
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
                raise PairingError(
                    "{} grid 不符: {}x{} active={}".format(
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
        except PairingStopped:
            pass
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
            self.notify_subscribers()


class FrameSubscription:
    """单消费者、有界且 fail-closed 的连续触觉帧订阅。"""

    def __init__(
        self,
        reader: TactileReader,
        side: str,
        start_mono_ns: int,
        start_wall_ns: int,
        next_sequence: int,
        baseline_health: Optional[ReaderHealth] = None,
    ) -> None:
        normalized = str(side).lower()
        if normalized not in SIDE_NAMES:
            raise PairingError("未知 tactile side={}".format(side))
        if isinstance(start_mono_ns, bool) or not isinstance(start_mono_ns, int):
            raise PairingError("start_mono_ns 必须是整数")
        if isinstance(start_wall_ns, bool) or not isinstance(start_wall_ns, int):
            raise PairingError("start_wall_ns 必须是整数")
        if isinstance(next_sequence, bool) or not isinstance(next_sequence, int):
            raise PairingError("next_sequence 必须是整数")
        if start_mono_ns < 0 or start_wall_ns < 0 or next_sequence < 0:
            raise PairingError("订阅边界和 sequence 不能为负数")
        self.side = normalized
        self.stream_id = reader.stream_id
        self.start_mono_ns = start_mono_ns
        self.start_wall_ns = start_wall_ns
        self.initial_sequence = next_sequence
        self._reader = reader
        self._next_sequence = next_sequence
        self._last_health = baseline_health or reader.health_snapshot()
        self._consume_lock = threading.Lock()
        self._closed = threading.Event()
        self._failure: Optional[PairingError] = None

    @property
    def next_sequence(self) -> int:
        return self._next_sequence

    @property
    def closed(self) -> bool:
        return self._closed.is_set()

    def close(self) -> None:
        """幂等关闭；可以由消费线程之外的控制线程调用。"""
        self._closed.set()
        self._reader.notify_subscribers()

    def poll(self, max_frames: int = 256) -> FrameBatch:
        """立即读取当前可用帧；没有新帧时返回空批次。"""
        return self.read_batch(max_frames=max_frames, timeout_s=0.0)

    def read_batch(
        self,
        max_frames: int = 256,
        timeout_s: Optional[float] = None,
    ) -> FrameBatch:
        """等待并复制一批连续帧；处理帧数据时不持有 reader 的锁。"""
        if (
            isinstance(max_frames, bool)
            or not isinstance(max_frames, int)
            or max_frames < 1
            or max_frames > MAX_SAMPLE_BUFFER
        ):
            raise PairingError(
                "max_frames 必须是 1..{} 的整数".format(MAX_SAMPLE_BUFFER)
            )
        if timeout_s is not None:
            if isinstance(timeout_s, bool):
                raise PairingError("timeout_s 必须是有限非负数或 None")
            try:
                timeout_value = float(timeout_s)
            except (TypeError, ValueError):
                raise PairingError("timeout_s 必须是有限非负数或 None")
            if (
                not math.isfinite(timeout_value)
                or timeout_value < 0
                or timeout_value > 60.0
            ):
                raise PairingError("timeout_s 必须是 0..60 的有限数或 None")
        else:
            timeout_value = None

        if not self._consume_lock.acquire(False):
            raise PairingError(
                "{} tactile subscription 仅允许一个消费者".format(self.side)
            )
        try:
            return self._read_batch_locked(max_frames, timeout_value)
        finally:
            self._consume_lock.release()

    def _terminal_state_error(self) -> Optional[PairingError]:
        if self._failure is not None:
            return self._failure
        if self._closed.is_set():
            return PairingStopped("{} tactile subscription 已关闭".format(self.side))
        if self._reader.error:
            return PairingError(
                "{} tactile stream 已失败: {}".format(
                    self.side, self._reader.error
                )
            )
        if self._reader.stop_event.is_set():
            return PairingStopped("{} tactile stream 已停止".format(self.side))
        return None

    def _raise_terminal_state(self) -> None:
        error = self._terminal_state_error()
        if error is not None:
            if self._failure is None and not self._closed.is_set():
                self._failure = error
            raise error

    def _capture_available_locked(
        self, max_frames: int
    ) -> Tuple[TimedFsrFrame, ...]:
        reader = self._reader
        oldest_sequence = (
            reader._samples[0].stream_seq
            if reader._samples
            else reader._next_stream_seq
        )
        if self._next_sequence < oldest_sequence:
            error = FrameGapError(
                self.side,
                self.stream_id,
                self._next_sequence,
                oldest_sequence,
            )
            self._failure = error
            raise error
        if self._next_sequence > reader._next_stream_seq:
            error = PairingError(
                "{} tactile subscription cursor 超过 stream 尾部: {} > {}".format(
                    self.side, self._next_sequence, reader._next_stream_seq
                )
            )
            self._failure = error
            raise error

        frames: List[TimedFsrFrame] = []
        expected = self._next_sequence
        for frame in reader._samples:
            if frame.stream_seq < expected:
                continue
            if frame.stream_seq != expected:
                error = FrameGapError(
                    self.side,
                    self.stream_id,
                    expected,
                    frame.stream_seq,
                )
                self._failure = error
                raise error
            frames.append(frame)
            expected += 1
            if len(frames) >= max_frames:
                break
        return tuple(frames)

    def _checked_health(self) -> Tuple[ReaderHealth, ReaderHealth]:
        current = self._reader.health_snapshot()
        delta = current.subtract(self._last_health)
        issues = [
            "{}={}".format(name, getattr(delta, name))
            for name in FATAL_SUBSCRIPTION_HEALTH_FIELDS
            if getattr(delta, name)
        ]
        if issues:
            error = PairingError(
                "{} tactile stream 健康增量异常: {}".format(
                    self.side, "; ".join(issues)
                )
            )
            self._failure = error
            raise error
        return current, delta

    def _read_batch_locked(
        self, max_frames: int, timeout_s: Optional[float]
    ) -> FrameBatch:
        deadline_ns = (
            None
            if timeout_s is None
            else time.monotonic_ns() + int(timeout_s * 1_000_000_000.0)
        )
        reader = self._reader
        while True:
            self._raise_terminal_state()
            with reader._samples_changed:
                self._raise_terminal_state()
                notification_seq = reader._notification_seq
                frames = self._capture_available_locked(max_frames)

            current_health, health_delta = self._checked_health()
            self._raise_terminal_state()
            if frames:
                next_sequence = frames[-1].stream_seq + 1
                self._next_sequence = next_sequence
                self._last_health = current_health
                return FrameBatch(
                    side=self.side,
                    stream_id=self.stream_id,
                    frames=frames,
                    next_sequence=next_sequence,
                    health_delta=health_delta,
                )

            remaining_s: Optional[float]
            if deadline_ns is None:
                remaining_s = None
            else:
                remaining_ns = deadline_ns - time.monotonic_ns()
                if remaining_ns <= 0:
                    self._last_health = current_health
                    return FrameBatch(
                        side=self.side,
                        stream_id=self.stream_id,
                        frames=(),
                        next_sequence=self._next_sequence,
                        health_delta=health_delta,
                    )
                remaining_s = remaining_ns / 1_000_000_000.0

            # 健康快照在 ring 锁外获取；generation 检查消除快照期间的丢唤醒竞态。
            with reader._samples_changed:
                self._raise_terminal_state()
                if reader._notification_seq != notification_seq:
                    continue
                reader._samples_changed.wait(remaining_s)


@dataclass(frozen=True)
class PairedSubscriptions:
    """从同一单调时钟边界开始的左右两路订阅。"""

    start_mono_ns: int
    start_wall_ns: int
    by_side: Mapping[str, FrameSubscription]

    def close(self) -> None:
        for side in SIDE_NAMES:
            subscription = self.by_side.get(side)
            if subscription is not None:
                subscription.close()


class PairingSession:
    """同时持有两路触觉串口，并按共同单调时钟切取采样窗口。"""

    def __init__(
        self,
        candidates: Sequence[PortCandidate],
        serial_module: Any,
        address: int = DEFAULT_ADDRESS,
        rate_hz: float = 60.0,
        response_timeout_ms: int = 200,
    ) -> None:
        if len(candidates) != 2:
            raise PairingError("当前配对要求恰好两只触觉设备")
        if not math.isfinite(rate_hz) or rate_hz <= 0 or rate_hz > 100:
            raise PairingError("rate_hz 必须在 (0,100]")
        if response_timeout_ms < 10 or response_timeout_ms > 5000:
            raise PairingError("response_timeout_ms 必须在 10..5000")
        self.candidates = tuple(candidates)
        self.serial_module = serial_module
        self.address = address
        self.rate_hz = rate_hz
        self.response_timeout_ms = response_timeout_ms
        self.start_event = threading.Event()
        self.stop_event = threading.Event()
        self.readers = tuple(
            TactileReader(
                candidate,
                serial_module,
                address,
                rate_hz,
                response_timeout_ms,
                self.start_event,
                self.stop_event,
            )
            for candidate in self.candidates
        )
        self._started: List[TactileReader] = []
        self._close_lock = threading.Lock()
        self._closing = False
        self._closed = False
        self._close_failures: Tuple[str, ...] = ()

    @classmethod
    def open(
        cls,
        candidates: Sequence[PortCandidate],
        serial_module: Any,
        address: int = DEFAULT_ADDRESS,
        rate_hz: float = 60.0,
        response_timeout_ms: int = 200,
    ) -> "PairingSession":
        session = cls(candidates, serial_module, address, rate_hz, response_timeout_ms)
        session.start()
        return session

    def start(self) -> None:
        try:
            for reader in self.readers:
                reader.start()
                self._started.append(reader)
            deadline = time.monotonic() + 10.0
            for reader in self.readers:
                remaining = deadline - time.monotonic()
                if remaining > 0:
                    reader.ready_event.wait(remaining)
            failures = [
                reader
                for reader in self.readers
                if reader.error is not None or reader.grid is None
            ]
            if failures:
                detail = "; ".join(
                    "{}={}".format(
                        reader.candidate.device,
                        reader.error or "初始化超时",
                    )
                    for reader in failures
                )
                raise PairingError("触觉初始化失败: {}".format(detail))
            first_grid = self.readers[0].grid
            second_grid = self.readers[1].grid
            assert first_grid is not None and second_grid is not None
            if first_grid.row_masks != second_grid.row_masks:
                raise PairingError("两只触觉设备的 wire cellmap 不一致，拒绝配对")
            before_sampling = {
                reader.candidate.label: reader.health_snapshot() for reader in self.readers
            }
            self.start_event.set()
            readiness_seconds = 1.0
            minimum_ready = max(2, int(self.rate_hz * readiness_seconds * 0.80))
            sample_deadline = time.monotonic() + max(1.5, readiness_seconds * 1.5)
            while time.monotonic() < sample_deadline:
                if any(reader.error for reader in self.readers):
                    break
                if all(
                    reader.health_snapshot().valid_frames
                    - before_sampling[reader.candidate.label].valid_frames
                    >= minimum_ready
                    for reader in self.readers
                ):
                    readiness_issues: List[str] = []
                    for reader in self.readers:
                        label = reader.candidate.label
                        delta = reader.health_snapshot().subtract(before_sampling[label])
                        for field_name in HEALTH_ERROR_FIELDS:
                            value = getattr(delta, field_name)
                            if value:
                                readiness_issues.append(
                                    "{} {}={}".format(label, field_name, value)
                                )
                    if readiness_issues:
                        raise PairingError(
                            "触觉初始化健康检查失败: " + "; ".join(readiness_issues)
                        )
                    return
                time.sleep(0.02)
            detail = "; ".join(
                "{} valid={} error={}".format(
                    reader.candidate.device,
                    reader.health_snapshot().valid_frames,
                    reader.error or "-",
                )
                for reader in self.readers
            )
            raise PairingError("触觉持续采样未就绪: {}".format(detail))
        except BaseException:
            try:
                self.close()
            except BaseException:
                pass
            raise

    def close(self) -> Tuple[str, ...]:
        with self._close_lock:
            if self._closed:
                return self._close_failures
            if self._closing:
                return tuple(reader.candidate.device for reader in self._started if reader.is_alive())
            self._closing = True

        interrupted: Optional[BaseException] = None
        failures: Tuple[str, ...] = ()
        self.stop_event.set()
        self.start_event.set()
        for reader in self.readers:
            reader.notify_subscribers()
        try:
            for reader in self._started:
                try:
                    reader.join(5.0)
                except BaseException as exc:
                    if interrupted is None:
                        interrupted = exc
                    break
        finally:
            running = [reader for reader in self._started if reader.is_alive()]
            for reader in running:
                reader.interrupt_io()
            for reader in running:
                try:
                    reader.join(2.0)
                except BaseException as exc:
                    if interrupted is None:
                        interrupted = exc
            failures = tuple(
                reader.candidate.device for reader in self._started if reader.is_alive()
            )
            with self._close_lock:
                self._close_failures = failures
                self._closed = True
                self._closing = False
        if interrupted is not None:
            raise interrupted
        return failures

    def __enter__(self) -> "PairingSession":
        return self

    def __exit__(self, exc_type: Any, _exc: Any, _traceback: Any) -> None:
        try:
            failures = self.close()
        except BaseException:
            if exc_type is None:
                raise
            return
        if failures and exc_type is None:
            raise PairingError("触觉线程未退出: {}".format(", ".join(failures)))

    def capture_window(
        self,
        seconds: float,
        capture_ratio: float = 0.80,
    ) -> WindowCapture:
        if not math.isfinite(seconds) or seconds <= 0 or seconds > 60:
            raise PairingError("capture seconds 必须在 (0,60]")
        if not math.isfinite(capture_ratio) or capture_ratio <= 0 or capture_ratio > 1:
            raise PairingError("capture_ratio 必须在 (0,1]")
        if seconds * self.rate_hz > MAX_CAPTURE_FRAMES:
            raise PairingError(
                "采样窗口请求帧数超过安全上限 {}".format(MAX_CAPTURE_FRAMES)
            )
        self.assert_active()
        before = {reader.candidate.label: reader.health_snapshot() for reader in self.readers}
        token = uuid.uuid4().hex
        registered: List[TactileReader] = []
        try:
            for reader in self.readers:
                reader.begin_capture(token)
                registered.append(reader)
            start_ns = time.monotonic_ns()
            stopped = self.stop_event.wait(seconds)
            end_ns = time.monotonic_ns()
            frames = {
                reader.candidate.label: reader.end_capture(token, start_ns, end_ns)
                for reader in registered
            }
            registered = []
        finally:
            for reader in registered:
                reader.cancel_capture(token)
        if stopped:
            errors = [reader.error for reader in self.readers if reader.error]
            raise PairingStopped("采样窗口中止: {}".format("; ".join(errors) or "stop"))
        after = {reader.candidate.label: reader.health_snapshot() for reader in self.readers}
        health_delta = {
            label: after[label].subtract(before[label]) for label in before
        }
        capture = WindowCapture(
            start_mono_ns=start_ns,
            end_mono_ns=end_ns,
            duration_s=(end_ns - start_ns) / 1_000_000_000.0,
            frames=frames,
            health_delta=health_delta,
        )
        self._validate_capture_health(capture, capture_ratio)
        return capture

    def _validate_capture_health(self, capture: WindowCapture, capture_ratio: float) -> None:
        minimum = max(1, int(capture.duration_s * self.rate_hz * capture_ratio))
        issues: List[str] = []
        for reader in self.readers:
            label = reader.candidate.label
            frame_count = len(capture.frames[label])
            delta = capture.health_delta[label]
            if frame_count < minimum:
                issues.append("{} frames={} < {}".format(label, frame_count, minimum))
            for field_name in HEALTH_ERROR_FIELDS:
                value = getattr(delta, field_name)
                if value:
                    issues.append("{} {}={}".format(label, field_name, value))
            if reader.error:
                issues.append("{} error={}".format(label, reader.error))
        if issues:
            raise PairingRejected("unhealthy", "采样窗口不健康: " + "; ".join(issues))

    def reader_by_label(self, label: str) -> TactileReader:
        matches = [reader for reader in self.readers if reader.candidate.label == label]
        if len(matches) != 1:
            raise PairingError("未知 tactile candidate: {}".format(label))
        return matches[0]

    def assert_active(self, max_frame_age_ms: float = 500.0) -> None:
        if not math.isfinite(max_frame_age_ms) or max_frame_age_ms <= 0:
            raise PairingError("max_frame_age_ms must be a finite positive number")
        if self._closed or self._closing or self.stop_event.is_set():
            raise PairingError("PairingSession 已关闭或正在停止")
        issues: List[str] = []
        for reader in self.readers:
            if not reader.is_alive():
                issues.append("{} thread_not_alive".format(reader.candidate.label))
            if reader.error:
                issues.append("{} error={}".format(reader.candidate.label, reader.error))
            age_ms = reader.latest_frame_age_ms()
            if age_ms > max_frame_age_ms:
                issues.append(
                    "{} latest_frame_age_ms={:.1f}".format(reader.candidate.label, age_ms)
                )
        if issues:
            raise PairingError("触觉 session 不再有效: " + "; ".join(issues))

    def reader_for_side(self, result: PairingResult, side: str) -> TactileReader:
        normalized = str(side).lower()
        binding = next((item for item in result.bindings if item.side == normalized), None)
        if binding is None:
            raise PairingError("配对结果中没有 side={}".format(side))
        reader = self.reader_by_label(binding.candidate_label)
        if reader.stream_id != binding.stream_id:
            raise PairingError("stream_id 已失效；禁止用旧配对结果重开端口")
        self.assert_active()
        return reader

    def subscribe_paired(self, result: PairingResult) -> PairedSubscriptions:
        """在一个共同 monotonic 边界复用当前左右 stream，不重开串口。"""
        self.assert_active()
        bindings = _strict_bindings_by_side(result)
        readers: Dict[str, TactileReader] = {}
        for side in SIDE_NAMES:
            reader = self.reader_by_label(bindings[side].candidate_label)
            if reader.stream_id != bindings[side].stream_id:
                raise PairingError("stream_id 已失效；禁止用旧配对结果重开端口")
            readers[side] = reader
        if readers["left"] is readers["right"]:
            raise PairingError("左右订阅不能复用同一个 tactile reader")

        # 先取健康基线。基线之后、共同边界之前发生的错误也会在首次读取时被拒绝。
        baseline_health = {
            side: readers[side].health_snapshot() for side in SIDE_NAMES
        }
        ordered_readers = sorted(
            readers.values(), key=lambda item: item.candidate.label
        )
        acquired: List[TactileReader] = []
        try:
            for reader in ordered_readers:
                reader._samples_changed.acquire()
                acquired.append(reader)
            if self.stop_event.is_set() or self._closing or self._closed:
                raise PairingStopped("PairingSession 正在停止")
            start_mono_ns = time.monotonic_ns()
            start_wall_ns = time.time_ns()
            next_sequences = {
                side: readers[side]._next_stream_seq for side in SIDE_NAMES
            }
        finally:
            for reader in reversed(acquired):
                reader._samples_changed.release()

        subscriptions: Dict[str, FrameSubscription] = {}
        try:
            for side in SIDE_NAMES:
                subscriptions[side] = FrameSubscription(
                    reader=readers[side],
                    side=side,
                    start_mono_ns=start_mono_ns,
                    start_wall_ns=start_wall_ns,
                    next_sequence=next_sequences[side],
                    baseline_health=baseline_health[side],
                )
            self.assert_active()
        except BaseException:
            for subscription in subscriptions.values():
                subscription.close()
            raise

        return PairedSubscriptions(
            start_mono_ns=start_mono_ns,
            start_wall_ns=start_wall_ns,
            by_side=MappingProxyType(dict(subscriptions)),
        )


def _percentile(values: Sequence[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(float(value) for value in values)
    index = max(0, min(len(ordered) - 1, int(math.ceil(fraction * len(ordered))) - 1))
    return ordered[index]


def _frame_activity(
    values: Sequence[int],
    centers: Sequence[float],
    noises: Sequence[float],
    scoring: ScoringConfig,
) -> Tuple[float, int]:
    excesses = [
        max(0.0, abs(float(value) - center) - scoring.sigma_multiplier * noise)
        for value, center, noise in zip(values, centers, noises)
    ]
    excesses.sort(reverse=True)
    top = excesses[: scoring.top_k]
    return sum(top), sum(1 for value in excesses if value > 0)


def score_activity(
    requested_side: str,
    reader: TactileReader,
    baseline_frames: Sequence[TimedFsrFrame],
    press_frames: Sequence[TimedFsrFrame],
    scoring: ScoringConfig,
) -> ActivityEvidence:
    if not baseline_frames or not press_frames:
        raise PairingRejected("no_response", "基线或按压窗口没有帧")
    width = len(baseline_frames[0].wire_values)
    if width < 1 or any(len(frame.wire_values) != width for frame in baseline_frames):
        raise PairingRejected("unhealthy", "基线帧通道数不一致")
    if any(len(frame.wire_values) != width for frame in press_frames):
        raise PairingRejected("unhealthy", "按压帧通道数不一致")

    centers: List[float] = []
    noises: List[float] = []
    for index in range(width):
        channel = [float(frame.wire_values[index]) for frame in baseline_frames]
        center = float(statistics.median(channel))
        mad = float(statistics.median(abs(value - center) for value in channel))
        centers.append(center)
        noises.append(max(scoring.noise_floor, 1.4826 * mad))

    baseline_metrics = [
        _frame_activity(frame.wire_values, centers, noises, scoring)
        for frame in baseline_frames
    ]
    press_metrics = [
        _frame_activity(frame.wire_values, centers, noises, scoring)
        for frame in press_frames
    ]
    baseline_score = _percentile([item[0] for item in baseline_metrics], 0.99)
    raw_press_score = _percentile([item[0] for item in press_metrics], 0.90)
    score = max(0.0, raw_press_score - baseline_score)
    frame_threshold = baseline_score + scoring.min_frame_score
    active_frame_count = sum(1 for item in press_metrics if item[0] >= frame_threshold)
    peak_active_cells = max((item[1] for item in press_metrics), default=0)
    return ActivityEvidence(
        requested_side=requested_side,
        candidate_label=reader.candidate.label,
        port=reader.candidate.device,
        baseline_frames=len(baseline_frames),
        press_frames=len(press_frames),
        baseline_score=baseline_score,
        raw_press_score=raw_press_score,
        score=score,
        active_frame_count=active_frame_count,
        peak_active_cells=peak_active_cells,
    )


def select_activity_winner(
    requested_side: str,
    evidence: Sequence[ActivityEvidence],
    scoring: ScoringConfig,
) -> str:
    if len(evidence) != 2:
        raise PairingRejected("unhealthy", "当前只支持两路证据", evidence)
    ordered = sorted(evidence, key=lambda item: item.score, reverse=True)
    winner, loser = ordered
    minimum_active_frames = max(
        scoring.min_active_frames,
        int(math.ceil(winner.press_frames * scoring.min_active_ratio)),
    )
    if winner.score < scoring.min_winner_score:
        raise PairingRejected(
            "weak_signal",
            "{} 按压信号太弱: {:.1f} < {:.1f}".format(
                requested_side, winner.score, scoring.min_winner_score
            ),
            evidence,
        )
    if winner.active_frame_count < minimum_active_frames:
        raise PairingRejected(
            "weak_signal",
            "{} 显著活动帧不足: {} < {}".format(
                requested_side, winner.active_frame_count, minimum_active_frames
            ),
            evidence,
        )
    if winner.peak_active_cells < scoring.min_peak_active_cells:
        raise PairingRejected(
            "weak_signal",
            "{} 显著活动通道不足".format(requested_side),
            evidence,
        )
    if loser.score >= scoring.max_non_target_score:
        raise PairingRejected(
            "non_target_activity",
            "{} 识别时另一只设备也有明显活动: {:.1f} >= {:.1f}".format(
                requested_side, loser.score, scoring.max_non_target_score
            ),
            evidence,
        )
    margin = winner.score - loser.score
    ratio = float("inf") if loser.score <= 0 else winner.score / loser.score
    if margin < scoring.min_margin or ratio < scoring.min_ratio:
        raise PairingRejected(
            "ambiguous",
            "{} 两路活动不够可分: winner={:.1f}, loser={:.1f}, ratio={:.2f}".format(
                requested_side, winner.score, loser.score, ratio
            ),
            evidence,
        )
    return winner.candidate_label


def _validate_binding_profiles(
    session: PairingSession,
    labels: Mapping[str, str],
    config: PairingConfig,
) -> None:
    for side in SIDE_NAMES:
        reader = session.reader_by_label(labels[side])
        assert reader.grid is not None
        profile = config.profiles[config.side_profiles[side]]
        expected_grid = profile["wire_grid"]
        actual_hash = grid_cellmap_sha256(reader.grid)
        if (
            reader.grid.rows != expected_grid["rows"]
            or reader.grid.cols != expected_grid["cols"]
            or reader.grid.active_count != expected_grid["active_count"]
            or actual_hash != expected_grid["cellmap_sha256"]
        ):
            raise PairingRejected(
                "grid_mismatch",
                "{} 触觉 grid/profile 不符: {}x{} active={} sha256={}".format(
                    side,
                    reader.grid.rows,
                    reader.grid.cols,
                    reader.grid.active_count,
                    actual_hash,
                ),
            )
        invalid_cells = profile["post_decode_invalid_cells"]
        if profile["mask_verified"]:
            active = set(reader.grid.active_coordinates)
            missing = [tuple(cell) for cell in invalid_cells if tuple(cell) not in active]
            if missing:
                raise PairingRejected(
                    "grid_mismatch",
                    "{} profile 无效点不在 wire-active map 中: {}".format(side, missing),
                )


def _build_result(
    method: str,
    session: PairingSession,
    labels: Mapping[str, str],
    config: PairingConfig,
    manus: ObservedManus,
    evidence: Sequence[ActivityEvidence],
    scoring: Optional[ScoringConfig] = None,
) -> PairingResult:
    session.assert_active()
    if labels["left"] == labels["right"]:
        raise PairingRejected("same_device", "左右手不能绑定同一个触觉 stream", evidence)
    _validate_binding_profiles(session, labels, config)
    bindings: List[SideBinding] = []
    for side in SIDE_NAMES:
        reader = session.reader_by_label(labels[side])
        profile_name = config.side_profiles[side]
        bindings.append(
            SideBinding(
                side=side,
                stream_id=reader.stream_id,
                candidate_label=reader.candidate.label,
                port_at_pairing=reader.candidate.device,
                manus_glove_id=config.manus_ids[side],
                physical_profile=profile_name,
                mask_verified=bool(config.profiles[profile_name]["mask_verified"]),
            )
        )
    session.assert_active()
    return PairingResult(
        method=method,
        paired_wall_ns=time.time_ns(),
        config_sha256=config.sha256,
        manus_source=manus.source,
        manus_observed_wall_ns=manus.observed_wall_ns,
        manus_live_snapshot=manus.live_snapshot,
        bindings=(bindings[0], bindings[1]),
        evidence=tuple(evidence),
        scoring=scoring,
    )


def pair_by_press(
    session: PairingSession,
    config: PairingConfig,
    manus: ObservedManus,
    prompt_callback: Callable[[str, str], None],
    baseline_seconds: float = 2.0,
    press_seconds: float = 4.0,
    scoring: ScoringConfig = ScoringConfig(),
) -> PairingResult:
    validate_manus_against_config(manus, config)
    if len(session.readers) != 2:
        raise PairingRejected("unhealthy", "排除法配对要求恰好两只触觉设备")
    candidate_labels = {reader.candidate.label for reader in session.readers}
    if len(candidate_labels) != 2:
        raise PairingRejected("unhealthy", "两只触觉设备的 candidate 标签必须唯一")

    prompt_callback(
        "pairing_baseline",
        "请松开两只触觉指套并保持静止；即将采集 {:.1f}s 基线。".format(
            baseline_seconds
        ),
    )
    baseline = session.capture_window(baseline_seconds, scoring.capture_ratio)
    prompt_callback(
        "left_press",
        "倒计时结束后，按住左手任意一个或多个触觉传感区域并持续到窗口结束；"
        "右手保持不动。即将采集 {:.1f}s。".format(press_seconds),
    )
    pressed = session.capture_window(press_seconds, scoring.capture_ratio)
    evidence = [
        score_activity(
            "left",
            reader,
            baseline.frames[reader.candidate.label],
            pressed.frames[reader.candidate.label],
            scoring,
        )
        for reader in session.readers
    ]
    left_label = select_activity_winner("left", evidence, scoring)
    remaining = candidate_labels - {left_label}
    if len(remaining) != 1:
        raise PairingRejected(
            "ambiguous",
            "无法从恰好两只设备中唯一推出右手",
            evidence,
        )
    selected = {"left": left_label, "right": next(iter(remaining))}
    # 配对结论生成前再验证一次双路新鲜健康窗口；后续正式采集仍须持续监控。
    session.capture_window(0.5, scoring.capture_ratio)
    return _build_result(
        MANUAL_PAIRING_RESULT_METHOD,
        session,
        selected,
        config,
        manus,
        evidence,
        scoring=scoring,
    )


def pair_explicit(
    session: PairingSession,
    config: PairingConfig,
    manus: ObservedManus,
    left_port: str,
    right_port: str,
    confirm_callback: Callable[[str], bool],
) -> PairingResult:
    validate_manus_against_config(manus, config)
    by_port = {reader.candidate.device.lower(): reader for reader in session.readers}
    left = by_port.get(str(left_port).lower())
    right = by_port.get(str(right_port).lower())
    if left is None or right is None:
        raise PairingRejected(
            "unknown_port",
            "显式端口必须来自当前已验证的两只设备: {}".format(
                ", ".join(reader.candidate.device for reader in session.readers)
            ),
        )
    if left is right:
        raise PairingRejected("same_device", "左右显式端口不能相同")
    # 用户确认前先证明两路当前仍在连续返回干净数据。
    session.capture_window(0.5)
    message = (
        "本次会话显式绑定：{} -> LEFT (MANUS {}), {} -> RIGHT (MANUS {})。"
        "此映射在进程退出/重插后失效。"
    ).format(
        left.candidate.device,
        config.manus_ids["left"],
        right.candidate.device,
        config.manus_ids["right"],
    )
    if not confirm_callback(message):
        raise PairingRejected("not_confirmed", "用户未确认显式左右绑定")
    # 用户可能在阅读/确认期间拔掉设备，因此确认后必须重新取健康窗口。
    session.capture_window(0.5)
    labels = {
        "left": left.candidate.label,
        "right": right.candidate.label,
    }
    return _build_result("explicit", session, labels, config, manus, ())


def _validate_manual_pairing_result(result: PairingResult) -> Dict[str, SideBinding]:
    """验证“左手正向识别、右手排除绑定”的完整会话证据。"""
    if result.method != MANUAL_PAIRING_RESULT_METHOD or len(result.evidence) != 2:
        raise PairingError("配对结果必须来自本次启动的左手按压与右手排除绑定")
    if not isinstance(result.scoring, ScoringConfig):
        raise PairingError("手动配对结果缺少可复验的评分参数")

    bindings = _strict_bindings_by_side(result)
    expected_labels = {
        bindings["left"].candidate_label,
        bindings["right"].candidate_label,
    }
    actual_pairs = [
        (item.requested_side, item.candidate_label) for item in result.evidence
    ]
    expected_pairs = {("left", label) for label in expected_labels}
    if len(set(actual_pairs)) != 2 or set(actual_pairs) != expected_pairs:
        raise PairingError("左手按压证据必须且只能覆盖两路 candidate 各一次")

    evidence_by_label = {item.candidate_label: item for item in result.evidence}
    for side in SIDE_NAMES:
        binding = bindings[side]
        evidence = evidence_by_label[binding.candidate_label]
        if evidence.port != binding.port_at_pairing:
            raise PairingError("{} 证据端口与 binding 不一致".format(side))

    try:
        winner = select_activity_winner("left", result.evidence, result.scoring)
    except PairingRejected as exc:
        raise PairingError("左手按压证据复验失败: {}".format(exc)) from exc
    if winner != bindings["left"].candidate_label:
        raise PairingError("左手 binding 与活动证据胜者不一致")
    remaining = expected_labels - {winner}
    if remaining != {bindings["right"].candidate_label}:
        raise PairingError("右手 binding 不是两路候选中的唯一剩余设备")
    return bindings


def require_raw_capture_ready(
    session: PairingSession,
    result: PairingResult,
    config: PairingConfig,
) -> None:
    """原始触觉采集门禁；不要求物理 mask 验证或 MANUS live lease。"""
    if result.config_sha256 != config.sha256:
        raise PairingError("配对结果使用的配置版本已经变化")
    bindings = _validate_manual_pairing_result(result)

    labels: Dict[str, str] = {}
    for side in SIDE_NAMES:
        binding = bindings[side]
        if binding.physical_profile != config.side_profiles[side]:
            raise PairingError("{} physical_profile 与当前配置不一致".format(side))
        if binding.manus_glove_id != config.manus_ids[side]:
            raise PairingError("{} 静态 MANUS glove_id 与当前配置不一致".format(side))
        reader = session.reader_for_side(result, side)
        labels[side] = reader.candidate.label

    # 这里只核对 wire grid/profile；不会读取或要求 mask_verified=True。
    _validate_binding_profiles(session, labels, config)
    session.assert_active()
    session.capture_window(0.5)
    session.assert_active()


def require_recording_ready(
    session: PairingSession,
    result: PairingResult,
    config: PairingConfig,
    manus_status: ObservedManus,
    max_manus_age_ms: float = 500.0,
    max_tactile_age_ms: float = 500.0,
) -> None:
    """正式 START 前的唯一强制门禁；失败时抛错，不返回可误用的布尔值。"""
    if result.config_sha256 != config.sha256:
        raise PairingError("配对结果使用的配置版本已经变化")
    _validate_manual_pairing_result(result)
    if not all(binding.mask_verified for binding in result.bindings):
        raise PairingError("4x7 后解码逻辑掩码尚未完成按压验证")
    if not manus_status.continuous_lease or not manus_status.service_instance_id:
        raise PairingError("缺少常驻 MANUS 服务提供的持续身份租约")
    if manus_status.observed_wall_ns <= 0:
        raise PairingError("MANUS 身份租约没有有效观察时间")
    if not math.isfinite(max_manus_age_ms) or max_manus_age_ms <= 0:
        raise PairingError("max_manus_age_ms 必须 > 0")
    if not math.isfinite(max_tactile_age_ms) or max_tactile_age_ms <= 0:
        raise PairingError("max_tactile_age_ms must be a finite positive number")
    now_wall_ns = time.time_ns()
    if manus_status.observed_wall_ns > now_wall_ns:
        raise PairingError("MANUS identity lease timestamp is in the future")
    age_ms = (now_wall_ns - manus_status.observed_wall_ns) / 1_000_000.0
    if age_ms > max_manus_age_ms:
        raise PairingError(
            "MANUS 身份租约过期: {:.1f}ms > {:.1f}ms".format(age_ms, max_manus_age_ms)
        )
    validate_manus_against_config(manus_status, config)
    for binding in result.bindings:
        if manus_status.identities.get(binding.side) != binding.manus_glove_id:
            raise PairingError("MANUS 身份与本次触觉绑定不一致")
        session.reader_for_side(result, binding.side)
    session.assert_active(max_frame_age_ms=max_tactile_age_ms)
    session.capture_window(0.5)


def pairing_result_to_dict(result: PairingResult) -> Dict[str, Any]:
    bindings: Dict[str, Any] = {}
    for binding in result.bindings:
        if result.method == MANUAL_PAIRING_RESULT_METHOD:
            assignment_basis = (
                "activity_verified" if binding.side == "left"
                else "remaining_candidate_by_exclusion"
            )
        elif result.method == "explicit":
            assignment_basis = "explicit_user_confirmed"
        else:
            assignment_basis = "unknown"
        bindings[binding.side] = {
            "tactile_stream_id": binding.stream_id,
            "candidate_label": binding.candidate_label,
            "observed_transport": {
                "port": binding.port_at_pairing,
                "identity": False,
            },
            "manus_glove_id": str(binding.manus_glove_id),
            "physical_profile": binding.physical_profile,
            "mask_verified": binding.mask_verified,
            "assignment_basis": assignment_basis,
        }
    evidence = [
        {
            "requested_side": item.requested_side,
            "candidate_label": item.candidate_label,
            "port": item.port,
            "baseline_frames": item.baseline_frames,
            "press_frames": item.press_frames,
            "baseline_score": round(item.baseline_score, 3),
            "raw_press_score": round(item.raw_press_score, 3),
            "score": round(item.score, 3),
            "active_frame_count": item.active_frame_count,
            "peak_active_cells": item.peak_active_cells,
        }
        for item in result.evidence
    ]
    masks_verified = all(binding.mask_verified for binding in result.bindings)
    blockers = [
        "standalone_result_expires_when_pairing_session_closes",
        "continuous_manus_identity_lease_required",
    ]
    if result.method != MANUAL_PAIRING_RESULT_METHOD:
        blockers.append("explicit_mode_is_a_maintenance_policy_override")
    if not masks_verified:
        blockers.append("physical_4x7_mask_not_verified")
    return {
        "schema": "pico_tactile_session_binding_v1",
        "method": result.method,
        "paired_wall_ns": result.paired_wall_ns,
        "config_sha256": result.config_sha256,
        "manus_source": result.manus_source,
        "manus_observed_wall_ns": result.manus_observed_wall_ns,
        "manus_live_snapshot": result.manus_live_snapshot,
        "bindings": bindings,
        "assignment_policy": {
            "positive_identified_side": "left",
            "left": "activity_verified",
            "right": "remaining_candidate_by_exclusion",
            "candidate_count": 2,
        } if result.method == MANUAL_PAIRING_RESULT_METHOD else None,
        "evidence": evidence,
        "validation_pass": True,
        "policy_override": result.method == "explicit",
        "recording_ready": False,
        "recording_blockers": blockers,
        "valid_until": "current_process_and_open_serial_handles_only",
    }


def _resolve_manus(args: argparse.Namespace, config: PairingConfig) -> ObservedManus:
    if args.manus_ndjson:
        observed = discover_manus_from_ndjson(
            Path(args.manus_ndjson), max_frames=args.manus_max_frames
        )
    elif args.manus_bridge_snapshot:
        observed = discover_manus_snapshot_from_bridge(
            Path(args.manus_bridge_snapshot), timeout_s=args.manus_timeout
        )
    else:
        observed = configured_manus(config)
    validate_manus_against_config(observed, config)
    return observed


def _read_console_line(prompt: str, stream: Any) -> str:
    print(prompt, end="", file=stream, flush=True)
    line = sys.stdin.readline()
    if line == "":
        raise PairingError("交互输入已关闭")
    return line.rstrip("\r\n")


def _console_prompt(phase: str, message: str, stream: Any = sys.stdout) -> None:
    print("\n[{}] {}".format(phase, message), file=stream, flush=True)
    _read_console_line("准备好后按 Enter；看到“开始采集”后执行上述动作: ", stream)
    if phase.endswith("_press"):
        for value in (3, 2, 1):
            print("  {}...".format(value), file=stream, flush=True)
            time.sleep(1.0)
    print("  开始采集。", file=stream, flush=True)


def _console_confirm(message: str, stream: Any = sys.stdout) -> bool:
    print("\n" + message, file=stream)
    answer = _read_console_line("如确认无误，请输入 BIND（无默认值）: ", stream)
    return answer.strip() == "BIND"


def _print_evidence(evidence: Sequence[ActivityEvidence], file: Any = None) -> None:
    stream = file or sys.stdout
    for item in evidence:
        print(
            "  side={} {}({}) score={:.1f} baseline={:.1f} press={:.1f} active_frames={} peak_cells={}".format(
                item.requested_side,
                item.candidate_label,
                item.port,
                item.score,
                item.baseline_score,
                item.raw_press_score,
                item.active_frame_count,
                item.peak_active_cells,
            ),
            file=stream,
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="触觉设备与 MANUS 左右手的会话级手动配对（不以COM口作为身份）"
    )
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH), help="持久预期配置 JSON")
    parser.add_argument("--check-config", action="store_true", help="校验配置；可选比对历史/诊断快照，不打开触觉串口")
    parser.add_argument("--list", action="store_true", help="只列出触觉串口，不打开设备")
    manuscript = parser.add_mutually_exclusive_group()
    manuscript.add_argument("--manus-ndjson", help="从已有 MANUS NDJSON 验证 glove_id/side")
    manuscript.add_argument(
        "--manus-bridge-snapshot",
        help="诊断用途：短暂启动指定MANUS bridge取身份快照；不能作为录制租约",
    )
    parser.add_argument("--manus-timeout", type=float, default=15.0, help="bridge快照发现超时秒数")
    parser.add_argument("--manus-max-frames", type=int, default=2000, help="NDJSON最多检查的manus_frame数")
    parser.add_argument("--mode", choices=("press", "explicit"), default="press")
    parser.add_argument("--port", action="append", default=[], help="显式候选串口，可重复")
    parser.add_argument("--left-port", help="explicit 模式的本次左手串口")
    parser.add_argument("--right-port", help="explicit 模式的本次右手串口")
    parser.add_argument("--address", type=parse_hex_byte, default=DEFAULT_ADDRESS, help="只读查询地址，默认0A")
    parser.add_argument("--rate-hz", type=float, default=60.0, help="每路采样请求频率，默认60")
    parser.add_argument("--response-timeout-ms", type=int, default=200, help="单次响应超时，默认200")
    parser.add_argument("--baseline-seconds", type=float, default=2.0, help="双手松开基线窗口，默认2秒")
    parser.add_argument("--press-seconds", type=float, default=4.0, help="左手按压识别窗口，默认4秒")
    parser.add_argument("--json", action="store_true", help="成功时只输出会话绑定JSON")
    return parser


def _validate_cli_args(args: argparse.Namespace) -> None:
    if not math.isfinite(args.manus_timeout) or args.manus_timeout <= 0:
        raise PairingError("--manus-timeout 必须 > 0")
    if args.manus_max_frames < 2 or args.manus_max_frames > 1_000_000:
        raise PairingError("--manus-max-frames 必须在 2..1000000")
    if not math.isfinite(args.rate_hz) or args.rate_hz <= 0 or args.rate_hz > 100:
        raise PairingError("--rate-hz 必须在 (0,100]")
    if args.response_timeout_ms < 10 or args.response_timeout_ms > 5000:
        raise PairingError("--response-timeout-ms 必须在 10..5000")
    for name in ("baseline_seconds", "press_seconds"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0 or value > 60:
            raise PairingError("--{} 必须在 (0,60]".format(name.replace("_", "-")))
    if args.list or args.check_config:
        return
    if args.mode == "explicit" and (not args.left_port or not args.right_port):
        raise PairingError("explicit 模式必须同时提供 --left-port 和 --right-port")
    if args.mode == "press" and (args.left_port or args.right_port):
        raise PairingError("press 模式不能提供 --left-port/--right-port")


def run_cli(args: argparse.Namespace) -> int:
    _validate_cli_args(args)
    if args.list:
        serial_module, list_ports = _load_pyserial()
        candidates, all_ports = discover_candidates(list_ports, args.port)
        print_port_table(all_ports, "全部串口：")
        print_port_table(candidates, "触觉候选：")
        return 0

    config = load_pairing_config(Path(args.config))
    manus = _resolve_manus(args, config)
    if args.check_config:
        print(
            "CONFIG VALID schema={} sha256={} manus_left={} manus_right={} observation={} live_snapshot={}".format(
                CONFIG_SCHEMA,
                config.sha256,
                manus.identities["left"],
                manus.identities["right"],
                manus.source,
                manus.live_snapshot,
            )
        )
        for name, profile in config.profiles.items():
            print(
                "  profile={} mask_verified={} cellmap_sha256={}".format(
                    name,
                    profile["mask_verified"],
                    profile["wire_grid"]["cellmap_sha256"],
                )
            )
        return 0

    serial_module, list_ports = _load_pyserial()
    candidates, all_ports = discover_candidates(list_ports, args.port)
    if len(candidates) != 2:
        print_port_table(all_ports, "当前串口：")
        raise PairingError("触觉候选必须恰好为2，实际 {}".format(len(candidates)))

    if not args.json:
        print("MANUS: left={} right={} source={} live_snapshot={}".format(
            manus.identities["left"], manus.identities["right"], manus.source, manus.live_snapshot
        ))
        print("触觉 candidate 仅在本次进程有效：")
        print_port_table(candidates, "准备打开：")
        if manus.live_snapshot:
            print("警告：MANUS仅为短时诊断快照，不是常驻身份租约；正式录制门禁仍为关闭。")
        else:
            print("警告：本次只验证配置/历史MANUS身份；正式录制门禁仍为关闭。")

    with PairingSession.open(
        candidates,
        serial_module,
        address=args.address,
        rate_hz=args.rate_hz,
        response_timeout_ms=args.response_timeout_ms,
    ) as session:
        if not args.json:
            for reader in session.readers:
                assert reader.grid is not None
                print(
                    "[ready] {} {} stream={} grid={}x{} active={} cellmap={}".format(
                        reader.candidate.label,
                        reader.candidate.device,
                        reader.stream_id[:12],
                        reader.grid.rows,
                        reader.grid.cols,
                        reader.grid.active_count,
                        grid_cellmap_sha256(reader.grid)[:12],
                    )
                )
        if args.mode == "press":
            prompt_stream = sys.stderr if args.json else sys.stdout
            result = pair_by_press(
                session,
                config,
                manus,
                lambda phase, message: _console_prompt(phase, message, prompt_stream),
                baseline_seconds=args.baseline_seconds,
                press_seconds=args.press_seconds,
            )
        else:
            confirm_stream = sys.stderr if args.json else sys.stdout
            result = pair_explicit(
                session,
                config,
                manus,
                args.left_port,
                args.right_port,
                lambda message: _console_confirm(message, confirm_stream),
            )
        output = pairing_result_to_dict(result)
        # standalone 必须先确认 session 已干净关闭，再发布 validation_pass。
        # 集成采集器则应在自己的 with/资源生命周期内继续持有 session。
    if args.json:
        print(json.dumps(output, ensure_ascii=False, sort_keys=True))
    else:
        print("\nPAIRING PASS method={}".format(result.method))
        for binding in result.bindings:
            assignment_basis = output["bindings"][binding.side]["assignment_basis"]
            print(
                "  {}: {}({}) stream={} -> MANUS {} profile={} mask_verified={} basis={}".format(
                    binding.side,
                    binding.candidate_label,
                    binding.port_at_pairing,
                    binding.stream_id[:12],
                    binding.manus_glove_id,
                    binding.physical_profile,
                    binding.mask_verified,
                    assignment_basis,
                )
            )
        _print_evidence(result.evidence)
        print("  validation_pass={}".format(output["validation_pass"]))
        print("  recording_ready=False")
        print("  blockers={}".format(",".join(output["recording_blockers"])))
        print("  注意：standalone退出即释放串口；正式录制必须由集成采集器持有本session并通过持续门禁。")
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return run_cli(args)
    except PairingRejected as exc:
        print("[pairing:{}] {}".format(exc.code, exc), file=sys.stderr)
        if exc.evidence:
            _print_evidence(exc.evidence, file=sys.stderr)
        return 1
    except PairingError as exc:
        print("[pairing] {}".format(exc), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("\n[pairing] 用户中止", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
