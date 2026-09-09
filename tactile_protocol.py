#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""超维触觉指套的纯协议编解码。

本模块不打开串口，也不依赖 pyserial。它只负责：

* CRC16-Modbus；
* ``3C 3C ... 3E 3E`` 帧的编码、严格解码和流式重同步；
* 行数、列数、wire-active 位图和 FSR 数据的解析。

设备只上传 wire-active 格点。FSR payload 必须先按设备位图完整展开，之后
才能应用物理有效掩码（例如四指 4x8 裁成 4x7）；物理掩码绝不能参与 wire
解码，否则后续通道会整体错位。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple


BAUD_RATE = 921600
DEFAULT_ADDRESS = 0x0A

FUNC_ROWS = 0x06
FUNC_COLS = 0x07
FUNC_CELLMAP = 0x0C
FUNC_FSR = 0x20
READ_ONLY_FUNCTIONS = frozenset({FUNC_ROWS, FUNC_COLS, FUNC_CELLMAP, FUNC_FSR})

WRAP_HEAD = b"<<"
WRAP_TAIL = b">>"
MAX_PAYLOAD = 4096
MAX_STREAM_BUFFER = 64 * 1024

_INNER_META_BYTES = 4  # address + function + payload_length(uint16 LE)
_CRC_BYTES = 2
_FRAME_OVERHEAD = len(WRAP_HEAD) + _INNER_META_BYTES + _CRC_BYTES + len(WRAP_TAIL)


class TactileProtocolError(ValueError):
    """触觉协议格式或语义错误。"""


class FrameFormatError(TactileProtocolError):
    """完整帧的边界、长度或字段格式错误。"""


class CrcMismatchError(FrameFormatError):
    """完整帧 CRC 与内容不匹配。"""


class ResponseMismatchError(TactileProtocolError):
    """响应地址或功能码不是调用方所期望的值。"""


class PayloadError(TactileProtocolError):
    """响应 payload 的长度或取值不合法。"""


def _byte_value(name: str, value: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError("{} 必须是 0..255 的整数".format(name))
    out = value
    if out < 0 or out > 0xFF:
        raise ValueError("{} 超出 0..255: {}".format(name, out))
    return out


def crc16_modbus(data: bytes) -> int:
    """计算 CRC16-Modbus（初值 0xFFFF，多项式 0xA001）。"""
    crc = 0xFFFF
    for value in bytes(data):
        crc ^= value
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def encode_frame(address: int, function: int, payload: bytes = b"") -> bytes:
    """编码一个带 CRC 和外层定界符的完整命令帧。"""
    addr = _byte_value("address", address)
    func = _byte_value("function", function)
    body = bytes(payload)
    if len(body) > MAX_PAYLOAD:
        raise ValueError("payload 过长: {} > {}".format(len(body), MAX_PAYLOAD))

    inner = bytes((addr, func, len(body) & 0xFF, (len(body) >> 8) & 0xFF)) + body
    crc = crc16_modbus(inner)
    return WRAP_HEAD + inner + bytes((crc & 0xFF, (crc >> 8) & 0xFF)) + WRAP_TAIL


@dataclass(frozen=True)
class Frame:
    """一个已经通过长度、边界和 CRC 校验的完整帧。"""

    address: int
    function: int
    payload: bytes
    crc: int
    raw: bytes


def decode_frame(
    packet: bytes,
    expected_address: Optional[int] = None,
    expected_function: Optional[int] = None,
) -> Frame:
    """严格解码单个完整帧；不接受前置垃圾或尾随字节。"""
    raw = bytes(packet)
    if len(raw) < _FRAME_OVERHEAD:
        raise FrameFormatError("帧过短: {} < {}".format(len(raw), _FRAME_OVERHEAD))
    if raw[:2] != WRAP_HEAD:
        raise FrameFormatError("帧头不是 3C 3C")
    if raw[-2:] != WRAP_TAIL:
        raise FrameFormatError("帧尾不是 3E 3E")

    payload_len = raw[4] | (raw[5] << 8)
    if payload_len > MAX_PAYLOAD:
        raise FrameFormatError("payload 长度超过上限: {}".format(payload_len))
    expected_len = payload_len + _FRAME_OVERHEAD
    if len(raw) != expected_len:
        raise FrameFormatError(
            "整帧长度不符: 声明 payload={}，应为 {} 字节，实际 {} 字节".format(
                payload_len, expected_len, len(raw)
            )
        )

    address = raw[2]
    function = raw[3]
    payload_end = 6 + payload_len
    payload = raw[6:payload_end]
    received_crc = raw[payload_end] | (raw[payload_end + 1] << 8)
    expected_crc = crc16_modbus(raw[2:payload_end])
    if received_crc != expected_crc:
        raise CrcMismatchError(
            "CRC 不符: expected=0x{:04X}, received=0x{:04X}".format(
                expected_crc, received_crc
            )
        )

    if expected_address is not None:
        normalized_address = _byte_value("expected_address", expected_address)
        if address != normalized_address:
            raise ResponseMismatchError(
                "响应地址不符: expected=0x{:02X}, received=0x{:02X}".format(
                    normalized_address, address
                )
            )
    if expected_function is not None:
        normalized_function = _byte_value("expected_function", expected_function)
        if function != normalized_function:
            raise ResponseMismatchError(
                "响应功能码不符: expected=0x{:02X}, received=0x{:02X}".format(
                    normalized_function, function
                )
            )

    return Frame(address=address, function=function, payload=payload, crc=received_crc, raw=raw)


@dataclass
class ParserStats:
    """流式解析器累计统计。"""

    frames: int = 0
    discarded_bytes: int = 0
    length_errors: int = 0
    tail_errors: int = 0
    crc_errors: int = 0
    buffer_overflows: int = 0


class FrameStreamParser:
    """从任意串口 chunk 中恢复完整触觉帧。

    长度、尾标志或 CRC 错误时仅丢弃候选帧头的第一个字节，再搜索下一个
    ``3C 3C``，从而能从错位数据中自愈。对于“长度合法但帧尚未收齐”的数据，
    解析器会等待后续 chunk；调用方在请求超时时应调用 :meth:`reset`。
    """

    def __init__(
        self,
        max_payload: int = MAX_PAYLOAD,
        max_buffer: int = MAX_STREAM_BUFFER,
    ) -> None:
        if isinstance(max_payload, bool) or not isinstance(max_payload, int):
            raise ValueError("max_payload 必须是整数")
        if isinstance(max_buffer, bool) or not isinstance(max_buffer, int):
            raise ValueError("max_buffer 必须是整数")
        if max_payload < 0 or max_payload > MAX_PAYLOAD:
            raise ValueError("max_payload 必须在 0..{}".format(MAX_PAYLOAD))
        if max_buffer < max_payload + _FRAME_OVERHEAD:
            raise ValueError("max_buffer 必须容纳一个最大帧")
        self.max_payload = int(max_payload)
        self.max_buffer = int(max_buffer)
        self.stats = ParserStats()
        self._buffer = bytearray()

    @property
    def buffered_bytes(self) -> int:
        return len(self._buffer)

    def reset(self) -> None:
        """丢弃尚未组成完整帧的缓存；累计错误统计保留。"""
        if self._buffer:
            self.stats.discarded_bytes += len(self._buffer)
            self._buffer.clear()

    def clear(self) -> None:
        """同时清空缓存和累计统计。"""
        self._buffer.clear()
        self.stats = ParserStats()

    def _discard_prefix(self, count: int) -> None:
        if count <= 0:
            return
        count = min(int(count), len(self._buffer))
        del self._buffer[:count]
        self.stats.discarded_bytes += count

    def _bound_buffer(self) -> None:
        if len(self._buffer) <= self.max_buffer:
            return
        drop = len(self._buffer) - self.max_buffer
        self._discard_prefix(drop)
        self.stats.buffer_overflows += 1

    def feed(self, chunk: bytes) -> List[Frame]:
        """加入一段字节并返回本次恢复出的所有完整好帧。"""
        incoming = bytes(chunk)
        if incoming:
            self._buffer.extend(incoming)
            self._bound_buffer()

        out: List[Frame] = []
        while True:
            head_index = self._buffer.find(WRAP_HEAD)
            if head_index < 0:
                # 保留最后一个孤立的 '<'，它可能是下一 chunk 的帧头首字节。
                keep = 1 if self._buffer and self._buffer[-1] == WRAP_HEAD[0] else 0
                self._discard_prefix(len(self._buffer) - keep)
                break
            if head_index > 0:
                self._discard_prefix(head_index)

            if len(self._buffer) < 6:
                break
            payload_len = self._buffer[4] | (self._buffer[5] << 8)
            if payload_len > self.max_payload:
                self.stats.length_errors += 1
                self._discard_prefix(1)
                continue

            frame_len = payload_len + _FRAME_OVERHEAD
            if len(self._buffer) < frame_len:
                break

            candidate = bytes(self._buffer[:frame_len])
            if candidate[-2:] != WRAP_TAIL:
                self.stats.tail_errors += 1
                self._discard_prefix(1)
                continue
            try:
                frame = decode_frame(candidate)
            except CrcMismatchError:
                self.stats.crc_errors += 1
                self._discard_prefix(1)
                continue
            except FrameFormatError:
                self.stats.length_errors += 1
                self._discard_prefix(1)
                continue

            del self._buffer[:frame_len]
            self.stats.frames += 1
            out.append(frame)

        return out


def require_response(frame: Frame, address: int, function: int) -> Frame:
    """检查已经解码的响应地址和功能码。"""
    expected_address = _byte_value("address", address)
    expected_function = _byte_value("function", function)
    if frame.address != expected_address:
        raise ResponseMismatchError(
            "响应地址不符: expected=0x{:02X}, received=0x{:02X}".format(
                expected_address, frame.address
            )
        )
    if frame.function != expected_function:
        raise ResponseMismatchError(
            "响应功能码不符: expected=0x{:02X}, received=0x{:02X}".format(
                expected_function, frame.function
            )
        )
    return frame


def parse_dimension(frame: Frame, address: int, function: int) -> int:
    """解析 0x06/0x07 的单字节行数或列数响应。"""
    if function not in (FUNC_ROWS, FUNC_COLS):
        raise ValueError("dimension 功能码只能是 0x06 或 0x07")
    require_response(frame, address, function)
    if len(frame.payload) != 1:
        raise PayloadError(
            "0x{:02X} payload 应为 1 字节，实际 {}".format(function, len(frame.payload))
        )
    value = frame.payload[0]
    if value < 1 or value > 32:
        raise PayloadError("无效维度 {}，允许范围 1..32".format(value))
    return value


@dataclass(frozen=True)
class GridConfig:
    """设备 wire-active 网格配置。"""

    rows: int
    cols: int
    row_masks: Tuple[int, ...]
    active_mask: Tuple[Tuple[bool, ...], ...]
    active_coordinates: Tuple[Tuple[int, int], ...]

    @property
    def active_count(self) -> int:
        return len(self.active_coordinates)

    @classmethod
    def from_payload(cls, rows: int, cols: int, payload: bytes) -> "GridConfig":
        if isinstance(rows, bool) or not isinstance(rows, int):
            raise PayloadError("rows 必须是整数")
        if isinstance(cols, bool) or not isinstance(cols, int):
            raise PayloadError("cols 必须是整数")
        if rows < 1 or rows > 32 or cols < 1 or cols > 32:
            raise PayloadError("网格维度必须在 1..32，收到 {}x{}".format(rows, cols))
        body = bytes(payload)
        expected = rows * 4
        if len(body) != expected:
            raise PayloadError(
                "cellmap payload 应为 {} 字节，实际 {}".format(expected, len(body))
            )

        row_masks: List[int] = []
        masks: List[Tuple[bool, ...]] = []
        coordinates: List[Tuple[int, int]] = []
        for row in range(rows):
            offset = row * 4
            mask = int.from_bytes(body[offset : offset + 4], byteorder="little", signed=False)
            if mask >> cols:
                raise PayloadError(
                    "cellmap 第 {} 行在 {} 列之外含有非零保留位".format(row, cols)
                )
            row_masks.append(mask)
            row_active: List[bool] = []
            for col in range(cols):
                active = bool(mask & (1 << col))
                row_active.append(active)
                if active:
                    coordinates.append((row, col))
            masks.append(tuple(row_active))

        return cls(
            rows=rows,
            cols=cols,
            row_masks=tuple(row_masks),
            active_mask=tuple(masks),
            active_coordinates=tuple(coordinates),
        )


def parse_grid(frame: Frame, address: int, rows: int, cols: int) -> GridConfig:
    """检查 0x0C 响应并构造 :class:`GridConfig`。"""
    require_response(frame, address, FUNC_CELLMAP)
    return GridConfig.from_payload(rows, cols, frame.payload)


@dataclass(frozen=True)
class FsrSample:
    """一帧按 wire map 严格解出的 signed-int16 FSR 数据。"""

    wire_values: Tuple[int, ...]
    matrix: Tuple[Tuple[int, ...], ...]


def decode_signed_int16_le(payload: bytes) -> Tuple[int, ...]:
    """把偶数字节 payload 严格解析为 signed int16 little-endian。"""
    body = bytes(payload)
    if len(body) % 2:
        raise PayloadError("int16 payload 长度必须为偶数，实际 {}".format(len(body)))
    values: List[int] = []
    for offset in range(0, len(body), 2):
        raw = body[offset] | (body[offset + 1] << 8)
        values.append(raw - 0x10000 if raw >= 0x8000 else raw)
    return tuple(values)


def decode_fsr(frame: Frame, address: int, grid: GridConfig) -> FsrSample:
    """严格解析 0x20 响应，并按 wire-active 行优先顺序展开矩阵。"""
    require_response(frame, address, FUNC_FSR)
    expected = grid.active_count * 2
    if len(frame.payload) != expected:
        raise PayloadError(
            "FSR payload 应为 {} 字节（{} active cells），实际 {}".format(
                expected, grid.active_count, len(frame.payload)
            )
        )
    values = decode_signed_int16_le(frame.payload)
    matrix: List[List[int]] = [[0 for _ in range(grid.cols)] for _ in range(grid.rows)]
    for index, (row, col) in enumerate(grid.active_coordinates):
        matrix[row][col] = values[index]
    return FsrSample(wire_values=values, matrix=tuple(tuple(row) for row in matrix))


def flatten_active_matrix(
    matrix: Sequence[Sequence[int]], grid: GridConfig
) -> Tuple[int, ...]:
    """按同一 wire 顺序从矩阵取回 active 值，主要用于测试和导出校验。"""
    if len(matrix) != grid.rows or any(len(row) != grid.cols for row in matrix):
        raise PayloadError("matrix 尺寸与 grid 不一致")
    return tuple(int(matrix[row][col]) for row, col in grid.active_coordinates)
