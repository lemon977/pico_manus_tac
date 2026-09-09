#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""HS13 实物触觉阵列与 MANUS 手指组对应。

设备仍按公司上位机的 24x16/369-wire 通道格式传输，但当前这副 HS13
实物只有五片指端 4x8 阵列，没有第二组指部阵列，也没有手掌阵列。HTML
显示代码还会把食指、无名指的两个逻辑半区上下互换，因此不能统一选取
REGIONS 中名为 ``*_tip`` 的半区；这里按屏幕上方、并由实采受力位置复核
后的五块区域导出。

输出每指统一为物理方向 ``(4, 8)``：axis0 为横向 4 点，axis1 从指尖朝
指根。拇指 32 点有效；食/中/无名/小指靠近指根的最后一列无信号，分别
为 28 点有效，总物理有效点数 144。
"""
from __future__ import annotations

from typing import Any, Dict, Mapping, Sequence, Tuple


TACTILE_LAYOUT_SCHEMA = "hs13_five_fingertip_4x8_four_4x7_v1"
TACTILE_LAYOUT_SOURCE = (
    "超维数采手套使用说明.pptx hardware photos + HS13_2D.html "
    "displayPos upper arrays + captured pressure verification"
)
TACTILE_GRID_ROWS = 24
TACTILE_GRID_COLS = 16
TACTILE_ACTIVE_COUNT = 369

# 固定为 MANUS chain_types 的顺序，便于模型按同一维度读取两种模态。
TACTILE_FINGER_ORDER = ("thumb", "index", "middle", "ring", "pinky")
TACTILE_PHYSICAL_FINGER_SHAPE = (4, 8)
TACTILE_PHYSICAL_SLOT_COUNT = 5 * 4 * 8
TACTILE_PHYSICAL_ACTIVE_COUNT = 32 + 4 * 28
TACTILE_NON_THUMB_DEAD_COLUMN = 7
TACTILE_PALM_PRESENT = False

# 屏幕上方五片阵列在 wire 24x16 矩阵中的 (row_start,row_stop,col_start,col_stop)。
# 每块 wire 方向为 8x4，导出时转置成物理声明的 4x8。
# HS13_2D.html displayPos() 对 index/ring 做了上下交换，所以二者选用原
# ``*_pad`` 半区；这里的物理含义仍然是这副硬件唯一的指端阵列。
TACTILE_FINGER_REGIONS: Mapping[str, Tuple[int, int, int, int]] = {
    "thumb": (0, 8, 0, 4),
    "index": (16, 24, 4, 8),
    "middle": (8, 16, 0, 4),
    "ring": (8, 16, 12, 16),
    "pinky": (16, 24, 8, 12),
}
TACTILE_VENDOR_REGION_IDS: Mapping[str, str] = {
    "thumb": "thumb_tip",
    "index": "index_pad (displayPos swapped to upper)",
    "middle": "middle_tip",
    "ring": "ring_pad (displayPos swapped to upper)",
    "pinky": "pinky_tip",
}

# 当前项目 MANUS 25 节点布局。这里只建立同一根手指的组级对应。
TACTILE_TO_MANUS_NODE_IDS: Mapping[str, Tuple[int, ...]] = {
    "thumb": (1, 2, 3, 4),
    "index": (5, 6, 7, 8, 9),
    "middle": (10, 11, 12, 13, 14),
    "ring": (15, 16, 17, 18, 19),
    "pinky": (20, 21, 22, 23, 24),
}


def finger_physical_active_mask() -> Tuple[Tuple[Tuple[bool, ...], ...], ...]:
    """返回 (5,4,8) 物理有效点掩码。"""
    masks = []
    for finger in TACTILE_FINGER_ORDER:
        finger_mask = [[True] * TACTILE_PHYSICAL_FINGER_SHAPE[1]
                       for _ in range(TACTILE_PHYSICAL_FINGER_SHAPE[0])]
        if finger != "thumb":
            for row in finger_mask:
                row[TACTILE_NON_THUMB_DEAD_COLUMN] = False
        masks.append(tuple(tuple(row) for row in finger_mask))
    return tuple(masks)


def physical_invalid_grid_coordinates() -> Tuple[Tuple[int, int], ...]:
    """返回四个非拇指失效列在原始 24x16 grid 中的 16 个坐标。"""
    cells = []
    for finger in TACTILE_FINGER_ORDER:
        if finger == "thumb":
            continue
        _r0, r1, c0, c1 = TACTILE_FINGER_REGIONS[finger]
        cells.extend((r1 - 1, col) for col in range(c0, c1))
    return tuple(cells)


def finger_region_metadata() -> Dict[str, Any]:
    """返回可写进 JSON/HDF5 attrs 的稳定物理映射说明。"""
    return {
        "schema": TACTILE_LAYOUT_SCHEMA,
        "source": TACTILE_LAYOUT_SOURCE,
        "wire_grid_shape": [TACTILE_GRID_ROWS, TACTILE_GRID_COLS],
        "wire_active_count": TACTILE_ACTIVE_COUNT,
        "finger_order": list(TACTILE_FINGER_ORDER),
        "physical_finger_array_shape": list(TACTILE_PHYSICAL_FINGER_SHAPE),
        "physical_axis_semantics": ["across_finger", "tip_to_base"],
        "finger_regions_in_wire_grid": {
            finger: list(TACTILE_FINGER_REGIONS[finger])
            for finger in TACTILE_FINGER_ORDER
        },
        "vendor_region_ids": dict(TACTILE_VENDOR_REGION_IDS),
        "effective_finger_shapes": {
            finger: [4, 8] if finger == "thumb" else [4, 7]
            for finger in TACTILE_FINGER_ORDER
        },
        "non_thumb_dead_physical_column": TACTILE_NON_THUMB_DEAD_COLUMN,
        "post_decode_invalid_grid_cells": [
            list(cell) for cell in physical_invalid_grid_coordinates()
        ],
        "physical_slot_count": TACTILE_PHYSICAL_SLOT_COUNT,
        "physical_active_count": TACTILE_PHYSICAL_ACTIVE_COUNT,
        "palm_present": TACTILE_PALM_PRESENT,
        "omitted_generic_ui_regions": "five lower finger arrays and two palm arrays",
        "manus_node_ids": {
            finger: list(TACTILE_TO_MANUS_NODE_IDS[finger])
            for finger in TACTILE_FINGER_ORDER
        },
        "correspondence_level": "finger group only; no taxel-to-MANUS-joint mapping",
    }


def active_coordinates(wire_layout: Mapping[str, Any]
                       ) -> Tuple[Tuple[int, int], ...]:
    """按设备 row-major set-bit 规则从 cellmap 恢复 wire_values 坐标顺序。"""
    rows = int(wire_layout.get("rows", -1))
    cols = int(wire_layout.get("cols", -1))
    count = int(wire_layout.get("active_count", -1))
    if (rows, cols, count) != (
        TACTILE_GRID_ROWS, TACTILE_GRID_COLS, TACTILE_ACTIVE_COUNT,
    ):
        raise ValueError(
            "不支持的触觉 wire layout: {}x{} active={}".format(rows, cols, count)
        )
    cellmap_hex = wire_layout.get("cellmap_hex")
    if not isinstance(cellmap_hex, str):
        raise ValueError("触觉 wire layout 缺少 cellmap_hex")
    try:
        cellmap = bytes.fromhex(cellmap_hex)
    except ValueError as exc:
        raise ValueError("触觉 cellmap_hex 不是有效十六进制") from exc
    if len(cellmap) != rows * 4:
        raise ValueError("触觉 cellmap 长度不是 rows*4")

    coordinates = []
    for row in range(rows):
        mask = int.from_bytes(cellmap[row * 4:(row + 1) * 4], "little")
        if mask >> cols:
            raise ValueError("触觉 cellmap 在声明列数之外含活动位")
        for col in range(cols):
            if mask & (1 << col):
                coordinates.append((row, col))
    if len(coordinates) != count:
        raise ValueError(
            "触觉 cellmap 活动点数 {} 与 active_count {} 不一致".format(
                len(coordinates), count,
            )
        )
    return tuple(coordinates)


def validate_manus_finger_nodes(chain_types: Sequence[Any]) -> None:
    """确认输入 MANUS 节点仍符合本项目记录的25节点手指分组。"""
    if len(chain_types) < 25:
        raise ValueError("MANUS chain_types 少于25节点，无法建立触觉手指对应")
    for finger in TACTILE_FINGER_ORDER:
        expected = finger.lower()
        for node_id in TACTILE_TO_MANUS_NODE_IDS[finger]:
            if str(chain_types[node_id]).strip().lower() != expected:
                raise ValueError(
                    "MANUS node{} chain={}，预期 {}".format(
                        node_id, chain_types[node_id], finger,
                    )
                )
