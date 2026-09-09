#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Select and snapshot PICO camera calibration for each capture session."""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
from typing import Any


_HERE = Path(__file__).resolve().parent
DEFAULT_DEVICE_CONFIG_ROOT = _HERE / "config" / "pico_cam" / "devices"
_SAFE_SERIAL_RE = re.compile(r"[A-Za-z0-9._-]+")


@dataclass(frozen=True)
class CameraParamsSelection:
    device_serial: str
    source: Path
    content: bytes
    sha256: str
    contains: tuple[str, ...]
    extrinsics_included: bool


def device_serial_from_status(status: str) -> str:
    """Return the sole online PICO serial from a receiver STATUS reply."""
    match = re.search(r"(?:^|\s)devices=([^\s]+)", status or "")
    value = match.group(1) if match else ""
    if value in ("", "(none)"):
        raise ValueError("PICO STATUS 中没有在线设备序列号")
    serials = [item for item in value.split(",") if item]
    if len(serials) != 1:
        raise ValueError(f"当前在线 PICO 数量不是 1: {value}")
    serial = serials[0]
    if _SAFE_SERIAL_RE.fullmatch(serial) is None:
        raise ValueError(f"PICO 设备序列号包含非法字符: {serial!r}")
    return serial


def _positive_number(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} 必须是数值")
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise ValueError(f"{field} 必须是正有限数")
    return number


def _finite_number(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} 必须是数值")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{field} 必须是有限数")
    return number


def _validate_distortion(camera: dict[str, Any], side: str) -> None:
    distortion = camera.get("distortion")
    if not isinstance(distortion, dict):
        raise ValueError(f"camera_params.json 缺少 {side}.distortion")
    if not isinstance(distortion.get("model"), str) or not distortion["model"]:
        raise ValueError(f"{side}.distortion.model 无效")
    coeffs = distortion.get("coeffs")
    if not isinstance(coeffs, list) or not coeffs:
        raise ValueError(f"{side}.distortion.coeffs 无效")
    for index, value in enumerate(coeffs):
        _finite_number(value, f"{side}.distortion.coeffs[{index}]")


def _validate_camera_params(doc: Any) -> bool:
    """Validate Documents intrinsics or full CaptureLib calibration."""
    if not isinstance(doc, dict):
        raise ValueError("camera_params.json 顶层必须是对象")
    resolution = doc.get("reference_resolution")
    compact_intrinsics = isinstance(resolution, dict)
    if compact_intrinsics:
        _positive_number(resolution.get("w"), "reference_resolution.w")
        _positive_number(resolution.get("h"), "reference_resolution.h")
    else:
        _positive_number(doc.get("width"), "width")
        _positive_number(doc.get("height"), "height")
        _positive_number(doc.get("outputWidth"), "outputWidth")
        _positive_number(doc.get("outputHeight"), "outputHeight")
        _positive_number(doc.get("fps"), "fps")

    all_extrinsics = True
    for side in ("left", "right"):
        camera = doc.get(side)
        if not isinstance(camera, dict):
            raise ValueError(f"camera_params.json 缺少 {side}")
        intrinsics = camera.get("intrinsics") if compact_intrinsics else camera
        if not isinstance(intrinsics, dict):
            raise ValueError(f"camera_params.json 缺少 {side} 内参")
        for key in ("fx", "fy", "cx", "cy"):
            _positive_number(intrinsics.get(key), f"{side}.intrinsics.{key}")
        _validate_distortion(camera, side)
        extrinsic = camera.get("extrinsic")
        has_extrinsic = camera.get("hasExtrinsics") is True
        if extrinsic is None and not has_extrinsic:
            all_extrinsics = False
            continue
        if not isinstance(extrinsic, list) or len(extrinsic) != 12:
            raise ValueError(f"{side}.extrinsic 必须包含 12 个数")
        for index, value in enumerate(extrinsic):
            _finite_number(value, f"{side}.extrinsic[{index}]")
    return all_extrinsics


def select_camera_params(
        status: str,
        config_root: Path | str = DEFAULT_DEVICE_CONFIG_ROOT,
) -> CameraParamsSelection:
    """Load and validate calibration for the sole device in STATUS."""
    serial = device_serial_from_status(status)
    source = Path(config_root) / serial / "camera_params.json"
    try:
        content = source.read_bytes()
    except OSError as exc:
        raise ValueError(f"找不到当前设备的相机标定: {source}") from exc
    try:
        doc = json.loads(content.decode("utf-8-sig"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"相机标定 JSON 不可读: {source}: {exc}") from exc
    extrinsics_included = _validate_camera_params(doc)
    contains = (("intrinsics", "distortion", "extrinsics")
                if extrinsics_included else ("intrinsics", "distortion"))
    return CameraParamsSelection(
        device_serial=serial,
        source=source,
        content=content,
        sha256=hashlib.sha256(content).hexdigest(),
        contains=contains,
        extrinsics_included=extrinsics_included,
    )


def snapshot_camera_params(
        raw_dir: Path | str,
        selection: CameraParamsSelection,
) -> tuple[Path, Path]:
    """Atomically write the exact calibration plus provenance into session raw/."""
    raw_dir = Path(raw_dir)
    raw_dir.mkdir(parents=True, exist_ok=True)
    target = raw_dir / "camera_params.json"
    meta_target = raw_dir / "camera_params.meta.json"
    if target.exists() and target.read_bytes() != selection.content:
        raise ValueError(f"拒绝覆盖内容不同的会话标定: {target}")

    partial = raw_dir / "camera_params.json.partial"
    partial.write_bytes(selection.content)
    os.replace(partial, target)

    meta = {
        "schema": "pico_camera_params_snapshot_v1",
        "device_serial": selection.device_serial,
        "source": str(selection.source),
        "sha256": selection.sha256,
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "contains": list(selection.contains),
        "extrinsics_included": selection.extrinsics_included,
        "extrinsic_convention": (
            "head_to_camera_3x4_row_major"
            if selection.extrinsics_included else None
        ),
    }
    meta_partial = raw_dir / "camera_params.meta.json.partial"
    meta_partial.write_text(
        json.dumps(meta, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    os.replace(meta_partial, meta_target)
    return target, meta_target
