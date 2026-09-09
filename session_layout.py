#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""session_layout.py — 采集数据目录约定与路径解析。

任务优先布局（默认，`<session>` 为 `<prefix>_<index>` 时）::

    data/
      sessions/<prefix>/<index>/
        raw/
          pico.jsonl
          manus.jsonl
          manus.meta.json
          tactile.jsonl
          tactile.meta.json
          vst.h264
          vst.ts.jsonl
          vst.qpc.ts.jsonl
        aligned.jsonl
        dataset.hdf5
        manifest.json
        review/overlay.mp4
        review/probes/

旧的类型优先 data/raw、data/tactile_raw、data/aligned、data/export 以及 logs
布局继续兼容读取。

    logs/pico_<session>.jsonl
    logs/manus_<session>.jsonl
    logs/vst_<session>.h264
    logs/vst_<session>.ts.jsonl
    logs/data/aligned/aligned_<session>.jsonl
"""
from __future__ import annotations

import json
from pathlib import Path
import re
from typing import Any, Dict, Optional

DEFAULT_DATA_ROOT = "data"
DEFAULT_RAW_ROOT = "data/sessions"  # 兼容参数名；新采集传入 sessions 根目录
_BATCH_SESSION_RE = re.compile(r"^(.+)_([0-9]{3,4})$")


def split_batch_session(session: str) -> tuple[str, str] | None:
    """把 ``expert_001`` 解析为 ``("expert", "001")``。

    非自动编号会话保持旧的扁平布局，兼容维护命令和历史数据。
    """
    match = _BATCH_SESSION_RE.fullmatch(str(session))
    if not match:
        return None
    prefix, index = match.groups()
    if not prefix or prefix in (".", ".."):
        return None
    return prefix, index


def session_relative_path(session: str) -> Path:
    """返回 session 在 ``data/sessions`` 根目录下的相对路径。"""
    batch = split_batch_session(session)
    return Path(*batch) if batch else Path(session)


def session_raw_dir(raw_root: Path | str, session: str) -> Path:
    """返回新任务优先布局的 raw 目录。"""
    return Path(raw_root) / session_relative_path(session) / "raw"


def new_session_paths(raw_root: Path | str, session: str,
                      data_root: Path | str | None = None) -> Dict[str, Path]:
    """返回任务优先布局下某会话全部标准路径（不要求文件已存在）。"""
    sessions_root = Path(raw_root)
    if data_root is None:
        data_root = sessions_root.parent
    data_root = Path(data_root)
    session_dir = sessions_root / session_relative_path(session)
    raw_dir = session_dir / "raw"
    review_dir = session_dir / "review"
    return {
        "session": session,
        "session_dir": session_dir,
        "dir": raw_dir,
        "pico": raw_dir / "pico.jsonl",
        "manus": raw_dir / "manus.jsonl",
        "manus_meta": raw_dir / "manus.meta.json",
        "vst": raw_dir / "vst.h264",
        "vst_ts": raw_dir / "vst.ts.jsonl",
        "vst_qpc_ts": raw_dir / "vst.qpc.ts.jsonl",
        "tactile_dir": raw_dir,
        "tactile": raw_dir / "tactile.jsonl",
        "tactile_meta": raw_dir / "tactile.meta.json",
        "camera_params": raw_dir / "camera_params.json",
        "camera_params_meta": raw_dir / "camera_params.meta.json",
        "manifest": session_dir / "manifest.json",
        "aligned": session_dir / "aligned.jsonl",
        "export": session_dir / "dataset.hdf5",
        "overlay": review_dir / "overlay.mp4",
        "probe_dir": review_dir / "probes",
        "data_root": data_root,
        "raw_root": sessions_root,
        "layout": "data/sessions/<prefix>/<index>",
        "task_first": True,
    }


def grouped_session_paths(raw_root: Path | str, session: str,
                          data_root: Path | str | None = None) -> Dict[str, Path]:
    """兼容 2026-09-07 以前的类型优先分组布局。"""
    raw_root = Path(raw_root)
    data_root = Path(data_root) if data_root is not None else raw_root.parent
    batch = split_batch_session(session)
    if batch:
        prefix, index = batch
        sess_dir = raw_root / prefix / index
        tactile_dir = data_root / "tactile_raw" / prefix / index
        aligned = data_root / "aligned" / prefix / f"{index}.jsonl"
        export = data_root / "export" / prefix / f"{index}.hdf5"
        overlay = data_root / "review" / prefix / f"overlay_{index}.mp4"
        probe_dir = data_root / "review" / prefix / f"probes_{index}"
    else:
        return flat_session_paths(raw_root, session, data_root=data_root)
    return {
        "session": session, "session_dir": sess_dir, "dir": sess_dir,
        "pico": sess_dir / "pico.jsonl", "manus": sess_dir / "manus.jsonl",
        "manus_meta": sess_dir / "manus.meta.json", "vst": sess_dir / "vst.h264",
        "vst_ts": sess_dir / "vst.ts.jsonl", "vst_qpc_ts": sess_dir / "vst.qpc.ts.jsonl",
        "tactile_dir": tactile_dir, "tactile": tactile_dir / "tactile.jsonl",
        "tactile_meta": tactile_dir / "tactile.meta.json", "manifest": sess_dir / "manifest.json",
        "camera_params": sess_dir / "camera_params.json",
        "camera_params_meta": sess_dir / "camera_params.meta.json",
        "aligned": aligned, "export": export, "overlay": overlay, "probe_dir": probe_dir,
        "data_root": data_root, "raw_root": raw_root,
        "layout": "data/<type>/<prefix>/<index> (legacy-grouped)",
        "legacy_grouped": True,
    }


def flat_session_paths(raw_root: Path | str, session: str,
                       data_root: Path | str | None = None) -> Dict[str, Path]:
    """兼容 2026-08-27 以前的 ``data/raw/<session>`` 扁平资产。"""
    raw_root = Path(raw_root)
    data_root = Path(data_root) if data_root is not None else raw_root.parent
    sess_dir = raw_root / session
    tactile_dir = data_root / "tactile_raw" / session
    return {
        "session": session,
        "session_dir": sess_dir,
        "dir": sess_dir,
        "pico": sess_dir / "pico.jsonl",
        "manus": sess_dir / "manus.jsonl",
        "manus_meta": sess_dir / "manus.meta.json",
        "vst": sess_dir / "vst.h264",
        "vst_ts": sess_dir / "vst.ts.jsonl",
        "vst_qpc_ts": sess_dir / "vst.qpc.ts.jsonl",
        "tactile_dir": tactile_dir,
        "tactile": tactile_dir / "tactile.jsonl",
        "tactile_meta": tactile_dir / "tactile.meta.json",
        "camera_params": sess_dir / "camera_params.json",
        "camera_params_meta": sess_dir / "camera_params.meta.json",
        "manifest": sess_dir / "manifest.json",
        "aligned": data_root / "aligned" / f"{session}.jsonl",
        "export": data_root / "export" / f"{session}_check.hdf5",
        "overlay": data_root / "review" / f"overlay_{session}.mp4",
        "probe_dir": data_root / "review" / f"probes_{session}",
        "data_root": data_root,
        "raw_root": raw_root,
        "layout": "data/raw/<session> (legacy-flat)",
        "legacy_flat": True,
    }


def legacy_session_paths(logs_dir: Path | str, session: str) -> Dict[str, Path]:
    logs_dir = Path(logs_dir)
    return {
        "session": session,
        "dir": logs_dir,
        "pico": logs_dir / f"pico_{session}.jsonl",
        "manus": logs_dir / f"manus_{session}.jsonl",
        "manus_meta": logs_dir / f"manus_{session}.meta.json",
        "vst": logs_dir / f"vst_{session}.h264",
        "vst_ts": logs_dir / f"vst_{session}.ts.jsonl",
        "vst_qpc_ts": logs_dir / f"vst_{session}.qpc.ts.jsonl",
        "tactile_dir": logs_dir / "data" / "tactile_raw" / session,
        "tactile": logs_dir / "data" / "tactile_raw" / session / "tactile.jsonl",
        "tactile_meta": logs_dir / "data" / "tactile_raw" / session / "tactile.meta.json",
        "manifest": logs_dir / f"manifest_{session}.json",
        "aligned": logs_dir / "data" / "aligned" / f"aligned_{session}.jsonl",
        "export": logs_dir / "data" / "export" / f"dataset_{session}.hdf5",
        "overlay": logs_dir / f"overlay_{session}.mp4",
        "probe_dir": logs_dir / "triage" / f"probes_{session}",
        "data_root": logs_dir / "data",
        "raw_root": logs_dir,
        "legacy": True,
    }


def resolve_session(session: str,
                    data_root: Path | str = DEFAULT_DATA_ROOT,
                    logs_dir: Path | str = "logs") -> Optional[Dict[str, Path]]:
    """解析会话：优先任务布局，再兼容旧类型布局和 logs；找不到则 None。"""
    data_root = Path(data_root)
    newp = new_session_paths(data_root / "sessions", session, data_root=data_root)
    if newp["pico"].is_file() or newp["session_dir"].is_dir():
        newp["legacy"] = False
        return newp
    raw_root = data_root / "raw"
    grouped = grouped_session_paths(raw_root, session, data_root=data_root)
    if grouped["pico"].is_file() or grouped["dir"].is_dir():
        grouped["legacy"] = False
        return grouped
    flatp = flat_session_paths(raw_root, session, data_root=data_root)
    if flatp["pico"].is_file() or flatp["dir"].is_dir():
        flatp["legacy"] = False
        return flatp
    oldp = legacy_session_paths(logs_dir, session)
    if oldp["pico"].is_file():
        return oldp
    return None


def list_sessions(data_root: Path | str = DEFAULT_DATA_ROOT,
                  logs_dir: Path | str = "logs") -> list[str]:
    """列出已知会话名（新+旧，去重，新优先顺序靠前）。"""
    data_root, logs_dir = Path(data_root), Path(logs_dir)
    names: list[str] = []
    seen = set()
    sessions = data_root / "sessions"
    if sessions.is_dir():
        candidates: list[tuple[float, str]] = []
        for first in sessions.iterdir():
            if not first.is_dir():
                continue
            if (first / "raw" / "pico.jsonl").exists():
                candidates.append((first.stat().st_mtime, first.name))
                continue
            for second in first.iterdir():
                if second.is_dir() and (second / "raw" / "pico.jsonl").exists():
                    candidates.append((second.stat().st_mtime, f"{first.name}_{second.name}"))
        for _, name in sorted(candidates, reverse=True):
            if name not in seen:
                names.append(name)
                seen.add(name)
    raw = data_root / "raw"
    if raw.is_dir():
        candidates: list[tuple[float, str]] = []
        for first in raw.iterdir():
            if not first.is_dir():
                continue
            if (first / "pico.jsonl").exists():
                candidates.append((first.stat().st_mtime, first.name))
                continue
            for second in first.iterdir():
                if second.is_dir() and (second / "pico.jsonl").exists():
                    candidates.append(
                        (second.stat().st_mtime, f"{first.name}_{second.name}"))
        for _, name in sorted(candidates, reverse=True):
            if name not in seen:
                names.append(name)
                seen.add(name)
    if logs_dir.is_dir():
        for p in sorted(logs_dir.glob("pico_*.jsonl"),
                        key=lambda x: x.stat().st_mtime, reverse=True):
            sess = p.name[len("pico_"):-len(".jsonl")]
            if sess not in seen:
                names.append(sess)
                seen.add(sess)
    return names


def write_manifest(paths: Dict[str, Path], extra: Optional[Dict[str, Any]] = None) -> Path:
    """写/更新 session manifest.json（对接用元数据）。"""
    man = paths["manifest"]
    man.parent.mkdir(parents=True, exist_ok=True)
    files = {}
    for key in ("pico", "manus", "manus_meta", "vst", "vst_ts", "vst_qpc_ts", "tactile", "tactile_meta",
                "camera_params", "camera_params_meta",
                "aligned", "export", "overlay"):
        p = paths.get(key)
        if p is None:
            continue
        files[key] = {
            "path": str(p),
            "exists": p.is_file(),
            "bytes": p.stat().st_size if p.is_file() else 0,
        }
    doc: Dict[str, Any] = {
        "schema": "pico_ego_session_manifest_v1",
        "session": paths.get("session"),
        "layout": "legacy" if paths.get("legacy") else paths.get("layout", "data/raw/<session>"),
        "files": files,
        "clocks": {
            "align_key": "recv_qpc_ns preferred; whole-session recv_wall_ns fallback",
            "align_clock": "single capture-host monotonic/QPC domain; never mixed",
            "pico_sensor_field": "data.*.timeStampNs (export resample only; not used for MANUS align)",
            "vst_sidecar": "paired vst.ts.jsonl wall_ns + vst.qpc.ts.jsonl QPC per picture",
            "tactile": "tactile.jsonl recv_qpc_ns preferred; nearest-neighbor, no value interpolation",
        },
        "coordinates": {
            "training_frame": "right-handed X-forward Y-left Z-up meters",
            "pico_raw": "left-handed in pose string; convert via LH→RH + Pico→Robot axes",
            "manus": "native RH wrist-local (HandMotion_None); world wrist = pico_ctrl ∘ T_calib",
            "hand_joints": "wrist-local after hand_local()",
        },
        "calib_refs": {
            "wrist": "config/calib_wrist.json",
            "head_to_cam": "config/pico_cam/head_to_cam_solved.json",
            "vst_intrinsics": "config/pico_cam/vst_cam.json",
            "pico_camera_params_snapshot": "raw/camera_params.json",
        },
    }
    manus_meta = paths.get("manus_meta")
    if manus_meta is not None and manus_meta.is_file():
        try:
            capture_meta = json.loads(manus_meta.read_text(encoding="utf-8-sig"))
            doc["manus_calibration"] = capture_meta.get("calibration", {})
            doc["manus_calibration_required"] = capture_meta.get(
                "calibration_required", True
            )
        except (OSError, ValueError) as exc:
            doc["manus_calibration_error"] = str(exc)
    if extra:
        doc.update(extra)
    man.write_text(json.dumps(doc, ensure_ascii=False, indent=2), encoding="utf-8")
    return man
