#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""批次数据删除与可恢复归档；其余条目保持原编号。"""
from __future__ import annotations

import argparse
import ctypes
from datetime import datetime
import json
import os
from pathlib import Path
import re
import shutil
import sys
import uuid
from typing import Any

from session_layout import flat_session_paths, grouped_session_paths, new_session_paths


ASSETS = (
    ("session_dir", "session"),
    ("dir", "raw"),
    ("tactile_dir", "tactile_raw"),
    ("aligned", "aligned.jsonl"),
    ("export", "export.hdf5"),
    ("overlay", "overlay.mp4"),
    ("probe_dir", "probes"),
)
PREFIX_FORBIDDEN = frozenset('<>:"/\\|?*')


def validate_prefix(value: str) -> str:
    prefix = str(value or "")
    if (not prefix or prefix != prefix.strip() or len(prefix) > 120
            or any(ch.isspace() or ord(ch) < 32 or ch in PREFIX_FORBIDDEN
                   for ch in prefix)
            or prefix in (".", "..")):
        raise ValueError("批次前缀无效：不能含空格、路径分隔符或 Windows 非法字符")
    return prefix


def _pid_is_running(pid: int) -> bool:
    if pid <= 0:
        return False
    if os.name == "nt":
        # Windows 的 os.kill(pid, 0) 并非 POSIX 式无害探测，在部分 Python
        # 版本上会终止目标进程。只申请查询句柄，绝不发送信号。
        process_query_limited_information = 0x1000
        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        handle = kernel32.OpenProcess(
            process_query_limited_information, False, pid,
        )
        if handle:
            kernel32.CloseHandle(handle)
            return True
        # 无权查询也说明 PID 对应的进程存在。
        return ctypes.get_last_error() == 5
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def ensure_collection_not_running(root: Path) -> None:
    lock = root / ".run" / "collect_windows.pid"
    if not lock.is_file():
        return
    try:
        pid = int(lock.read_text(encoding="ascii").strip())
    except OSError as exc:
        # collect_windows holds the lock with FileShare.None. On Windows an
        # unreadable lock is positive evidence that collection owns it.
        raise RuntimeError(
            "一键采集程序持有独占锁；请先在采集窗口按 Q 结束"
        ) from exc
    except ValueError:
        return
    if _pid_is_running(pid):
        raise RuntimeError(f"一键采集程序仍在运行 (pid={pid})；请先在采集窗口按 Q 结束")


def parse_indices(text: str) -> list[int]:
    values: set[int] = set()
    for token in re.split(r"[,，;；\s]+", str(text or "").strip()):
        if not token:
            continue
        match = re.fullmatch(r"(\d+)(?:\s*[-~～]\s*(\d+))?", token)
        if not match:
            raise ValueError(f"编号格式无效: {token!r}；示例 2,5,7-9")
        start = int(match.group(1))
        end = int(match.group(2) or start)
        if start < 1 or end > 9999 or end < start or end - start > 10000:
            raise ValueError(f"编号范围无效: {token!r}")
        values.update(range(start, end + 1))
    if not values:
        raise ValueError("至少输入一个要删除的编号")
    return sorted(values)


def session_name(prefix: str, index: int) -> str:
    return f"{prefix}_{index:03d}"


def _path_sets(root: Path, session: str) -> tuple[dict[str, Path], ...]:
    data_root = root / "data"
    return (
        new_session_paths(data_root / "sessions", session, data_root=data_root),
        grouped_session_paths(data_root / "raw", session, data_root=data_root),
        flat_session_paths(data_root / "raw", session, data_root=data_root),
    )


def existing_assets(root: Path, session: str) -> dict[str, Path]:
    task_first, grouped, flat = _path_sets(root, session)
    task_path = task_first["session_dir"]
    task_exists = task_path.exists() or task_path.is_symlink()
    legacy_exists = any(
        path.exists() or path.is_symlink()
        for paths in (grouped, flat)
        for key, _ in ASSETS if key in paths
        for path in (paths[key],)
    )
    if task_exists and legacy_exists:
        raise RuntimeError(f"{session} 同时存在任务优先与旧布局数据，拒绝猜测")
    if task_exists:
        return {"session_dir": task_path}
    found: dict[str, Path] = {}
    for key, _ in ASSETS[1:]:
        candidates = []
        for path in (grouped[key], flat[key]):
            if path not in candidates and (path.exists() or path.is_symlink()):
                candidates.append(path)
        if len(candidates) > 1:
            raise RuntimeError(
                f"{session} 的 {key} 同时存在新旧两份，拒绝猜测: {candidates}")
        if candidates:
            found[key] = candidates[0]
    return found


def list_batch_indices(root: Path, prefix: str) -> list[int]:
    data = root / "data"
    values: set[int] = set()
    legacy_dir_re = re.compile(rf"^{re.escape(prefix)}_(\d{{3,4}})$")
    sessions_batch = data / "sessions" / prefix
    if sessions_batch.is_dir():
        for path in sessions_batch.iterdir():
            if (path.is_dir() and re.fullmatch(r"\d{3,4}", path.name)
                    and int(path.name) > 0):
                values.add(int(path.name))
    for kind in ("raw", "tactile_raw"):
        batch = data / kind / prefix
        if batch.is_dir():
            for path in batch.iterdir():
                if (path.is_dir() and re.fullmatch(r"\d{3,4}", path.name)
                        and int(path.name) > 0):
                    values.add(int(path.name))
        base = data / kind
        if base.is_dir():
            for path in base.iterdir():
                match = legacy_dir_re.fullmatch(path.name)
                if path.is_dir() and match:
                    values.add(int(match.group(1)))
    for kind, suffix in (("aligned", ".jsonl"), ("export", ".hdf5")):
        batch = data / kind / prefix
        if batch.is_dir():
            for path in batch.iterdir():
                if path.is_file() and path.suffix.lower() == suffix:
                    if re.fullmatch(r"\d{3,4}", path.stem) and int(path.stem) > 0:
                        values.add(int(path.stem))
    aligned = data / "aligned"
    if aligned.is_dir():
        for path in aligned.glob(f"{prefix}_*.jsonl"):
            match = legacy_dir_re.fullmatch(path.stem)
            if match:
                values.add(int(match.group(1)))
    export = data / "export"
    legacy_export_re = re.compile(rf"^{re.escape(prefix)}_(\d{{3,4}})_check$")
    if export.is_dir():
        for path in export.glob(f"{prefix}_*_check.hdf5"):
            match = legacy_export_re.fullmatch(path.stem)
            if match:
                values.add(int(match.group(1)))
    return sorted(values)


def _move(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists() or destination.is_symlink():
        raise FileExistsError(f"目标已存在，拒绝覆盖: {destination}")
    shutil.move(str(source), str(destination))


def _atomic_json(path: Path, document: Any) -> None:
    temp = path.with_name(path.name + f".tmp.{uuid.uuid4().hex}")
    temp.write_text(json.dumps(document, ensure_ascii=False, indent=2), encoding="utf-8")
    os.replace(temp, path)


def _replace_strings(value: Any, replacements: list[tuple[str, str]]) -> Any:
    if isinstance(value, str):
        for old, new in replacements:
            value = value.replace(old, new)
        return value
    if isinstance(value, list):
        return [_replace_strings(item, replacements) for item in value]
    if isinstance(value, dict):
        return {key: _replace_strings(item, replacements)
                for key, item in value.items()}
    return value


def _validate_metadata(assets: dict[str, Path]) -> None:
    if "session_dir" in assets:
        manifest = assets["session_dir"] / "manifest.json"
        tactile_meta = assets["session_dir"] / "raw" / "tactile.meta.json"
        export = assets["session_dir"] / "dataset.hdf5"
    else:
        manifest = assets.get("dir", Path()) / "manifest.json" if "dir" in assets else None
        tactile_meta = (assets.get("tactile_dir", Path()) / "tactile.meta.json"
                        if "tactile_dir" in assets else None)
        export = assets.get("export")
    for path in (manifest, tactile_meta):
        if path is not None and path.is_file():
            json.loads(path.read_text(encoding="utf-8-sig"))
    if export is not None and export.is_file():
        import h5py
        with h5py.File(export, "r"):
            pass


def _update_metadata(root: Path, prefix: str, old_index: int,
                     new_index: int, old_assets: dict[str, Path]) -> None:
    old_session = session_name(prefix, old_index)
    new_session = session_name(prefix, new_index)
    new_paths = new_session_paths(
        root / "data" / "sessions", new_session, data_root=root / "data")
    replacements = [(old_session, new_session)]
    for key, _ in ASSETS:
        if key in old_assets:
            replacements.append((str(old_assets[key]), str(new_paths[key])))

    tactile_meta = new_paths["tactile_meta"]
    if tactile_meta.is_file():
        doc = json.loads(tactile_meta.read_text(encoding="utf-8-sig"))
        doc["session"] = new_session
        doc["batch_prefix"] = prefix
        doc["batch_index"] = new_index
        _atomic_json(tactile_meta, doc)

    manifest = new_paths["manifest"]
    if manifest.is_file():
        doc = json.loads(manifest.read_text(encoding="utf-8-sig"))
        doc = _replace_strings(doc, replacements)
        doc["session"] = new_session
        doc["batch_prefix"] = prefix
        doc["batch_index"] = new_index
        _atomic_json(manifest, doc)

    export = new_paths["export"]
    if export.is_file():
        import h5py
        source_attrs = {
            "source_pico": new_paths["pico"],
            "source_manus": new_paths["manus"],
            "source_tactile": new_paths["tactile"],
            "source_tactile_meta": new_paths["tactile_meta"],
            "video_path": new_paths["vst"],
            "video_ts_path": new_paths["vst_ts"],
        }
        with h5py.File(export, "r+") as handle:
            for attr, path in source_attrs.items():
                if attr in handle.attrs:
                    handle.attrs[attr] = str(path.resolve())
            handle.attrs["session"] = new_session
            handle.attrs["batch_prefix"] = prefix
            handle.attrs["batch_index"] = new_index


def _remove_empty_batch_dirs(root: Path, prefix: str) -> None:
    for kind in ("sessions", "raw", "tactile_raw", "aligned", "export", "review"):
        path = root / "data" / kind / prefix
        if path.is_dir() and not any(path.iterdir()):
            path.rmdir()


def _remove_empty_tree(path: Path) -> None:
    """只清理空目录；绝不递归删除可能尚未回滚的数据文件。"""
    if not path.is_dir():
        return
    directories = sorted(
        (item for item in path.rglob("*") if item.is_dir()),
        key=lambda item: len(item.parts), reverse=True)
    for directory in directories:
        if not any(directory.iterdir()):
            directory.rmdir()
    if path.is_dir() and not any(path.iterdir()):
        path.rmdir()


def plan_delete(root: Path, prefix: str, delete: list[int]) -> dict[str, Any]:
    existing = list_batch_indices(root, prefix)
    missing = sorted(set(delete) - set(existing))
    if missing:
        raise ValueError(f"批次 {prefix} 中不存在编号: {missing}")
    return {
        "prefix": prefix,
        "existing": existing,
        "delete": delete,
        # 保留键供旧审计/调用方兼容；删除策略不再移动或重命名幸存条目。
        "renumber": {},
    }


def execute_delete(root: Path, plan: dict[str, Any]) -> Path:
    prefix = plan["prefix"]
    deleted = plan["delete"]
    if plan.get("renumber"):
        raise ValueError("当前删除策略禁止重编号；请重新生成删除预览")
    renumber: dict[int, int] = {}
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    archive = root / "data" / "deleted" / prefix / timestamp
    transaction = root / "data" / ".batch_transactions" / uuid.uuid4().hex
    archive.mkdir(parents=True, exist_ok=False)
    transaction.mkdir(parents=True, exist_ok=False)

    deleted_journal: list[tuple[Path, Path]] = []
    staged_journal: list[tuple[Path, Path]] = []
    final_journal: list[tuple[Path, Path]] = []
    originals: dict[int, dict[str, Path]] = {}
    try:
        for index in sorted(set(deleted) | set(renumber)):
            assets = existing_assets(root, session_name(prefix, index))
            if not assets:
                raise RuntimeError(f"没有找到 {session_name(prefix, index)} 的任何资产")
            _validate_metadata(assets)
            originals[index] = assets

        for index in deleted:
            for key, label in ASSETS:
                source = originals[index].get(key)
                if source is None:
                    continue
                target = archive / f"{index:03d}" / label
                _move(source, target)
                deleted_journal.append((source, target))

        for old_index in sorted(renumber):
            for key, label in ASSETS:
                source = originals[old_index].get(key)
                if source is None:
                    continue
                target = transaction / f"{old_index:03d}" / label
                _move(source, target)
                staged_journal.append((source, target))

        for old_index, new_index in sorted(renumber.items()):
            new_paths = new_session_paths(
                root / "data" / "sessions", session_name(prefix, new_index),
                data_root=root / "data")
            for key, label in ASSETS:
                staged = transaction / f"{old_index:03d}" / label
                if not staged.exists() and not staged.is_symlink():
                    continue
                destination = new_paths[key]
                _move(staged, destination)
                final_journal.append((destination, staged))

        for old_index, new_index in sorted(renumber.items()):
            _update_metadata(root, prefix, old_index, new_index, originals[old_index])
    except Exception:
        for final, staged in reversed(final_journal):
            if final.exists() and not staged.exists():
                _move(final, staged)
        for original, staged in reversed(staged_journal):
            if staged.exists() and not original.exists():
                _move(staged, original)
        for original, archived in reversed(deleted_journal):
            if archived.exists() and not original.exists():
                _move(archived, original)
        raise
    finally:
        _remove_empty_tree(transaction)

    audit = {
        "schema": "pico_batch_delete_v2",
        "prefix": prefix,
        "deleted_indices": deleted,
        "renumber": {str(old): new for old, new in renumber.items()},
        "numbering_policy": "preserve_existing_indices",
        "archived_sources": {
            str(index): {key: str(path) for key, path in originals[index].items()}
            for index in deleted
        },
        "created_at": datetime.now().astimezone().isoformat(),
        "recoverable": True,
    }
    _atomic_json(archive / "batch_edit.json", audit)
    _remove_empty_batch_dirs(root, prefix)
    return archive


def main() -> None:
    parser = argparse.ArgumentParser(description="删除批次中的若干条并保留其余原编号")
    parser.add_argument("--prefix", required=True)
    parser.add_argument("--delete", required=True, dest="delete_text")
    parser.add_argument("--project-root", default=str(Path(__file__).resolve().parent))
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    try:
        root = Path(args.project_root).resolve()
        if not args.dry_run:
            ensure_collection_not_running(root)
        prefix = validate_prefix(args.prefix)
        delete = parse_indices(args.delete_text)
        plan = plan_delete(root, prefix, delete)
        print(f"批次: {prefix}")
        print("现有编号: " + ", ".join(f"{x:03d}" for x in plan["existing"]))
        print("删除编号: " + ", ".join(f"{x:03d}" for x in plan["delete"]))
        print("编号策略: 保留其余条目的原编号（允许编号空缺）")
        if args.dry_run:
            return
        archive = execute_delete(root, plan)
        print(f"完成。被删除数据可恢复归档: {archive}")
    except Exception as exc:  # noqa: BLE001
        print(f"[FAILED] {type(exc).__name__}: {exc}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
