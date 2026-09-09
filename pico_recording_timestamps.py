#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 PICO 连续录制包的 MP4 PTS 转成可与电脑采集数据对齐的墙钟 sidecar。

PICO 录制包的 ``meta.json`` 将 MP4 的 PTS=0 锚定到
``recording_sync.video_start_wall_ns``。输出文件每行对应一个解码显示帧，内容为
该帧的 Unix wall-clock 纳秒；可直接传给 ``export_dataset.py --vst-ts``。
"""
from __future__ import annotations

import argparse
from fractions import Fraction
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


class PicoRecordingError(RuntimeError):
    pass


def load_bundle(recording_dir: Path) -> tuple[dict, Path]:
    recording_dir = recording_dir.resolve()
    meta_path = recording_dir / "meta.json"
    if not meta_path.is_file():
        raise PicoRecordingError(f"缺少 {meta_path}")
    try:
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise PicoRecordingError(f"meta.json 不可读: {exc}") from exc
    video_name = str((meta.get("video") or {}).get("file_name") or "")
    if not video_name or Path(video_name).name != video_name:
        raise PicoRecordingError("meta.json 中 video.file_name 无效")
    video_path = recording_dir / video_name
    if not video_path.is_file():
        raise PicoRecordingError(f"缺少视频文件 {video_path}")
    return meta, video_path


def _ffprobe_frames(video_path: Path, ffprobe: str) -> tuple[Fraction, list[int]]:
    try:
        proc = subprocess.run(
            [
                ffprobe, "-v", "error", "-select_streams", "v:0",
                "-show_entries", "stream=time_base:frame=best_effort_timestamp",
                "-of", "json", str(video_path),
            ],
            check=True, capture_output=True, text=True,
        )
        payload = json.loads(proc.stdout)
    except (OSError, subprocess.CalledProcessError, json.JSONDecodeError) as exc:
        raise PicoRecordingError(f"ffprobe 读取视频帧 PTS 失败: {exc}") from exc
    streams = payload.get("streams") or []
    if len(streams) != 1 or not streams[0].get("time_base"):
        raise PicoRecordingError("ffprobe 未返回唯一视频流的 time_base")
    try:
        time_base = Fraction(streams[0]["time_base"])
        pts = [int(frame["best_effort_timestamp"]) for frame in payload.get("frames", [])]
    except (KeyError, TypeError, ValueError, ZeroDivisionError) as exc:
        raise PicoRecordingError(f"ffprobe 帧时间戳无效: {exc}") from exc
    if not pts:
        raise PicoRecordingError("视频没有可解码帧")
    if any(b <= a for a, b in zip(pts, pts[1:])):
        raise PicoRecordingError("视频显示顺序 PTS 未严格递增")
    return time_base, pts


def build_wall_timestamps(meta: dict, time_base: Fraction,
                          frame_pts: list[int]) -> list[int]:
    sync = meta.get("recording_sync") or {}
    try:
        anchor = int(sync["video_start_wall_ns"])
    except (KeyError, TypeError, ValueError) as exc:
        raise PicoRecordingError("meta.json 缺少有效 video_start_wall_ns") from exc
    if anchor <= 0:
        raise PicoRecordingError("video_start_wall_ns 必须为正数")
    timebase_name = str(sync.get("video_pose_timebase") or "")
    if timebase_name and timebase_name != "relative_pts_sec":
        raise PicoRecordingError(f"不支持 video_pose_timebase={timebase_name!r}")
    scale_num = time_base.numerator * 1_000_000_000
    scale_den = time_base.denominator
    walls = [anchor + (pts * scale_num + scale_den // 2) // scale_den
             for pts in frame_pts]
    if any(b <= a for a, b in zip(walls, walls[1:])):
        raise PicoRecordingError("换算后的逐帧墙钟未严格递增")
    return walls


def _atomic_lines(path: Path, values: list[int]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp_name = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp",
                                     dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="ascii", newline="\n") as handle:
            handle.writelines(f"{value}\n" for value in values)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temp_name, path)
    except Exception:
        try:
            os.unlink(temp_name)
        except OSError:
            pass
        raise


def convert(recording_dir: Path, output: Path | None = None,
            ffprobe: str | None = None, overwrite: bool = False) -> dict:
    meta, video_path = load_bundle(recording_dir)
    ffprobe = ffprobe or os.environ.get("FFPROBE") or shutil.which("ffprobe")
    if not ffprobe:
        raise PicoRecordingError("找不到 ffprobe；请安装 FFmpeg 或设置 FFPROBE")
    output = (output or video_path.with_suffix(".wall.ts.jsonl")).resolve()
    if output.exists() and not overwrite:
        raise PicoRecordingError(f"输出已存在，拒绝覆盖: {output}")
    time_base, pts = _ffprobe_frames(video_path, ffprobe)
    walls = build_wall_timestamps(meta, time_base, pts)
    expected = ((meta.get("recording_diagnostics") or {})
                .get("video_probe_finalized") or {}).get("sample_count")
    if expected is not None and int(expected) != len(walls):
        raise PicoRecordingError(
            f"meta 帧数 {expected} 与 ffprobe 解码帧数 {len(walls)} 不一致")
    _atomic_lines(output, walls)
    return {
        "video": str(video_path),
        "sidecar": str(output),
        "frames": len(walls),
        "first_wall_ns": walls[0],
        "last_wall_ns": walls[-1],
        "time_base": str(time_base),
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="生成 PICO 连续录制视频的逐帧绝对墙钟 sidecar")
    parser.add_argument("recording_dir", help="含 meta.json 和 CameraRecord_*.mp4 的目录")
    parser.add_argument("-o", "--out", default=None, help="输出 .jsonl；默认放在视频旁")
    parser.add_argument("--ffprobe", default=None, help="ffprobe 可执行文件")
    parser.add_argument("--force", action="store_true", help="允许覆盖已有 sidecar")
    args = parser.parse_args()
    try:
        result = convert(
            Path(args.recording_dir), Path(args.out) if args.out else None,
            ffprobe=args.ffprobe, overwrite=args.force,
        )
    except PicoRecordingError as exc:
        print(f"[pico-video] {exc}", file=sys.stderr)
        raise SystemExit(2)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
