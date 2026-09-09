#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pico_record.py — 一条命令协调 PICO + MANUS + 双手触觉原始录制。

前提: 已用统一脚本起好 PICO/MANUS/触觉，并按终端提示完成触觉左右配对:
  bash scripts/ego_ctl.sh start

日常采集(同一终端, Enter 结束, 不必另开窗口):
  python3 pico_record.py start [任务名]
  python3 pico_record.py start [任务名] --hands left

开录默认会等头显/手柄/MANUS/触觉(及 VST 暖机)都有新鲜数据。
`--force` 只跳过开录前 PICO/MANUS/VST 新鲜度；录制中断流监控仍生效，且不能跳过触觉左右配对与双路原始流门禁。
旧的 PICO+MANUS 双路流程显式使用 `--no-tactile`。

其它:
  python3 pico_record.py stop
  python3 pico_record.py status
  python3 pico_record.py start X --detach
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import math
import re
import select
import signal
import shutil
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

try:
    import msvcrt  # Windows 控制台按键；POSIX 不提供该模块。
except ImportError:  # pragma: no cover - 仅非 Windows
    msvcrt = None

from record_control import (
    send_command, PICO_CONTROL_PORT, MANUS_CONTROL_PORT, TACTILE_CONTROL_PORT,
    parse_hands, hands_label, HANDS_CHOICES,
)
from pico_camera_params import select_camera_params, snapshot_camera_params
from session_layout import flat_session_paths, grouped_session_paths, new_session_paths

_BAR = "=" * 60
_HERE = Path(__file__).resolve().parent
_SESSIONS_ROOT = _HERE / "data" / "sessions"
_LEGACY_RAW_ROOT = _HERE / "data" / "raw"
_WINDOWS_FORBIDDEN = frozenset('<>:"/\\|?*')
_WINDOWS_RESERVED = frozenset(
    {"CON", "PRN", "AUX", "NUL"}
    | {f"COM{i}" for i in range(1, 10)}
    | {f"LPT{i}" for i in range(1, 10)})
_DEFAULT_COMMAND_TIMEOUT_S = 5.0
_DEFAULT_TACTILE_STOP_TIMEOUT_S = 15.0
_DEFAULT_READY_STABLE_S = 2.0
_DEFAULT_RECORD_FAULT_GRACE_S = 1.5
_RECORD_HEALTH_POLL_S = 0.5
_EXIT_OPERATOR_RESTART = 10
_EXIT_OPERATOR_QUIT = 11
_EXIT_READY_TIMEOUT = 12
_EXIT_SENSOR_FAULT = 13


def _session_paths(session: str) -> dict[str, Path]:
    return new_session_paths(_SESSIONS_ROOT, session, data_root=_HERE / "data")


def _safe_print(*args, **kwargs) -> None:
    """控制/回滚不能因 stdout 管道关闭而被打断。"""
    try:
        print(*args, **kwargs)
    except (BrokenPipeError, OSError, ValueError):
        pass


def _cmd(port, line, label, quiet=False, timeout=_DEFAULT_COMMAND_TIMEOUT_S):
    try:
        reply = send_command(port, line, timeout=timeout)
    except Exception as e:  # noqa: BLE001
        if not quiet:
            _safe_print(f"[{label}] 连不上服务(端口 {port}): {e}", flush=True)
            _safe_print(f"[{label}] → 先在本机跑: bash scripts/ego_ctl.sh start",
                        flush=True)
        return str(e), False
    transport_ok = bool(reply)
    if not quiet:
        _safe_print(f"[{label}] {reply or '(空回复)'}", flush=True)
    return reply, transport_ok


def _kv_int(text: str, key: str, default: int = -1) -> int:
    m = re.search(rf"\b{re.escape(key)}=(-?\d+)\b", text or "")
    if not m:
        return default
    try:
        return int(m.group(1))
    except ValueError:
        return default


def _kv_str(text: str, key: str, default: str = "") -> str:
    m = re.search(rf"\b{re.escape(key)}=([^\s]+)", text or "")
    return m.group(1) if m else default


def _parse_side_values(text: str, key: str) -> dict:
    """解析 age_ms=l=12,r=15 / frames=l=10,r=11。"""
    m = re.search(rf"\b{re.escape(key)}=([^\s]+)", text or "")
    out = {"left": -1, "right": -1}
    if not m:
        return out
    for part in m.group(1).split(","):
        if "=" not in part:
            continue
        k, v = part.split("=", 1)
        try:
            iv = int(v)
        except ValueError:
            continue
        if k.startswith("l"):
            out["left"] = iv
        elif k.startswith("r"):
            out["right"] = iv
    return out


def _parse_manus_ages(text: str) -> dict:
    return _parse_side_values(text, "age_ms")


def _validate_session_name(value: str) -> str:
    """使用三路服务都能无歧义接受的单 token session 名。"""
    original = str(value or "")
    name = original.strip()
    if not name:
        raise ValueError("session 名不能为空")
    if name != original or len(name) > 128:
        raise ValueError("session 名不能有首尾空白，且不能超过128个字符")
    if name in (".", "..") or name.endswith((" ", ".")):
        raise ValueError(f"session 名不安全: {name!r}")
    if any(ch.isspace() or ord(ch) < 32 or ch in _WINDOWS_FORBIDDEN for ch in name):
        raise ValueError("session 名不能含空白、路径分隔符、控制字符或文件名非法字符")
    if name.split(".", 1)[0].upper() in _WINDOWS_RESERVED:
        raise ValueError(f"session 名不能使用 Windows 设备保留名: {name!r}")
    return name


def _session_conflicts(session: str) -> list[Path]:
    """同名目录跨模态一律拒绝，避免追加、覆盖或误关联旧采集。"""
    paths = _session_paths(session)
    conflicts = []
    for target in (paths["dir"], paths["tactile_dir"]):
        if target.exists() or target.is_symlink():
            conflicts.append(target)
    for legacy in (
        grouped_session_paths(_LEGACY_RAW_ROOT, session, data_root=_HERE / "data"),
        flat_session_paths(_LEGACY_RAW_ROOT, session, data_root=_HERE / "data"),
    ):
        for target in (legacy["dir"], legacy["tactile_dir"]):
            if target not in conflicts and (target.exists() or target.is_symlink()):
                conflicts.append(target)
    return conflicts


def _check_tactile_ready(tactile_st: str, max_age_ms: int) -> list[str]:
    """触觉是 fail-closed 门禁；raw 阶段允许 pipeline/mask/manus-live 为0。"""
    reasons = []
    if not tactile_st.startswith("OK "):
        return ["触觉服务离线或 STATUS 协议错误"]
    if _kv_str(tactile_st, "capture_scope") != "tactile_only":
        reasons.append("63912 不是预期的触觉原始采集服务")
    if _kv_int(tactile_st, "pipeline_ready", -1) != 0:
        reasons.append("触觉服务阶段标记异常 (raw-only 应为 pipeline_ready=0)")
    state = _kv_str(tactile_st, "state")
    if state != "PAIRED_IDLE":
        if state == "RECORDING_RAW":
            reasons.append("触觉已在录制中：先 stop")
        elif state == "FAULT_LATCHED":
            reasons.append(f"触觉故障已锁存: {_kv_str(tactile_st, 'fault', 'unknown')}")
        else:
            reasons.append(f"触觉状态不可开录 (state={state or 'missing'})")
    for key in ("paired", "raw_stream_valid", "start_ready"):
        if _kv_int(tactile_st, key, 0) != 1:
            reasons.append(f"触觉 {key}!=1")
    if _kv_int(tactile_st, "errors", -1) != 0:
        reasons.append(f"触觉读写错误计数非零 (errors={_kv_int(tactile_st, 'errors')})")
    if _kv_int(tactile_st, "gaps", -1) != 0:
        reasons.append(f"触觉订阅存在缺帧 (gaps={_kv_int(tactile_st, 'gaps')})")
    if _kv_str(tactile_st, "fault", "missing") != "none":
        reasons.append(f"触觉 fault={_kv_str(tactile_st, 'fault', 'missing')}")
    ages = _parse_side_values(tactile_st, "age_ms")
    for side in ("left", "right"):
        age = ages[side]
        if age < 0 or age > max_age_ms:
            reasons.append(f"触觉 {side} 过旧/无数据 (age_ms={age}, 要≤{max_age_ms})")
    return reasons


def _check_ready(pico_st: str, manus_st: str, tactile_st: str, *,
                 do_pico: bool, do_manus: bool, do_tactile: bool,
                 hands: frozenset, max_age_ms: int, require_vst: bool,
                 skip_motion_freshness: bool = False) -> list[str]:
    """返回未就绪原因列表；空=可开录。"""
    reasons = []
    if do_pico:
        if not pico_st.startswith("OK "):
            reasons.append("PICO 服务离线或 STATUS 协议错误")
        elif pico_st.startswith("OK REC"):
            reasons.append("PICO 已在录制中：先 stop")
        elif not pico_st.startswith("OK idle"):
            reasons.append("PICO STATUS 返回未知状态")
        elif _kv_int(pico_st, "writer_errors", 0) != 0:
            reasons.append(
                f"PICO 异步写盘错误非零 (writer_errors={_kv_int(pico_st, 'writer_errors')})"
            )
        elif _kv_int(pico_st, "vst_writer_errors", 0) != 0:
            reasons.append(
                "VST 异步写盘错误非零 "
                f"(vst_writer_errors={_kv_int(pico_st, 'vst_writer_errors')})"
            )
        elif not skip_motion_freshness:
            if ("devices=(none)" in pico_st
                    or _kv_str(pico_st, "devices") in ("", "(none)")):
                reasons.append("头显未连接 (devices=none)：App 选本机 IP 并开 Send/Head/Controller")
            else:
                age = _kv_int(pico_st, "pose_age_ms")
                if age < 0 or age > max_age_ms:
                    reasons.append(f"PICO 姿态过旧/无数据 (pose_age_ms={age}, 要≤{max_age_ms})")
                if _kv_int(pico_st, "head", 0) != 1:
                    reasons.append("PICO 无 Head 数据：App 打开 Head 追踪")
                if "left" in hands and _kv_int(pico_st, "ctrl_l", 0) != 1:
                    reasons.append("左手柄无数据：握紧/唤醒左手柄")
                if "right" in hands and _kv_int(pico_st, "ctrl_r", 0) != 1:
                    reasons.append("右手柄无数据：握紧/唤醒右手柄")
            if require_vst:
                if _kv_int(pico_st, "vst_on", 0) != 1:
                    reasons.append("VST 未推流 (vst_on=0)：等暖机或检查 --video 服务")
                else:
                    vage = _kv_int(pico_st, "vst_age_ms")
                    if vage < 0 or vage > max_age_ms:
                        reasons.append(f"VST 帧过旧/无 (vst_age_ms={vage}, 要≤{max_age_ms})")
    if do_manus:
        if not manus_st.startswith("OK "):
            reasons.append("MANUS 服务离线或 STATUS 协议错误")
        elif manus_st.startswith("OK REC"):
            reasons.append("MANUS 已在录制中：先 stop")
        elif not manus_st.startswith("OK idle"):
            reasons.append("MANUS STATUS 返回未知状态")
        elif _kv_int(manus_st, "writer_errors", 0) != 0:
            reasons.append(
                f"MANUS 异步写盘错误非零 (writer_errors={_kv_int(manus_st, 'writer_errors')})"
            )
        elif not skip_motion_freshness:
            gloves = _kv_str(manus_st, "gloves", "(none)")
            if gloves in ("", "(none)"):
                reasons.append("手套未上线 (gloves=none)：dongle/开机，等几秒")
            else:
                ages = _parse_manus_ages(manus_st)
                calibration = _parse_side_values(manus_st, "calib")
                for side in hands:
                    if side not in gloves:
                        reasons.append(f"缺 {side} 手套在线 (gloves={gloves})")
                    else:
                        age = ages.get(side, -1)
                        if age < 0 or age > max_age_ms:
                            reasons.append(
                                f"MANUS {side} 过旧/无数据 (age_ms={age}, 要≤{max_age_ms})")
                        if calibration.get(side, 0) != 1:
                            reasons.append(
                                f"MANUS {side} 个人手型标定未成功加载 (calib=0)")
    if do_tactile:
        reasons.extend(_check_tactile_ready(tactile_st, max_age_ms))
    return reasons


def _advance_ready_window(ready_since: float | None, is_ready: bool,
                          now: float, stable_s: float) -> tuple[float | None, bool]:
    """就绪必须连续保持 stable_s，避免一帧偶然恢复就立即开录。"""
    if not is_ready:
        return None, False
    since = now if ready_since is None else ready_since
    return since, (now - since) >= stable_s


def _advance_fault_window(fault_since: float | None, has_issue: bool,
                          now: float, grace_s: float
                          ) -> tuple[float | None, bool, bool]:
    """录制故障去抖：返回 (起点, 已确认, 本轮是否恢复)。"""
    if not has_issue:
        return None, False, fault_since is not None
    since = now if fault_since is None else fault_since
    return since, (now - since) >= grace_s, False


def _wait_ready(*, do_pico, do_manus, do_tactile,
                pico_port, manus_port, tactile_port, hands,
                timeout_s: float, max_age_ms: int, require_vst: bool,
                skip_motion_freshness: bool = False,
                stable_s: float = _DEFAULT_READY_STABLE_S) -> bool:
    checked = []
    if do_pico:
        checked.append("头显/手柄")
    if do_manus:
        checked.append("MANUS")
    if do_tactile:
        checked.append("双手触觉配对/原始流")
    if require_vst and not skip_motion_freshness:
        checked.append("VST暖机")
    print(_BAR, flush=True)
    timeout_label = f"超时 {timeout_s:.0f}s" if timeout_s > 0 else "不限时"
    print(f"  开录前等待（{'/'.join(checked)}，连续稳定 {stable_s:.1f}s，{timeout_label}）",
          flush=True)
    print(_BAR, flush=True)
    t0 = time.monotonic()
    last_print = 0.0
    ready_since = None
    while True:
        pico_st = manus_st = tactile_st = ""
        if do_pico:
            pico_st, ok = _cmd(pico_port, "STATUS", "PICO", quiet=True)
            if not ok:
                pico_st = "ERR offline"
        if do_manus:
            manus_st, ok = _cmd(manus_port, "STATUS", "MANUS", quiet=True)
            if not ok:
                manus_st = "ERR offline"
        if do_tactile:
            tactile_st, ok = _cmd(tactile_port, "STATUS", "TACTILE", quiet=True)
            if not ok:
                tactile_st = "ERR offline"
        reasons = _check_ready(
            pico_st, manus_st, tactile_st,
            do_pico=do_pico, do_manus=do_manus, do_tactile=do_tactile,
            hands=hands, max_age_ms=max_age_ms, require_vst=require_vst,
            skip_motion_freshness=skip_motion_freshness)
        now = time.monotonic()
        was_waiting_stable = ready_since is not None
        ready_since, stable = _advance_ready_window(
            ready_since, not reasons, now, stable_s,
        )
        if stable:
            print(f"[ready] 全部传感器连续稳定 {stable_s:.1f}s，允许开始录制。",
                  flush=True)
            return True
        if not reasons and not was_waiting_stable:
            print(f"[stabilize] 当前全部就绪，继续观察 {stable_s:.1f}s…", flush=True)
        if reasons and was_waiting_stable:
            print("[wait] 稳定确认期间再次掉线，稳定计时已重新开始。", flush=True)
        if reasons and now - last_print >= 1.0:
            last_print = now
            print(f"[wait] {now - t0:4.0f}s  未就绪 {len(reasons)} 项:", flush=True)
            for reason in reasons[:10]:
                print(f"       - {reason}", flush=True)
            if do_pico:
                print(f"       PICO    {pico_st[:180]}", flush=True)
            if do_manus:
                print(f"       MANUS   {manus_st[:180]}", flush=True)
            if do_tactile:
                print(f"       TACTILE {tactile_st[:220]}", flush=True)
        if timeout_s > 0 and now - t0 >= timeout_s:
            if do_tactile:
                print("[!] 就绪超时；触觉门禁不能用 --force 绕过。请重启触觉服务并重新人工配对。",
                      flush=True)
            else:
                print("[!] 就绪超时。修好设备后重试，或 --force 强开(不推荐)。", flush=True)
            return False
        time.sleep(0.4)


def _wait_vst_after_start(pico_port, timeout_s: float = 40.0,
                          min_frames: int = 5) -> bool:
    """START 后等 VST 落盘帧增长(重启推流有短暂空窗)。"""
    t0 = time.time()
    base = 0  # compatibility with the existing localized progress message
    while time.time() - t0 < timeout_s:
        st, ok = _cmd(pico_port, "STATUS", "PICO", quiet=True)
        if not ok or not st.startswith("OK "):
            time.sleep(0.3)
            continue
        n = _kv_int(st, "vst_record_frames", 0)
        if (_kv_int(st, "vst_recording", 0) == 1
                and _kv_int(st, "vst_record_ready", 0) == 1
                and n >= min_frames
                and _kv_int(st, "vst_on", 0) == 1
                and _kv_int(st, "vst_writer_errors", 0) == 0):
            print(f"[ready] VST 已出帧 (+{n - base}，累计 {n})", flush=True)
            return True
        time.sleep(0.3)
    print("[!] 防呆: START 后 VST 仍无新帧；拒绝进入正式录制。", flush=True)
    print("[!] VST guard: no durable SPS/PPS + IDR session stream.", flush=True)
    return False


def _build_services(*, do_pico: bool, do_manus: bool, do_tactile: bool,
                    pico_port: int, manus_port: int, tactile_port: int) -> list[dict]:
    """返回 START 顺序：严格门禁的触觉最先，随后 PICO、MANUS。"""
    services = []
    if do_tactile:
        services.append({"key": "tactile", "label": "TACTILE", "port": tactile_port})
    if do_pico:
        services.append({"key": "pico", "label": "PICO", "port": pico_port})
    if do_manus:
        services.append({"key": "manus", "label": "MANUS", "port": manus_port})
    return services


def _service_is_recording(service: dict, status: str, session: str) -> bool:
    if not status.startswith("OK ") or _kv_str(status, "session") != session:
        return False
    if service["key"] == "tactile":
        return _kv_str(status, "state") == "RECORDING_RAW"
    return status.startswith("OK REC")


def _service_is_idle(service: dict, status: str) -> bool:
    if not status.startswith("OK "):
        return False
    if service["key"] == "tactile":
        return _kv_str(status, "state") == "PAIRED_IDLE"
    return status.startswith("OK idle")


def _capture_tactile_paths(service: dict, reply: str,
                           session: str) -> tuple[bool, str]:
    match = re.search(r"\bfile=(.+?)\s+meta=(.+)$", reply or "")
    if not match:
        return False, "START 回复缺 file/meta 路径"
    try:
        final_path = Path(match.group(1).strip())
        meta_path = Path(match.group(2).strip())
        expected_dir = _session_paths(session)["tactile_dir"].resolve()
        final_resolved = final_path.resolve()
        meta_resolved = meta_path.resolve()
    except (OSError, RuntimeError, ValueError) as exc:
        return False, f"START 路径不可解析: {type(exc).__name__}: {exc}"
    if (final_resolved != expected_dir / "tactile.jsonl"
            or meta_resolved != expected_dir / "tactile.meta.json"):
        return False, ("触觉服务必须使用标准目录 "
                       f"{expected_dir}; reply file={final_path} meta={meta_path}")
    service["file_path"] = final_path
    service["meta_path"] = meta_path
    return True, ""


def _capture_motion_path(service: dict, reply: str,
                         session: str) -> tuple[bool, str]:
    match = re.search(r"\bfile=(.+)$", reply or "")
    if not match:
        return False, "START 回复缺 file 路径"
    try:
        path = Path(match.group(1).strip())
        expected = _session_paths(session)[service["key"]].resolve()
        resolved = path.resolve()
    except (OSError, RuntimeError, ValueError) as exc:
        return False, f"START 路径不可解析: {type(exc).__name__}: {exc}"
    if resolved != expected:
        return False, f"{service['label']} 必须写标准路径 {expected}; reply file={path}"
    service["file_path"] = path
    return True, ""


def _valid_start_reply(service: dict, reply: str, session: str,
                       expected_hands: str) -> tuple[bool, str]:
    if service["key"] == "tactile":
        if not reply.startswith("OK tactile_raw_recording "):
            return False, "回复不是 OK tactile_raw_recording"
        if _kv_str(reply, "capture_scope") != "tactile_only":
            return False, "capture_scope 不是 tactile_only"
        if _kv_int(reply, "pipeline_ready", -1) != 0:
            return False, "raw 阶段 pipeline_ready 标记异常"
    elif not reply.startswith("OK recording "):
        return False, "回复不是 OK recording"
    if _kv_str(reply, "session") != session:
        return False, f"回复 session={_kv_str(reply, 'session', 'missing')}，期望 {session}"
    if service["key"] != "tactile" and _kv_str(reply, "hands") != expected_hands:
        return False, f"回复 hands={_kv_str(reply, 'hands', 'missing')}，期望 {expected_hands}"
    return True, ""


def _confirm_started(service: dict, session: str,
                     timeout_s: float = 2.5) -> tuple[bool, bool, str]:
    """START 回包丢失时仅凭同一 session 的录制状态确认，绝不盲停其他会话。"""
    deadline = time.monotonic() + timeout_s
    last = "ERR no_status"
    while time.monotonic() < deadline:
        last, transport = _cmd(
            service["port"], "STATUS", service["label"], quiet=True, timeout=1.0)
        if transport and _service_is_recording(service, last, session):
            return True, True, last
        if transport and last.startswith("OK "):
            if service["key"] == "tactile":
                state = _kv_str(last, "state")
                if state in ("PAIRED_IDLE", "FAULT_LATCHED", "SHUTDOWN"):
                    return False, True, last
                if state == "RECORDING_RAW":
                    return False, True, last
            elif last.startswith(("OK idle", "OK REC")):
                return False, True, last
        time.sleep(0.15)
    return False, False, last


def _start_one(service: dict, command: str, session: str,
               expected_hands: str) -> tuple[bool, bool, bool]:
    """返回 (成功, 确认由本事务启动, 结果未知)。"""
    reply, transport = _cmd(service["port"], command, service["label"])
    valid, reason = (
        _valid_start_reply(service, reply, session, expected_hands)
        if transport else (False, "传输失败"))
    if valid:
        if service["key"] == "tactile":
            paths_ok, path_reason = _capture_tactile_paths(service, reply, session)
        else:
            paths_ok, path_reason = _capture_motion_path(service, reply, session)
        if not paths_ok:
            _safe_print(
                f"[!] {service['label']} START 已成功但输出路径不符合统一目录契约: "
                f"{path_reason}", flush=True)
            return False, True, False
        return True, True, False

    if transport:
        proves_started = (
            _kv_str(reply, "session") == session
            and ((service["key"] == "tactile"
                  and reply.startswith("OK tactile_raw_recording "))
                 or (service["key"] != "tactile"
                     and reply.startswith("OK recording "))))
        _safe_print(f"[!] {service['label']} START 协议失败: {reason}; reply={reply[:220]}",
                    flush=True)
        # 成功前缀+目标session证明本请求已启动，即使其余字段错误也必须纳入回滚。
        if proves_started:
            return False, True, False
        # ERR 也可能来自服务端“已打开文件/置录制状态”之后的异常，必须查状态诊断；
        # 但无 transaction token 时仍不盲目认领或停止同名会话。
        active, outcome_known, status = _confirm_started(service, session)
        outcome_unknown = active or not outcome_known
        if outcome_unknown:
            _safe_print(
                f"[!!!] {service['label']} 无效 START 回包后无法证明会话归属；"
                f"status={status[:220]}", flush=True)
        return False, False, outcome_unknown

    active, outcome_known, status = _confirm_started(service, session)
    if active:
        _safe_print(
            f"[!!!] {service['label']} START 回包丢失；STATUS 显示 session={session} 正在录，"
            "但无 transaction token，不能证明归属本请求。", flush=True)
        _safe_print("[!!!] 为避免误停并发客户端，本进程不会认领或盲目 STOP 该路。",
                    flush=True)
        return False, False, True
    if not outcome_known:
        _safe_print(f"[!!!] {service['label']} START 结果未知：回包和后续 STATUS 均不可达。",
                    flush=True)
        _safe_print(
            f"[!!!] 不会盲目 STOP 可能属于其他客户端的会话；"
            f"请人工复查端口和 session={session}。", flush=True)
    _safe_print(f"[!] {service['label']} START 协议失败: {reason}; status={status[:220]}",
                flush=True)
    return False, False, not outcome_known


def _valid_stop_reply(service: dict, reply: str,
                      expected_session: str | None,
                      require_nonempty: bool) -> tuple[bool, str, str]:
    stopped_session = _kv_str(reply, "session")
    if service["key"] == "tactile":
        if not reply.startswith("OK tactile_raw_stopped "):
            return False, "回复不是 OK tactile_raw_stopped", stopped_session
        if _kv_str(reply, "capture_scope") != "tactile_only":
            return False, "capture_scope 不是 tactile_only", stopped_session
        if _kv_int(reply, "capture_valid", 0) != 1:
            return False, "capture_valid!=1", stopped_session
        if _kv_int(reply, "pipeline_ready", -1) != 0:
            return False, "raw 阶段 pipeline_ready 标记异常", stopped_session
        frames = _parse_side_values(reply, "frames")
        if frames["left"] <= 0 or frames["right"] <= 0:
            return False, f"双侧帧数无效: {frames}", stopped_session
        if _kv_int(reply, "bytes", 0) <= 0:
            return False, "bytes<=0", stopped_session
        digest = _kv_str(reply, "sha256")
        if re.fullmatch(r"[0-9a-f]{64}", digest or "") is None:
            return False, "sha256 格式无效", stopped_session
    else:
        if not reply.startswith("OK stopped "):
            return False, "回复不是 OK stopped", stopped_session
        frames = _kv_int(reply, "frames", -1)
        if frames < 0:
            return False, "frames 缺失或无效", stopped_session
        if require_nonempty and frames == 0:
            return False, "正式采集 frames=0", stopped_session
    if not stopped_session:
        return False, "STOP 回复缺 session", stopped_session
    if expected_session is not None and stopped_session != expected_session:
        return False, f"STOP session={stopped_session}，期望 {expected_session}", stopped_session
    return True, "", stopped_session


def _validate_tactile_meta(service: dict, session: str, reply: str = "") -> tuple[bool, str]:
    paths = _session_paths(session)
    final_path = Path(service.get("file_path") or paths["tactile"])
    meta_path = Path(service.get("meta_path") or paths["tactile_meta"])
    partial_path = final_path.with_name("tactile.jsonl.partial")
    try:
        doc = json.loads(meta_path.read_text(encoding="utf-8"))
    except Exception as exc:  # noqa: BLE001
        return False, f"触觉 meta 不可读 {meta_path}: {type(exc).__name__}: {exc}"
    if not isinstance(doc, dict):
        return False, f"触觉 meta 顶层必须是对象，实际 {type(doc).__name__}"
    checks = (
        (doc.get("schema") == "pico_tactile_raw_meta_v1", "meta schema 不匹配"),
        (doc.get("state") == "complete", "meta state!=complete"),
        (doc.get("complete") is True, "meta complete!=true"),
        (doc.get("capture_valid") is True, "meta capture_valid!=true"),
        (doc.get("session") == session, "meta session 不匹配"),
        (doc.get("capture_scope") == "tactile_only", "meta capture_scope 不匹配"),
        (doc.get("data_file") == "tactile.jsonl", "meta data_file 不是 tactile.jsonl"),
        (final_path.is_file(), f"最终触觉文件不存在: {final_path}"),
        (not partial_path.exists() and not partial_path.is_symlink(),
         f"仍存在未封存触觉文件: {partial_path}"),
    )
    for passed, reason in checks:
        if not passed:
            return False, reason
    summary = doc.get("summary")
    if not isinstance(summary, dict) or summary.get("complete") is not True:
        return False, "meta summary 不完整"
    try:
        frames = summary.get("frames_by_side") or {}
        left_frames = int(frames.get("left") or 0)
        right_frames = int(frames.get("right") or 0)
        gap_frames = int(summary.get("subscription_gap_frames") or 0)
        writer_errors = int(summary.get("writer_errors") or 0)
        data_bytes = int(summary.get("data_bytes") or 0)
        actual_bytes = final_path.stat().st_size
    except (AttributeError, OSError, TypeError, ValueError) as exc:
        return False, f"meta summary 字段类型/文件状态无效: {type(exc).__name__}: {exc}"
    if left_frames <= 0 or right_frames <= 0:
        return False, "meta 双侧帧数无效"
    if gap_frames != 0:
        return False, "meta subscription_gap_frames 非零"
    if writer_errors != 0:
        return False, "meta writer_errors 非零"
    if data_bytes <= 0 or actual_bytes != data_bytes:
        return False, "meta/file 字节数不匹配"
    digest = str(summary.get("data_sha256") or "")
    if re.fullmatch(r"[0-9a-f]{64}", digest) is None:
        return False, "meta data_sha256 无效"
    actual_hash = hashlib.sha256()
    try:
        with final_path.open("rb") as stream:
            while True:
                chunk = stream.read(4 * 1024 * 1024)
                if not chunk:
                    break
                actual_hash.update(chunk)
    except OSError as exc:
        return False, f"触觉文件哈希复验失败: {exc}"
    if actual_hash.hexdigest() != digest:
        return False, "触觉文件实际 sha256 与 meta 不一致"
    if reply:
        reply_frames = _parse_side_values(reply, "frames")
        if (reply_frames["left"] != left_frames
                or reply_frames["right"] != right_frames):
            return False, "STOP 回复与 meta 帧数不一致"
        if _kv_int(reply, "bytes", -1) != data_bytes:
            return False, "STOP 回复与 meta 字节数不一致"
        if _kv_str(reply, "sha256") != digest:
            return False, "STOP 回复与 meta sha256 不一致"
    return True, ""


def _validate_motion_file(service: dict, session: str, reply: str,
                          require_nonempty: bool) -> tuple[bool, str]:
    match = re.search(r"\bfile=(.+)$", reply or "")
    if not match:
        return False, "STOP 回复缺 file 路径"
    try:
        reply_path = Path(match.group(1).strip())
        expected = _session_paths(session)[service["key"]].resolve()
        resolved = reply_path.resolve()
    except (OSError, RuntimeError, ValueError) as exc:
        return False, f"STOP 路径不可解析: {type(exc).__name__}: {exc}"
    if resolved != expected:
        return False, f"STOP file={reply_path}，期望 {expected}"
    try:
        size = expected.stat().st_size
    except OSError as exc:
        return False, f"采集文件不可读 {expected}: {exc}"
    if require_nonempty and size <= 0:
        return False, f"正式采集文件为空: {expected}"
    if service.get("require_vst"):
        valid, reason = _validate_vst_session(expected.parent, reply)
        if not valid:
            return False, reason
    return True, ""


def _validate_vst_session(raw_dir: Path, reply: str = "") -> tuple[bool, str]:
    """Cheap but strict VST integrity gate used immediately after STOP."""
    video = raw_dir / "vst.h264"
    wall = raw_dir / "vst.ts.jsonl"
    qpc = raw_dir / "vst.qpc.ts.jsonl"
    for path in (video, wall, qpc):
        try:
            if not path.is_file() or path.stat().st_size <= 0:
                return False, f"VST output missing or empty: {path}"
        except OSError as exc:
            return False, f"VST output is not readable: {path}: {exc}"
    try:
        with video.open("rb") as stream:
            head = stream.read(8 * 1024 * 1024)
        nal_types = set()
        for signature in (b"\x00\x00\x00\x01", b"\x00\x00\x01"):
            start = 0
            while True:
                pos = head.find(signature, start)
                if pos < 0:
                    break
                header = pos + len(signature)
                if header < len(head):
                    nal_types.add(head[header] & 0x1F)
                start = header
        if not {5, 7, 8}.issubset(nal_types):
            return False, "VST H.264 lacks decodable SPS/PPS/IDR data"
        wall_values = [int(line) for line in wall.read_text(
            encoding="utf-8").splitlines() if line.strip()]
        qpc_values = [int(line) for line in qpc.read_text(
            encoding="utf-8").splitlines() if line.strip()]
    except (OSError, UnicodeError, ValueError) as exc:
        return False, f"VST integrity parsing failed: {type(exc).__name__}: {exc}"
    if len(wall_values) < 5 or len(wall_values) != len(qpc_values):
        return False, ("VST timestamp sidecars are empty or disagree: "
                       f"wall={len(wall_values)} qpc={len(qpc_values)}")
    if any(a > b for a, b in zip(wall_values, wall_values[1:])):
        return False, "VST wall timestamps are not monotonic"
    if any(a > b for a, b in zip(qpc_values, qpc_values[1:])):
        return False, "VST QPC timestamps are not monotonic"
    reply_frames = _kv_int(reply, "vst_frames", -1)
    reply_bytes = _kv_int(reply, "vst_bytes", -1)
    if reply_frames >= 0 and reply_frames != len(wall_values):
        return False, ("VST STOP frame count disagrees with sidecar: "
                       f"reply={reply_frames} file={len(wall_values)}")
    try:
        video_bytes = video.stat().st_size
    except OSError as exc:
        return False, f"VST H.264 size check failed: {exc}"
    if reply_bytes >= 0 and reply_bytes != video_bytes:
        return False, ("VST STOP byte count disagrees with H.264 file: "
                       f"reply={reply_bytes} file={video_bytes}")
    expected_width = _kv_int(reply, "vst_width", 0)
    expected_height = _kv_int(reply, "vst_height", 0)
    try:
        probe = subprocess.run(
            ["ffprobe", "-v", "error", "-select_streams", "v:0",
             "-count_frames", "-show_entries",
             "stream=width,height,nb_read_frames", "-of", "json", str(video)],
            capture_output=True, text=True, encoding="utf-8", errors="replace",
            timeout=60.0, check=False,
        )
        if probe.returncode != 0 or probe.stderr.strip():
            return False, ("VST full-stream ffprobe failed: "
                           f"exit={probe.returncode} {probe.stderr.strip()[:300]}")
        streams = (json.loads(probe.stdout).get("streams") or [])
        if len(streams) != 1:
            return False, f"VST ffprobe found {len(streams)} video streams"
        stream = streams[0]
        actual_width = int(stream.get("width") or 0)
        actual_height = int(stream.get("height") or 0)
        decoded_frames = int(stream.get("nb_read_frames") or 0)
    except (OSError, subprocess.TimeoutExpired, ValueError, TypeError,
            json.JSONDecodeError) as exc:
        return False, f"VST ffprobe validation failed: {type(exc).__name__}: {exc}"
    if expected_width > 0 and actual_width != expected_width:
        return False, f"VST width mismatch: expected={expected_width} actual={actual_width}"
    if expected_height > 0 and actual_height != expected_height:
        return False, f"VST height mismatch: expected={expected_height} actual={actual_height}"
    if decoded_frames != len(wall_values):
        return False, ("VST decoded frame count disagrees with timestamps: "
                       f"decoded={decoded_frames} timestamps={len(wall_values)}")
    return True, ""


def _verify_services_idle(services: list[dict]) -> bool:
    ok = True
    for service in services:
        status, transport = _cmd(
            service["port"], "STATUS", service["label"], quiet=True, timeout=3.0)
        if not transport or not _service_is_idle(service, status):
            print(f"[!] {service['label']} 停录终态未确认: {status[:220]}", flush=True)
            ok = False
    return ok


def _stop_services(services: list[dict], *, expected_session: str | None,
                   tactile_timeout_s: float, concurrent: bool,
                   context: str, require_nonempty: bool) -> bool:
    """发送并严格校验 STOP；正常停录并发，START 回滚按传入的反序串行。"""
    if not services:
        return True

    def send_stop(service: dict) -> tuple[str, bool]:
        timeout = (tactile_timeout_s if service["key"] == "tactile"
                   else (50.0 if service["key"] == "pico" else 25.0))
        return _cmd(service["port"], "STOP", service["label"], quiet=True, timeout=timeout)

    results = {}
    if concurrent and len(services) > 1:
        with ThreadPoolExecutor(max_workers=len(services)) as pool:
            futures = {pool.submit(send_stop, service): service["key"] for service in services}
            for future, key in futures.items():
                try:
                    results[key] = future.result()
                except Exception as exc:  # noqa: BLE001
                    results[key] = (f"{type(exc).__name__}: {exc}", False)
    else:
        for service in services:
            results[service["key"]] = send_stop(service)

    all_ok = True
    stopped_sessions = []
    for service in services:
        reply, transport = results[service["key"]]
        print(f"[{service['label']}] {reply or '(空回复)'}", flush=True)
        valid, reason, stopped_session = (
            _valid_stop_reply(
                service, reply, expected_session, require_nonempty) if transport
            else (False, "STOP 传输失败/超时", "")
        )
        if valid and service["key"] == "tactile":
            valid, reason = _validate_tactile_meta(service, stopped_session, reply)
        elif valid:
            valid, reason = _validate_motion_file(
                service, stopped_session, reply, require_nonempty)
        if (not valid and not transport and service["key"] == "tactile"
                and expected_session is not None):
            status, status_ok = _cmd(
                service["port"], "STATUS", service["label"], quiet=True, timeout=3.0)
            meta_ok, meta_reason = _validate_tactile_meta(service, expected_session)
            if status_ok and _service_is_idle(service, status) and meta_ok:
                print("[TACTILE] STOP 回包虽丢失，但 PAIRED_IDLE + complete meta 已复验通过。",
                      flush=True)
                valid, reason = True, ""
            elif not meta_ok:
                reason += f"; {meta_reason}"
        if not valid:
            print(f"[!] {context}: {service['label']} STOP 未取得有效完成证明: {reason}",
                  flush=True)
            all_ok = False
        else:
            stopped_sessions.append(stopped_session)
    if expected_session is None and len(set(stopped_sessions)) > 1:
        print(f"[!] {context}: 各路停止的 session 不一致: {sorted(set(stopped_sessions))}",
              flush=True)
        all_ok = False
    if not _verify_services_idle(services):
        all_ok = False
    return all_ok


def _wait_tactile_first_frames(service: dict, session: str,
                                timeout_s: float = 2.5) -> tuple[bool, str]:
    """先确认左右各落一帧，保证后续事务回滚可被 collector 干净封存。"""
    deadline = time.monotonic() + timeout_s
    last = "ERR no_status"
    while time.monotonic() < deadline:
        last, transport = _cmd(
            service["port"], "STATUS", service["label"], quiet=True, timeout=1.0)
        if transport and _service_is_recording(service, last, session):
            frames = _parse_side_values(last, "frames")
            if (frames["left"] > 0 and frames["right"] > 0
                    and _kv_int(last, "raw_stream_valid", 0) == 1
                    and _kv_int(last, "errors", -1) == 0
                    and _kv_int(last, "gaps", -1) == 0):
                return True, last
        if transport and _kv_str(last, "state") in ("FAULT_LATCHED", "SHUTDOWN"):
            return False, last
        time.sleep(0.05)
    return False, last


def _start_transaction(services: list[dict], session: str, motion_command: str,
                       tactile_timeout_s: float,
                       cancel_flag: dict | None = None) -> list[dict] | None:
    started = []
    expected_hands = _kv_str(motion_command, "hands", "both")
    for service in services:
        if cancel_flag is not None and cancel_flag.get("flag"):
            _safe_print("[!] START 期间收到中断，回滚已启动服务。", flush=True)
            if started:
                _stop_services(
                    list(reversed(started)), expected_session=session,
                    tactile_timeout_s=tactile_timeout_s,
                    concurrent=False, context="START中断回滚", require_nonempty=False)
            return None
        command = f"START {session}" if service["key"] == "tactile" else motion_command
        if service["key"] != "tactile":
            service["expected_hands"] = expected_hands
        try:
            success, active, outcome_unknown = _start_one(
                service, command, session, expected_hands)
        except Exception as exc:  # noqa: BLE001
            _safe_print(
                f"[!!!] {service['label']} START 客户端内部异常，结果按未知处理: "
                f"{type(exc).__name__}: {exc}", flush=True)
            success, active, outcome_unknown = False, False, True
        if active:
            started.append(service)
        if success and service["key"] == "tactile":
            try:
                warm, warm_status = _wait_tactile_first_frames(service, session)
            except Exception as exc:  # noqa: BLE001
                warm = False
                warm_status = f"{type(exc).__name__}: {exc}"
            if not warm:
                _safe_print(
                    "[!] TACTILE START 后未确认左右各至少1帧；不启动后续模态。"
                    f" status={warm_status[:220]}", flush=True)
                success = False
        if cancel_flag is not None and cancel_flag.get("flag"):
            success = False
        if success:
            continue
        if started:
            _safe_print("[!] START 部分失败，按反向顺序回滚已确认启动的服务…", flush=True)
            _stop_services(
                list(reversed(started)), expected_session=session,
                tactile_timeout_s=tactile_timeout_s, concurrent=False,
                context="START回滚", require_nonempty=False)
        _safe_print(
            "[!] 本 session 可能已留下短采集/取证文件；确认全路空闲后由上层归档再重试。",
            flush=True)
        if outcome_unknown:
            _safe_print("[!!!] 当前失败服务可能仍在录制；在 STATUS 恢复前不要启动新采集。",
                        flush=True)
        return None
    return started


def _banner_record(session: str, hands: str, detach: bool, do_tactile: bool,
                   operator_control: bool = False,
                   require_vst: bool = True,
                   fault_grace_s: float = _DEFAULT_RECORD_FAULT_GRACE_S) -> None:
    paths = _session_paths(session)
    print(_BAR, flush=True)
    print(f"  录制中  session={session}  hands={hands}", flush=True)
    print("  坐标: 训练/可视化 = 右手系 X前Y左Z上", flush=True)
    video_outputs = (" + vst.h264 + vst.ts.jsonl + vst.qpc.ts.jsonl"
                     if require_vst else "")
    print(f"  产出: {paths['dir']}/{{pico,manus}}.jsonl{video_outputs}", flush=True)
    if do_tactile:
        print(f"        {paths['tactile_dir']}/tactile.jsonl + tactile.meta.json",
              flush=True)
        print("  触觉: 左右两侧全部369通道原始采集；当前仍为 pipeline_ready=0", flush=True)
    if detach:
        print("  模式: --detach (已发 START；另开终端 stop)", flush=True)
    elif operator_control:
        print("  控制: Enter=停止本条  H=重新采本条  Q=停止、导出并结束本批", flush=True)
        print("  防呆: 每0.5s检查全路；异常持续{:.1f}s则本条作废并安全停录".format(
            fault_grace_s), flush=True)
    else:
        print("  结束: Enter 或 Q → 自动 STOP", flush=True)
    print(_BAR, flush=True)


def _recording_issue(service: dict, status: str, transport: bool,
                     session: str, max_age_ms: int) -> str | None:
    if not transport or not status.startswith("OK "):
        return f"{service['label']} STATUS 离线/协议错误"
    if not _service_is_recording(service, status, session):
        return (f"{service['label']} 已不在录制目标 session；"
                f"state/session={_kv_str(status, 'state', status.split()[1] if len(status.split()) > 1 else '?')}"
                f"/{_kv_str(status, 'session', 'missing')}")
    if service["key"] != "tactile":
        expected_hands = str(service.get("expected_hands") or "both")
        if _kv_str(status, "hands") != expected_hands:
            return (f"{service['label']} 录制 hands={_kv_str(status, 'hands', 'missing')}，"
                    f"期望 {expected_hands}")
        if _kv_int(status, "writer_errors", 0) != 0:
            return (f"{service['label']} 录制写盘错误: "
                    f"writer_errors={_kv_int(status, 'writer_errors')}")
        write_backlog = _kv_int(status, "write_backlog", 0)
        if write_backlog > 512:
            return (f"{service['label']} 写盘队列严重积压: "
                    f"write_backlog={write_backlog}")
        if (service["key"] == "pico"
                and _kv_int(status, "vst_writer_errors", 0) != 0):
            return ("VST 录制写盘错误: "
                    f"vst_writer_errors={_kv_int(status, 'vst_writer_errors')}")
        if service.get("monitor_motion_freshness"):
            expected_sides = ("left", "right") if expected_hands == "both" else (expected_hands,)
            if service["key"] == "pico":
                if _kv_str(status, "devices") in ("", "(none)"):
                    return "PICO 录制中头显离线"
                pose_age = _kv_int(status, "pose_age_ms")
                if pose_age < 0 or pose_age > max_age_ms:
                    return f"PICO 录制中姿态过旧/中断: pose_age_ms={pose_age}"
                if _kv_int(status, "head", 0) != 1:
                    return "PICO 录制中 Head 数据消失"
                for side in expected_sides:
                    key = "ctrl_l" if side == "left" else "ctrl_r"
                    if _kv_int(status, key, 0) != 1:
                        return f"PICO 录制中 {side} 手柄数据消失"
                if service.get("require_vst"):
                    if _kv_int(status, "vst_recording", 0) != 1:
                        return "VST session writer disappeared during capture"
                    if _kv_int(status, "vst_record_ready", 0) != 1:
                        return "VST session writer lost its decodable start state"
                    backlog = _kv_int(status, "vst_write_backlog", 0)
                    if backlog > 120:
                        return f"VST writer backlog is unsafe: {backlog} frames"
                    vst_age = _kv_int(status, "vst_age_ms")
                    if _kv_int(status, "vst_on", 0) != 1 or not 0 <= vst_age <= max_age_ms:
                        return f"PICO 录制中 VST 中断/过旧: vst_age_ms={vst_age}"
            elif service["key"] == "manus":
                gloves = _kv_str(status, "gloves", "(none)")
                ages = _parse_manus_ages(status)
                for side in expected_sides:
                    if side not in gloves:
                        return f"MANUS 录制中 {side} 手套离线"
                    if ages[side] < 0 or ages[side] > max_age_ms:
                        return f"MANUS 录制中 {side} 数据过旧/中断: age_ms={ages[side]}"
    if service["key"] == "tactile":
        if _kv_int(status, "raw_stream_valid", 0) != 1:
            return "触觉录制中 raw_stream_valid!=1"
        if _kv_int(status, "errors", -1) != 0 or _kv_int(status, "gaps", -1) != 0:
            return (f"触觉录制中出现错误/缺帧: errors={_kv_int(status, 'errors')} "
                    f"gaps={_kv_int(status, 'gaps')}")
        if _kv_str(status, "fault", "missing") != "none":
            return f"触觉录制中 fault={_kv_str(status, 'fault', 'missing')}"
        ages = _parse_side_values(status, "age_ms")
        for side in ("left", "right"):
            if ages[side] < 0 or ages[side] > max_age_ms:
                return f"触觉 {side} 流中断/过旧: age_ms={ages[side]}"
    return None


def _compact_recording_status(service: dict, status: str, transport: bool) -> str:
    """Operator-facing one-line summary; full protocol text remains in errors."""
    label = service["label"]
    if not transport:
        return f"{label}:离线"
    if service["key"] == "pico":
        return (f"PICO:p{_kv_int(status, 'pose_age_ms')}ms "
                f"v{_kv_int(status, 'vst_age_ms')}ms/"
                f"{_kv_int(status, 'vst_record_frames', 0)} "
                f"q{_kv_int(status, 'write_backlog', 0)}/"
                f"{_kv_int(status, 'vst_write_backlog', 0)}")
    if service["key"] == "manus":
        ages = _parse_manus_ages(status)
        return (f"MANUS:lr{ages['left']}/{ages['right']}ms "
                f"n{_kv_int(status, 'frames', 0)} "
                f"q{_kv_int(status, 'write_backlog', 0)}")
    ages = _parse_side_values(status, "age_ms")
    frames = _parse_side_values(status, "frames")
    return (f"TACT:lr{ages['left']}/{ages['right']}ms "
            f"n{frames['left']}/{frames['right']} "
            f"w{_kv_int(status, 'warnings', 0)}")


def _wait_foreground(services: list[dict], session: str,
                     stop_flag: dict, max_age_ms: int,
                     operator_control: bool = False,
                     fault_grace_s: float = _DEFAULT_RECORD_FAULT_GRACE_S,
                     health_poll_s: float = _RECORD_HEALTH_POLL_S,
                     ) -> tuple[str | None, str]:
    def drain_windows_keys() -> None:
        if msvcrt is None:
            return
        while msvcrt.kbhit():
            key = msvcrt.getwch()
            if key in ("\x00", "\xe0") and msvcrt.kbhit():
                msvcrt.getwch()

    def poll_windows_control_key() -> str | None:
        if msvcrt is None:
            return None
        while msvcrt.kbhit():
            key = msvcrt.getwch()
            if key in ("\x00", "\xe0"):
                if msvcrt.kbhit():
                    msvcrt.getwch()
                continue
            if key in ("\r", "\n"):
                return "stop"
            if key in ("q", "Q"):
                return "quit" if operator_control else "stop"
            if operator_control and key in ("h", "H"):
                return "restart"
        return None

    t0 = time.monotonic()
    last = 0.0
    fault_since = None
    if sys.platform == "win32":
        # 丢弃就绪等待期间误按的键，防止刚开始录制就被旧 Enter 立即停止。
        drain_windows_keys()
    if operator_control:
        print("[*] 录制进行中… Enter=停止本条  H=重新采本条  Q=导出并结束本批", flush=True)
    else:
        print("[*] 录制进行中… (Enter 或 Q 结束)", flush=True)
    action = "stop"
    while not stop_flag["flag"]:
        if sys.platform == "win32":
            pressed_action = poll_windows_control_key()
            if pressed_action is not None:
                action = pressed_action
                labels = {
                    "stop": "Enter → 停止本条",
                    "restart": "H → 安全停止后重新采本条",
                    "quit": "Q → 停止本条、导出并结束本批",
                }
                print(f"[*] {labels[action]}", flush=True)
                break
            time.sleep(0.1)
        else:
            try:
                ready, _, _ = select.select([sys.stdin], [], [], 0.5)
            except (OSError, ValueError):
                ready = []
            if ready:
                try:
                    sys.stdin.readline()
                except Exception:  # noqa: BLE001
                    pass
                print("[*] Enter → 停录", flush=True)
                break
        now = time.monotonic()
        if now - last < health_poll_s:
            continue
        last = now
        parts = [f"  t={now - t0:5.1f}s"]
        issues = []
        for service in services:
            status, transport = _cmd(
                service["port"], "STATUS", service["label"], quiet=True,
                timeout=max(0.2, min(1.0, health_poll_s + 0.2)))
            issue = _recording_issue(service, status, transport, session, max_age_ms)
            parts.append(_compact_recording_status(service, status, transport))
            if issue is not None:
                issues.append(issue)
        had_pending_fault = fault_since is not None
        fault_since, confirmed, recovered = _advance_fault_window(
            fault_since, bool(issues), now, fault_grace_s,
        )
        if issues and not had_pending_fault:
            print("\n[sensor-check] 检测到传感器异常，进入连续异常确认："
                  + "；".join(issues), flush=True)
        if recovered:
            print("\n[sensor-recovered] 异常未持续达到门限，录制继续。", flush=True)
        if confirmed:
            issue_text = "；".join(issues)
            print("\n[!] 传感器异常持续 {:.1f}s，判定本条无效并进入全路 STOP：{}".format(
                fault_grace_s, issue_text), flush=True)
            return issue_text, "sensor_fault"
        if issues and fault_since is not None:
            parts.insert(1, "[SENSOR_WAIT {:.1f}/{:.1f}s]".format(
                now - fault_since, fault_grace_s,
            ))
        width = max(80, shutil.get_terminal_size((180, 24)).columns - 1)
        line = " | ".join(parts)
        if len(line) > width and not issues:
            # Narrow-window fallback keeps all three sensor groups visible.
            short = [f"t{now - t0:.0f}"]
            for item in parts[1:]:
                short.append(
                    item.replace("PICO:", "P:")
                        .replace("MANUS:", "M:")
                        .replace("TACT:", "T:")
                        .replace("ms", "")
                )
            line = " ".join(short)
        line = line[:width]
        sys.stdout.write("\r" + line.ljust(width))
        sys.stdout.flush()
    print(flush=True)
    return None, action


def _print_outputs(session: str, *, do_pico: bool, do_manus: bool,
                   do_tactile: bool, require_vst: bool) -> None:
    paths = _session_paths(session)
    raw = paths["dir"]
    print(_BAR, flush=True)
    print(f"  session={session}", flush=True)
    if do_pico or do_manus:
        print(f"  运动/视频目录: {raw}", flush=True)
        expected = []
        if do_pico:
            expected.append("pico.jsonl")
            if require_vst:
                expected.extend(("vst.h264", "vst.ts.jsonl", "vst.qpc.ts.jsonl"))
        if do_manus:
            expected.append("manus.jsonl")
        for name in expected:
            path = raw / name
            if path.is_file():
                print(f"    OK {name}  ({path.stat().st_size} bytes)", flush=True)
            else:
                print(f"    缺 {name}", flush=True)
    if do_tactile:
        tactile = paths["tactile_dir"]
        print(f"  触觉目录: {tactile}", flush=True)
        for name in ("tactile.jsonl", "tactile.meta.json"):
            path = tactile / name
            if path.is_file():
                print(f"    OK {name}  ({path.stat().st_size} bytes)", flush=True)
            else:
                print(f"    缺 {name}", flush=True)
    if sys.platform == "win32":
        pipeline_cmd = f"powershell -ExecutionPolicy Bypass -File .\\ego_ctl.ps1 pipeline {session}"
    else:
        pipeline_cmd = f"bash scripts/ego_ctl.sh pipeline {session}"
    print(f"  离线复验(包含同名完整触觉资产): {pipeline_cmd}", flush=True)
    print(_BAR, flush=True)


def _write_capture_failure(session: str, *, stage: str, reason: str,
                           max_age_ms: int, fault_grace_s: float) -> Path | None:
    """把运行中故障原因写进原始会话，供 rejected 归档和审计。"""
    raw_dir = _session_paths(session)["dir"]
    if not raw_dir.is_dir():
        return None
    target = raw_dir / "capture.failure.json"
    partial = raw_dir / "capture.failure.json.partial"
    payload = {
        "schema": "capture_failure_v1",
        "session": session,
        "outcome": "invalid_sensor_dropout",
        "stage": stage,
        "reason": str(reason),
        "detected_at": datetime.now().astimezone().isoformat(),
        "ready_max_age_ms": int(max_age_ms),
        "record_fault_grace_s": float(fault_grace_s),
        "policy": (
            "stop all routes; never export failed attempt; archive and retry same session index"
        ),
    }
    partial.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    partial.replace(target)
    return target


def main() -> None:
    ap = argparse.ArgumentParser(
        description="PICO+MANUS+双手触觉录制开关；start 默认三路严格就绪后再录。")
    ap.add_argument("action", choices=["start", "stop", "status", "ping"])
    ap.add_argument("name", nargs="?", default=None,
                    help="会话/任务名(start 时可选; 缺省用时间戳)")
    ap.add_argument("--pico-port", type=int, default=PICO_CONTROL_PORT)
    ap.add_argument("--manus-port", type=int, default=MANUS_CONTROL_PORT)
    ap.add_argument("--tactile-port", type=int, default=TACTILE_CONTROL_PORT)
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--pico-only", action="store_true")
    mode.add_argument("--manus-only", action="store_true")
    mode.add_argument("--no-tactile", action="store_true",
                      help="旧流程：只协调 PICO+MANUS，不联系触觉服务")
    ap.add_argument("--hands", default="both", choices=list(HANDS_CHOICES))
    ap.add_argument("--detach", action="store_true")
    ap.add_argument("--no-wait", action="store_true", help="同 --detach")
    ap.add_argument(
        "--operator-control", action="store_true",
        help=argparse.SUPPRESS,
    )
    ap.add_argument("--force", action="store_true",
                    help="只跳过开录前PICO/MANUS/VST新鲜度；录制中断流监控仍生效，且绝不跳过触觉配对/双路流")
    ap.add_argument("--ready-timeout", type=float, default=60.0,
                    help="等传感器就绪超时秒数；0=不限时(默认60)")
    ap.add_argument("--ready-max-age-ms", type=int, default=800,
                    help="姿态/手套/VST 最大允许陈旧毫秒(默认800)")
    ap.add_argument("--ready-stable-seconds", type=float,
                    default=_DEFAULT_READY_STABLE_S,
                    help="全部传感器连续稳定多久才允许开录(默认2秒)")
    ap.add_argument("--record-fault-grace-seconds", type=float,
                    default=_DEFAULT_RECORD_FAULT_GRACE_S,
                    help="录制中异常持续多久才判本条无效(默认1.5秒)")
    ap.add_argument("--no-require-vst", action="store_true",
                    help="开录前不要求 VST 暖机已出帧")
    ap.add_argument("--no-vst-recording", action="store_true",
                    help="本条不写 VST 文件（仍保留 PICO 位姿）")
    ap.add_argument("--tactile-stop-timeout", type=float,
                    default=_DEFAULT_TACTILE_STOP_TIMEOUT_S,
                    help="等待触觉STOP封存的客户端超时秒数(默认15；应大于采集器stop-timeout)")
    args = ap.parse_args()

    if args.action != "start" and args.name is not None:
        ap.error("session 名只允许用于 start；stop/status/ping 不接受位置参数")
    for label, port in (("PICO", args.pico_port), ("MANUS", args.manus_port),
                        ("TACTILE", args.tactile_port)):
        if not 1 <= port <= 65535:
            ap.error(f"{label} 控制端口必须在1..65535")
    if not math.isfinite(args.ready_timeout) or not 0 <= args.ready_timeout <= 3600:
        ap.error("--ready-timeout 必须是0..3600的有限秒数")
    if not 1 <= args.ready_max_age_ms <= 60000:
        ap.error("--ready-max-age-ms 必须在1..60000")
    if (not math.isfinite(args.ready_stable_seconds)
            or not 0.0 <= args.ready_stable_seconds <= 30.0):
        ap.error("--ready-stable-seconds 必须是0..30的有限秒数")
    if (not math.isfinite(args.record_fault_grace_seconds)
            or not 0.0 <= args.record_fault_grace_seconds <= 30.0):
        ap.error("--record-fault-grace-seconds 必须是0..30的有限秒数")
    if (not math.isfinite(args.tactile_stop_timeout)
            or not 5.0 < args.tactile_stop_timeout <= 120.0):
        ap.error("--tactile-stop-timeout 必须是大于5且不超过120的有限秒数")

    if args.pico_only:
        do_pico, do_manus, do_tactile = True, False, False
    elif args.manus_only:
        do_pico, do_manus, do_tactile = False, True, False
    else:
        do_pico, do_manus = True, True
        do_tactile = not args.no_tactile
    hands = parse_hands(args.hands)
    detach = bool(args.detach or args.no_wait)
    require_vst = (do_pico and not args.no_require_vst
                   and not args.no_vst_recording)
    services = _build_services(
        do_pico=do_pico, do_manus=do_manus, do_tactile=do_tactile,
        pico_port=args.pico_port, manus_port=args.manus_port,
        tactile_port=args.tactile_port)
    for service in services:
        # --force 只影响 START 前的等待。START 后所有已选传感器仍必须
        # 持续接受新数据，否则本条会按传感器断流作废。
        service["monitor_motion_freshness"] = True
        service["require_vst"] = bool(service["key"] == "pico" and require_vst)

    if args.action == "ping":
        ok = True
        for service in services:
            reply, transport = _cmd(service["port"], "PING", service["label"])
            ok &= transport and reply == "PONG"
        sys.exit(0 if ok else 1)

    if args.action == "status":
        ok = True
        for service in services:
            reply, transport = _cmd(service["port"], "STATUS", service["label"])
            protocol_ok = transport and reply.startswith("OK ")
            if service["key"] == "tactile":
                protocol_ok &= _kv_str(reply, "capture_scope") == "tactile_only"
            ok &= protocol_ok
        if do_tactile:
            print("提示: 触觉当前仅为独立 raw 资产，尚未进入 aligned/HDF5，pipeline_ready=0。",
                  flush=True)
        sys.exit(0 if ok else 1)

    if args.action == "stop":
        _safe_print("[*] 并发发送 STOP…", flush=True)
        ok = _stop_services(
            services, expected_session=None,
            tactile_timeout_s=args.tactile_stop_timeout,
            concurrent=True, context="独立STOP", require_nonempty=True)
        sys.exit(0 if ok else 1)

    # ---- start ----
    try:
        session = _validate_session_name(
            args.name or datetime.now().strftime("%Y%m%d_%H%M%S"))
    except ValueError as exc:
        print(f"[!] {exc}", flush=True)
        sys.exit(2)
    conflicts = _session_conflicts(session)
    if conflicts:
        print("[!] 拒绝复用已有 session；这会导致 PICO追加、MANUS覆盖或触觉误关联:",
              flush=True)
        for path in conflicts:
            print(f"    - {path}", flush=True)
        print("[!] 请使用新的 session 名。", flush=True)
        sys.exit(2)
    label = hands_label(hands)

    for service in services:
        reply, transport = _cmd(service["port"], "PING", service["label"])
        if not transport or reply != "PONG":
            print(f"[!] {service['label']} 服务尚未就绪，未创建本条数据。", flush=True)
            sys.exit(_EXIT_READY_TIMEOUT)

    if not args.force:
        if not _wait_ready(
            do_pico=do_pico, do_manus=do_manus, do_tactile=do_tactile,
            pico_port=args.pico_port, manus_port=args.manus_port,
            tactile_port=args.tactile_port,
            hands=hands, timeout_s=args.ready_timeout,
            max_age_ms=args.ready_max_age_ms, require_vst=require_vst,
            stable_s=args.ready_stable_seconds,
        ):
            sys.exit(_EXIT_READY_TIMEOUT)
    elif do_tactile:
        print("[!] --force: 仅跳过开录前 PICO/MANUS/VST 新鲜度；录制中断流监控仍生效，且仍严格等待触觉配对和双路流。",
              flush=True)
        if not _wait_ready(
            do_pico=do_pico, do_manus=do_manus, do_tactile=True,
            pico_port=args.pico_port, manus_port=args.manus_port,
            tactile_port=args.tactile_port,
            hands=hands, timeout_s=args.ready_timeout,
            max_age_ms=args.ready_max_age_ms, require_vst=False,
            skip_motion_freshness=True,
            stable_s=args.ready_stable_seconds,
        ):
            sys.exit(_EXIT_READY_TIMEOUT)
    else:
        print("[!] --force: 已跳过开录前 PICO/MANUS/VST 就绪检查；录制中断流监控仍生效。", flush=True)

    late_conflicts = _session_conflicts(session)
    if late_conflicts:
        print("[!] 就绪等待期间出现同名 session 目录，已在发送任何 START 前中止:", flush=True)
        for path in late_conflicts:
            print(f"    - {path}", flush=True)
        sys.exit(2)

    camera_params = None
    if do_pico:
        pico_status, transport = _cmd(
            args.pico_port, "STATUS", "PICO", quiet=True)
        if not transport:
            print("[!] 无法读取 PICO 设备序列号；未开始录制。", flush=True)
            sys.exit(_EXIT_READY_TIMEOUT)
        try:
            camera_params = select_camera_params(pico_status)
        except ValueError as exc:
            print(f"[!] {exc}；未开始录制。", flush=True)
            sys.exit(_EXIT_READY_TIMEOUT)

    # 始终显式传 hands，不能受 PICO/MANUS service 启动时的默认侧影响。
    motion_line = f"START {session} hands={label}"
    if args.no_vst_recording:
        motion_line += " video=0"

    stop_flag = {"flag": False}

    def _sig(_signal_number, _frame):
        if not stop_flag["flag"]:
            _safe_print("\n[*] 收到中断 → 准备停录…", flush=True)
        stop_flag["flag"] = True

    old_int = signal.signal(signal.SIGINT, _sig)
    old_term = signal.signal(signal.SIGTERM, _sig)
    try:
        started = _start_transaction(
            services, session, motion_line,
            tactile_timeout_s=args.tactile_stop_timeout,
            cancel_flag=stop_flag)
    except BaseException:
        signal.signal(signal.SIGINT, old_int)
        signal.signal(signal.SIGTERM, old_term)
        raise
    if started is None:
        signal.signal(signal.SIGINT, old_int)
        signal.signal(signal.SIGTERM, old_term)
        # START 与正式录制之间仍可能发生竞态掉线。只有确认所有服务都已回到空闲，
        # 且确有短采资产时，才允许上层按传感器故障归档并复用同一编号。
        safely_idle = _verify_services_idle(services)
        has_partial_assets = bool(_session_conflicts(session))
        if safely_idle and has_partial_assets:
            try:
                _write_capture_failure(
                    session, stage="start_transaction",
                    reason="START 事务期间一路失败，已确认全路回到空闲",
                    max_age_ms=args.ready_max_age_ms,
                    fault_grace_s=args.record_fault_grace_seconds,
                )
            except Exception as exc:  # noqa: BLE001
                print(f"[!] 写入失败标记失败: {type(exc).__name__}: {exc}", flush=True)
                sys.exit(1)
            sys.exit(_EXIT_SENSOR_FAULT)
        sys.exit(1)

    post_start_error = None
    try:
        if camera_params is not None:
            target, _meta = snapshot_camera_params(
                _session_paths(session)["dir"], camera_params)
            print(
                f"[PICO] 已保存相机标定快照: {target} "
                f"(device={camera_params.device_serial})",
                flush=True,
            )
        if require_vst and not args.force:
            if not _wait_vst_after_start(args.pico_port):
                raise RuntimeError("START 后 VST 未产生新帧")
        if not stop_flag["flag"]:
            _banner_record(
                session, label, detach, do_tactile,
                operator_control=args.operator_control,
                require_vst=require_vst,
                fault_grace_s=args.record_fault_grace_seconds)
            if detach:
                print("[*] 已后台录制。停录: python3 pico_record.py stop", flush=True)
                print(f"[*] 产出: {_session_paths(session)['dir']}/", flush=True)
                if do_tactile:
                    print(f"[*] 触觉: {_session_paths(session)['tactile_dir']}/", flush=True)
    except Exception as exc:  # noqa: BLE001
        post_start_error = exc

    if post_start_error is not None:
        # 即使 stdout 已断开，_stop_services 也会先并发发完 STOP，再打印/验证回复。
        rollback_ok = False
        try:
            rollback_ok = _stop_services(
                started, expected_session=session,
                tactile_timeout_s=args.tactile_stop_timeout,
                concurrent=True, context="START后异常回滚", require_nonempty=True)
        except Exception:
            pass
        finally:
            signal.signal(signal.SIGINT, old_int)
            signal.signal(signal.SIGTERM, old_term)
        try:
            print(f"[!] START 后异常，已触发全路 STOP: "
                  f"{type(post_start_error).__name__}: {post_start_error}", flush=True)
        except Exception:
            pass
        try:
            _write_capture_failure(
                session, stage="post_start_gate",
                reason=f"{type(post_start_error).__name__}: {post_start_error}",
                max_age_ms=args.ready_max_age_ms,
                fault_grace_s=args.record_fault_grace_seconds,
            )
        except Exception as exc:  # noqa: BLE001
            _safe_print(f"[!] 写入失败标记失败: {type(exc).__name__}: {exc}", flush=True)
            rollback_ok = False
        sys.exit(_EXIT_SENSOR_FAULT if rollback_ok else 1)

    if stop_flag["flag"]:
        _safe_print("[*] START/VST 阶段收到中断，执行全路 STOP…", flush=True)
        try:
            _stop_services(
                started, expected_session=session,
                tactile_timeout_s=args.tactile_stop_timeout,
                concurrent=True, context="启动阶段中断", require_nonempty=True)
        finally:
            signal.signal(signal.SIGINT, old_int)
            signal.signal(signal.SIGTERM, old_term)
        _print_outputs(
            session, do_pico=do_pico, do_manus=do_manus,
            do_tactile=do_tactile, require_vst=require_vst)
        sys.exit(1)

    if detach:
        signal.signal(signal.SIGINT, old_int)
        signal.signal(signal.SIGTERM, old_term)
        sys.exit(0)

    health_issue = None
    operator_action = "stop"
    wait_error = None
    stop_ok = False
    try:
        health_issue, operator_action = _wait_foreground(
            started, session, stop_flag, args.ready_max_age_ms,
            operator_control=args.operator_control,
            fault_grace_s=args.record_fault_grace_seconds)
    except Exception as exc:  # noqa: BLE001
        wait_error = exc
        _safe_print(
            f"[!] 前台监控异常，仍会执行全路 STOP: {type(exc).__name__}: {exc}",
            flush=True)
    finally:
        try:
            _safe_print("[*] 并发发送 STOP…", flush=True)
            stop_ok = _stop_services(
                started, expected_session=session,
                tactile_timeout_s=args.tactile_stop_timeout,
                concurrent=True, context="正式停录", require_nonempty=True)
        finally:
            signal.signal(signal.SIGINT, old_int)
            signal.signal(signal.SIGTERM, old_term)

    _print_outputs(
        session, do_pico=do_pico, do_manus=do_manus,
        do_tactile=do_tactile, require_vst=require_vst)
    if health_issue is not None:
        print(f"[!] 会话因运行中健康故障结束: {health_issue}", flush=True)
        try:
            marker = _write_capture_failure(
                session, stage="recording",
                reason=health_issue,
                max_age_ms=args.ready_max_age_ms,
                fault_grace_s=args.record_fault_grace_seconds,
            )
            if marker is not None:
                print(f"[!] 已写入失败标记: {marker}", flush=True)
        except Exception as exc:  # noqa: BLE001
            print(f"[!] 写入失败标记失败: {type(exc).__name__}: {exc}", flush=True)
            wait_error = wait_error or exc
    if wait_error is not None:
        print(f"[!] 会话因监控异常结束: {type(wait_error).__name__}: {wait_error}", flush=True)
    if not stop_ok:
        print("[!] 至少一路没有取得严格 STOP 完成证明，本会话不能标记为完整采集。", flush=True)
    if not stop_ok or wait_error is not None:
        sys.exit(1)
    if health_issue is not None:
        sys.exit(_EXIT_SENSOR_FAULT)
    if args.operator_control and operator_action == "restart":
        sys.exit(_EXIT_OPERATOR_RESTART)
    if args.operator_control and operator_action == "quit":
        sys.exit(_EXIT_OPERATOR_QUIT)
    sys.exit(0)


if __name__ == "__main__":
    main()
