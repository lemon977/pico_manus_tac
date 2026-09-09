#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pico_receiver.py — XRoboToolkit (PICO) 位姿数据 + VST 视频本地接收端

替代 RoboticsService（/opt/apps/roboticsservice/runService.sh）的最小实现：
  1. 每 5s 向局域网广播 UDP 29888（让头显 App 发现本机 IP）
  2. 监听 TCP 63901，接收头显主动连接
  3. 解析全部跟踪数据：头显位姿 / 双手柄位姿+按键 / 手势 26 关节 /
     全身动捕 24 关节 / Motion Tracker，打印摘要并完整落盘 JSONL
  4. --video 时向头显下发 RequestVRCamera，接收 VST 相机 H.264 流，
     用 ffplay 实时播放 和/或 保存 .h264 裸流文件

协议（来自 XR-Robotics/XRoboToolkit-PC-Service 与 XRoboToolkit-Unity-Client 源码）：
  帧格式: [head:1B][cmd:1B][length:u32le][payload][ts:u64le 秒级][tail:1B]
  头显→PC head=0x3F, PC→头显 head=0xCF, tail=0xA5
  cmd: 0x19/0x1A/0x1B=注册(SN|status) 0x23=心跳 0x6D=设备状态JSON 0x5F=PC下发控制JSON
  广播: head=0xCF cmd=0x7E payload=本机IP字符串, UDP 29888, 周期5s
  视频: PC 发 0x5F {"functionName":"RequestVRCamera","value":"{on,port,width,height,fps,...}"}
        头显随即作为 TCP client 连 PC 的指定 port, 码流为 [4B 大端 len][一个 H.264
        Annex-B access unit] 连续写入; SPS/PPS 随 IDR 内联(1s 一个 IDR);
        默认 4096x1536@30 左右眼水平拼接(左半=左眼)

用法: python3 pico_receiver.py [--print-hz 30] [--video [PORT]] [--video-save f.h264]
"""

import argparse
import json
import queue
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

from pico_retarget import Retargeter
from record_control import (
    ControlServer, PICO_CONTROL_PORT,
    parse_hands, hands_label, parse_start_arg, filter_pico_record,
    HANDS_BOTH, HANDS_CHOICES, wait_queue_drained,
)
from session_layout import session_raw_dir

HEAD_CLIENT = 0x3F          # 头显 -> PC
HEAD_SERVER = 0xCF          # PC -> 头显
TAIL = 0xA5

CMD_CONNECT = 0x19          # 注册: "SN|status"
CMD_BATTERY = 0x1A
CMD_SENSOR = 0x1B
CMD_HEARTBEAT = 0x23
CMD_STATE_JSON = 0x6D       # 设备状态/跟踪 JSON
CMD_BYTES_TO_PC = 0x72
CMD_SERVER_CONTROL_JSON = 0x5F   # PC -> 头显: 控制 JSON

BCAST_CMD_TCPIP = 0x7E
BCAST_UDP_PORT = 29888
BCAST_INTERVAL = 5.0
TCP_PORT = 63901
VIDEO_DEFAULT_PORT = 63902

CMD_NAMES = {
    CMD_CONNECT: "CONNECT", CMD_BATTERY: "BATTERY", CMD_SENSOR: "SENSOR",
    CMD_HEARTBEAT: "HEARTBEAT", CMD_STATE_JSON: "STATE_JSON",
    CMD_BYTES_TO_PC: "BYTES_TO_PC",
}

# 手势 26 关节名（HandJointLocations 数组顺序）
HAND_JOINTS = [
    "Palm", "Wrist",
    "Thumb_metacarpal", "Thumb_proximal", "Thumb_distal", "Thumb_tip",
    "Index_metacarpal", "Index_proximal", "Index_intermediate", "Index_distal", "Index_tip",
    "Middle_metacarpal", "Middle_proximal", "Middle_intermediate", "Middle_distal", "Middle_tip",
    "Ring_metacarpal", "Ring_proximal", "Ring_intermediate", "Ring_distal", "Ring_tip",
    "Little_metacarpal", "Little_proximal", "Little_intermediate", "Little_distal", "Little_tip",
]

# 全身动捕 24 关节（BodyTrackerRole）
BODY_JOINTS = [
    "Pelvis", "LEFT_HIP", "RIGHT_HIP", "SPINE1", "LEFT_KNEE", "RIGHT_KNEE",
    "SPINE2", "LEFT_ANKLE", "RIGHT_ANKLE", "SPINE3", "LEFT_FOOT", "RIGHT_FOOT",
    "NECK", "LEFT_COLLAR", "RIGHT_COLLAR", "HEAD", "LEFT_SHOULDER", "RIGHT_SHOULDER",
    "LEFT_ELBOW", "RIGHT_ELBOW", "LEFT_WRIST", "RIGHT_WRIST", "LEFT_HAND", "RIGHT_HAND",
]

state_lock = threading.Lock()
devices = {}        # sn -> {"latest": dict, "frames": int, "last_rx": float, "addr": str}
device_conns = {}   # sn -> DeviceConn


class Recorder:
    """门控落盘: START 打开以 session 命名的 jsonl 开始写, STOP 关闭。
    非 service 模式下启动即 start(), 行为与旧版一致(连上就一直录)。"""

    def __init__(self, log_dir: Path, video_rx=None, video_inner=None,
                 video_autosave=False, default_hands=None):
        self.log_dir = Path(log_dir)
        self.video_rx = video_rx            # 可选 VideoReceiver
        self.video_inner = video_inner      # RequestVRCamera 的 value 内层(不含 on)
        self.video_autosave = video_autosave  # service: 每会话自动存 vst_<session>.h264
        self.default_hands = (parse_hands(default_hands)
                              if default_hands is not None else HANDS_BOTH)
        self._lock = threading.Lock()
        self._video_command_lock = threading.Lock()
        self._fp = None
        self._active = False
        self._stopping = False
        self.session = None
        self.count = 0
        self.jsonl_path = None
        self.hands = self.default_hands
        self.writer_errors = 0
        self.max_write_backlog = 0
        self.last_video_frames = 0
        self.last_video_bytes = 0
        self._video_recovery_generation = 0
        self._video_recovery_active = False
        self._write_queue = queue.Queue(maxsize=32768)
        self._writer_thread = threading.Thread(
            target=self._writer_loop, name="pico-jsonl-writer", daemon=True
        )
        self._writer_thread.start()

    def _writer_loop(self) -> None:
        last_flush = time.monotonic()
        while True:
            fp, session, rec, hands = self._write_queue.get()
            try:
                out = filter_pico_record(rec, hands)
                line = json.dumps(out, ensure_ascii=False) + "\n"
                fp.write(line)
                with self._lock:
                    if session == self.session:
                        self.count += 1
                now = time.monotonic()
                if now - last_flush >= 0.5:
                    fp.flush()
                    last_flush = now
            except Exception as exc:  # noqa: BLE001
                with self._lock:
                    self.writer_errors += 1
                event(f"[!] PICO JSONL 异步写盘失败: {exc!r}")
            finally:
                self._write_queue.task_done()

    def start(self, session: str, hands=None, record_video: bool = True) -> str:
        h = parse_hands(hands) if hands is not None else self.default_hands
        with self._lock:
            if self._active or self._stopping:
                return f"ERR already recording session={self.session}"
            # 任务优先布局: <sessions>/<task>/<index>/raw/pico.jsonl (+ vst.h264)
            sess_dir = session_raw_dir(self.log_dir, session)
            sess_dir.mkdir(parents=True, exist_ok=True)
            self.session = session
            self.hands = h
            self.jsonl_path = sess_dir / "pico.jsonl"
            self._fp = open(self.jsonl_path, "a", encoding="utf-8")
            self._active = True
            self.count = 0
            self.writer_errors = 0
            self.max_write_backlog = 0
        # 视频: 设落盘路径后重启推流(service 空闲时可能已在暖机推流、未落盘)
        if self.video_rx is not None and record_video:
            self.video_rx.writer_errors = 0
            self.video_rx.max_write_backlog = 0
            self.last_video_frames = 0
            self.last_video_bytes = 0
            video_path = (str(sess_dir / "vst.h264") if self.video_autosave
                          else self.video_rx.save_path)
            if video_path:
                try:
                    self.video_rx.start_recording(video_path)
                except OSError:
                    with self._lock:
                        self._active = False
                        self.session = None
                        if self._fp is not None:
                            self._fp.close()
                            self._fp = None
                    raise
                self._video_recovery_generation += 1
                generation = self._video_recovery_generation
                threading.Thread(
                    target=self._ensure_video_recording,
                    args=(session, generation),
                    name="pico-vst-session-start",
                    daemon=True,
                ).start()
        hl = hands_label(h)
        event(f">>> 开始录制 session={session} hands={hl} -> {self.jsonl_path}")
        return f"OK recording session={session} hands={hl} file={self.jsonl_path}"

    def _ensure_video_recording(self, session: str, generation: int) -> None:
        """Recover only when the warm stream cannot supply SPS/PPS + an IDR."""
        if self.video_rx.wait_recording_ready(2.5):
            return
        delays = (8.0, 12.0, 18.0)
        for attempt, wait_s in enumerate(delays, 1):
            with self._lock:
                current = (self._active and self.session == session and
                           self._video_recovery_generation == generation)
            if not current or not self.video_rx.recording_pending():
                return
            event(f"[VST] session header/IDR timeout; controlled reconnect {attempt}/3")
            self._video_recovery_active = True
            try:
                self._video_cmd(on=False)
                time.sleep(0.6)
                with self._lock:
                    current = (self._active and self.session == session and
                               self._video_recovery_generation == generation)
                if not current:
                    return
                self.video_rx.reset_codec_config()
                self._video_cmd(on=True)
                ready = self.video_rx.wait_recording_ready(wait_s)
            finally:
                self._video_recovery_active = False
            if ready:
                return
        if self.video_rx.recording_pending():
            self.video_rx.writer_errors += 1
            self.video_rx._record_error = "SPS/PPS + IDR timeout after 3 reconnects"
            self.video_rx._record_ready.set()
            event("[!] VST session could not obtain SPS/PPS + IDR")

    def stop(self) -> str:
        # 先关闭入队门禁，再停 VST；最后等待异步队列完全落盘。
        with self._lock:
            if not self._active:
                return "ERR not recording"
            self._active = False
            self._stopping = True
            sess = self.session
            path = self.jsonl_path
        if self.video_rx is not None:
            self._video_recovery_generation += 1
            try:
                (self.last_video_frames,
                 self.last_video_bytes) = self.video_rx.stop_recording()
            except OSError as exc:
                self.video_rx.writer_errors += 1
                event(f"[!] VST writer finalization failed: {exc}")
        if not wait_queue_drained(self._write_queue, 20.0):
            with self._lock:
                self.writer_errors += 1
                fp = self._fp
            event("[!] PICO JSON writer drain exceeded 20s; finalizing in background")
            threading.Thread(
                target=self._finish_delayed_stop,
                args=(fp, sess),
                name="pico-jsonl-delayed-stop",
                daemon=True,
            ).start()
            return (f"ERR writer_errors={self.writer_errors} stopped session={sess} "
                    f"frames={self.count} file={path}")
        with self._lock:
            n = self.count
            if self._fp is not None:
                self._fp.flush()
                self._fp.close()
                self._fp = None
            errors = self.writer_errors
            video_errors = self.video_rx.writer_errors if self.video_rx is not None else 0
            self.session = None
            self._stopping = False
        event(f"<<< 停止录制 session={sess} 共 {n} 帧 -> {path}")
        target = self.video_inner or {}
        video_spec = (f"vst_width={int(target.get('width', 0))} "
                      f"vst_height={int(target.get('height', 0))} "
                      f"vst_fps={int(target.get('fps', 0))} ")
        if errors or video_errors:
            return (f"ERR writer_errors={errors} vst_writer_errors={video_errors} "
                    f"stopped session={sess} "
                    f"frames={n} vst_frames={self.last_video_frames} "
                    f"vst_bytes={self.last_video_bytes} {video_spec}file={path}")
        return (f"OK stopped session={sess} frames={n} "
                f"vst_frames={self.last_video_frames} "
                f"vst_bytes={self.last_video_bytes} {video_spec}file={path}")

    def _finish_delayed_stop(self, fp, session: str) -> None:
        self._write_queue.join()
        with self._lock:
            if self._fp is fp and fp is not None:
                try:
                    fp.flush()
                    fp.close()
                except OSError as exc:
                    self.writer_errors += 1
                    event(f"[!] delayed PICO file close failed: {exc}")
                self._fp = None
            if self.session == session:
                self.session = None
            self._stopping = False
        event(f"[PICO] delayed writer finalization completed: session={session}")

    def write(self, rec: dict, recv_wall_ns: int | None = None,
              recv_qpc_ns: int | None = None) -> None:
        wall_ns = int(recv_wall_ns or time.time_ns())
        qpc_ns = int(recv_qpc_ns or time.perf_counter_ns())
        rec["recv_ts"] = wall_ns / 1e9
        rec["recv_wall_ns"] = wall_ns
        rec["recv_qpc_ns"] = qpc_ns
        with self._lock:
            if self._active and self._fp is not None:
                try:
                    self._write_queue.put_nowait(
                        (self._fp, self.session, rec, set(self.hands))
                    )
                    self.max_write_backlog = max(
                        self.max_write_backlog, self._write_queue.qsize()
                    )
                except queue.Full:
                    self.writer_errors += 1
                    event("[!] PICO JSONL 写盘队列溢出；本会话将拒绝封存")

    @staticmethod
    def _ctrl_ok(ctrl: dict, side: str) -> bool:
        if not isinstance(ctrl, dict):
            return False
        for key in (side, side.capitalize(), "Left" if side == "left" else "Right"):
            c = ctrl.get(key)
            if isinstance(c, dict) and (c.get("pos") is not None or c.get("pose") is not None):
                return True
        return False

    def status(self) -> str:
        now = time.time()
        with state_lock:
            online = sorted(devices.keys())
            best = None
            for sn in online:
                d = devices.get(sn) or {}
                if best is None or float(d.get("last_rx") or 0) > float(best.get("last_rx") or 0):
                    best = d
        age_ms = -1
        has_head = has_l = has_r = False
        if best:
            age_ms = int(max(0.0, (now - float(best.get("last_rx") or 0)) * 1000))
            latest = best.get("latest") or {}
            if isinstance(latest, dict):
                h = latest.get("Head")
                has_head = isinstance(h, dict) and (
                    h.get("pos") is not None or h.get("pose") is not None)
                ctrl = latest.get("Controller") or {}
                has_l = self._ctrl_ok(ctrl, "left")
                has_r = self._ctrl_ok(ctrl, "right")
        vst_frames = 0
        vst_age_ms = -1
        vst_on = 0
        vst_writer_errors = 0
        vst_write_backlog = 0
        vst_retries = 0
        vst_protocol_errors = 0
        vst_recording = 0
        vst_record_ready = 0
        vst_record_pending = 0
        vst_record_frames = 0
        vst_record_bytes = 0
        if self.video_rx is not None:
            vst_frames = int(self.video_rx.frames)
            vst_on = 1 if self.video_rx.streaming else 0
            if self.video_rx.last_rx > 0:
                vst_age_ms = int(max(0.0, (now - self.video_rx.last_rx) * 1000))
            vst_writer_errors = int(self.video_rx.writer_errors)
            vst_retries = int(self.video_rx.reconnect_attempts)
            vst_protocol_errors = int(self.video_rx.protocol_errors)
            with self.video_rx._disk_writer_lock:
                writer = self.video_rx._disk_writer
                vst_write_backlog = int(writer.queue.qsize() if writer is not None else 0)
                vst_recording = int(writer is not None)
                vst_record_pending = int(
                    writer is not None and self.video_rx._record_pending)
                vst_record_ready = int(
                    writer is not None and not self.video_rx._record_pending
                    and self.video_rx._record_ready.is_set())
                if writer is not None:
                    vst_record_frames = int(writer.picture_frames)
                    vst_record_bytes = int(writer.bytes_written)
        with self._lock:
            rec = "STOPPING" if self._stopping else ("REC" if self._active else "idle")
            hl = hands_label(self.hands if self._active else self.default_hands)
            target = self.video_inner or {}
            vst_target = (f"{target.get('width', 0)}x{target.get('height', 0)}"
                          f"@{target.get('fps', 0)}")
            return (
                f"OK {rec} session={self.session} frames={self.count} "
                f"hands={hl} "
                f"devices={','.join(online) if online else '(none)'} "
                f"pose_age_ms={age_ms} head={int(has_head)} "
                f"ctrl_l={int(has_l)} ctrl_r={int(has_r)} "
                f"vst_on={vst_on} vst_frames={vst_frames} vst_age_ms={vst_age_ms} "
                f"writer_errors={self.writer_errors} "
                f"write_backlog={self._write_queue.qsize()} "
                f"vst_writer_errors={vst_writer_errors} "
                f"vst_write_backlog={vst_write_backlog} "
                f"vst_retries={vst_retries} "
                f"vst_protocol_errors={vst_protocol_errors} "
                f"vst_recording={vst_recording} "
                f"vst_record_ready={vst_record_ready} "
                f"vst_record_pending={vst_record_pending} "
                f"vst_record_frames={vst_record_frames} "
                f"vst_record_bytes={vst_record_bytes} "
                f"vst_target={vst_target}"
            )

    def _video_cmd(self, on: bool) -> None:
        with self._video_command_lock:
            with state_lock:
                conns = list(device_conns.values())
            inner = dict(self.video_inner or {})
            inner["on"] = 1 if on else 0
            for dev in conns:
                try:
                    dev.send_control_json({"functionName": "RequestVRCamera",
                                           "value": json.dumps(inner)})
                except OSError:
                    pass


RECORDER = None     # 全局 Recorder, main() 里创建

from collections import deque
EVENTS = deque(maxlen=5)      # 最近事件（随状态区一起重绘，不滚屏）
INFO_LINES = []               # 状态区顶部固定信息（IP/端口/日志路径）
PRINT_HZ0 = True              # print_hz<=0 时事件直接滚屏打印
REDRAW_STARTED = False


def event(msg):
    """记录一条事件: 重绘模式下进事件区, 非重绘模式直接打印"""
    global REDRAW_STARTED
    line = f"[{datetime.now():%H:%M:%S}] {msg}"
    with state_lock:
        EVENTS.append(line)
    if PRINT_HZ0 or not REDRAW_STARTED:
        print(line, flush=True)


def render_status(text):
    """重绘状态区: 数据摘要 + 最近事件 (print_hz>0 时使用)。
    固定信息(IP/端口)只在启动时打印一次, 不进重绘块, 避免终端矮时滚屏。"""
    global REDRAW_STARTED
    if PRINT_HZ0:
        return
    REDRAW_STARTED = True
    with state_lock:
        ev = list(EVENTS)
    out = (text + "\n\n-- 最近事件 --\n" + ("\n".join(ev) if ev else "(无)"))
    print("\033[H\033[J" + out, flush=True)


def pack_frame(head, cmd, payload: bytes) -> bytes:
    return (struct.pack("<BBI", head, cmd, len(payload)) + payload
            + struct.pack("<QB", int(time.time()), TAIL))


def parse_pose(s):
    """'x,y,z,qx,qy,qz,qw' -> ([x,y,z],[qx,qy,qz,qw])，解析失败返回原样"""
    try:
        v = [float(x) for x in s.split(",")]
        if len(v) == 7:
            return v[:3], v[3:]
        if len(v) == 6:
            return v[:3], v[3:]
    except (ValueError, AttributeError):
        pass
    return s, None


def parse_tracking(value: dict) -> dict:
    """把 value 里的位姿字符串解析成数组，保留所有原始字段"""
    out = dict(value)
    if "Head" in value and isinstance(value["Head"], dict):
        h = dict(value["Head"])
        if "pose" in h:
            h["pos"], h["quat"] = parse_pose(h["pose"])
        out["Head"] = h
    if "Controller" in value and isinstance(value["Controller"], dict):
        c = dict(value["Controller"])
        for side in ("left", "right"):
            if isinstance(c.get(side), dict) and "pose" in c[side]:
                d = dict(c[side])
                d["pos"], d["quat"] = parse_pose(d["pose"])
                c[side] = d
        out["Controller"] = c
    for hand_key in ("leftHand", "rightHand"):
        pass  # 手势关节较多，保持原始结构，仅完整落盘
    return out


def fmt_pose(pos, quat):
    if not isinstance(pos, list):
        return "  (无位姿)"
    p = " ".join(f"{x:+.3f}" for x in pos)
    q = " ".join(f"{x:+.3f}" for x in quat) if isinstance(quat, list) else "-"
    return f"  pos[{p}] quat[{q}]"


def _quat_axes(quat):
    """四元数[qx,qy,qz,qw] -> 该坐标系局部 X/Y/Z 轴在世界系中的方向(单位向量)。"""
    if not (isinstance(quat, list) and len(quat) == 4):
        return None
    x, y, z, w = quat
    n = (x * x + y * y + z * z + w * w) ** 0.5
    if n < 1e-9:
        return None
    x, y, z, w = x / n, y / n, z / n, w / n
    xd = [1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w)]
    yd = [2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w)]
    zd = [2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)]
    return xd, yd, zd


def fmt_axes(quat):
    """把局部三轴的世界方向排成一行, 供 MANUS xyz 对齐参考; quat 无效返回空串。"""
    ax = _quat_axes(quat)
    if ax is None:
        return ""
    def f(v):
        return "[" + ",".join(f"{c:+.2f}" for c in v) + "]"
    xd, yd, zd = ax
    return f"       轴(世界系) X→{f(xd)} Y→{f(yd)} Z→{f(zd)}"


def _dist(a, b):
    if not (isinstance(a, list) and isinstance(b, list) and len(a) == 3 and len(b) == 3):
        return None
    return sum((a[i] - b[i]) ** 2 for i in range(3)) ** 0.5


def summarize(dev_sn, d, retargeted=None, frames=0):
    """生成一帧的紧凑摘要文本"""
    lines = [f"[{datetime.now():%H:%M:%S}] {dev_sn} "
             f"frames={frames} ts={d.get('timeStampNs', '-')}"]
    head = d.get("Head")
    head_pos = None
    if isinstance(head, dict) and isinstance(head.get("pos"), list):
        head_pos = head.get("pos")
        lines.append("HEAD " + fmt_pose(head_pos, head.get("quat"))
                     + f" status={head.get('status')} handMode={head.get('handMode')}")
        ax = fmt_axes(head.get("quat"))
        if ax:
            lines.append(ax)
    else:
        lines.append("HEAD  ⚠ 无头位姿(PICO 未上报 Head) → 可视化里头三轴/骨架不显示; "
                     "请确认头显已正常佩戴且头部6DoF在跟踪")
    positions = {}
    ctrl = d.get("Controller")
    if isinstance(ctrl, dict):
        for side, label in (("left", "L"), ("right", "R")):
            c = ctrl.get(side)
            if isinstance(c, dict):
                btn = []
                if c.get("primaryButton"): btn.append("A/X")
                if c.get("secondaryButton"): btn.append("B/Y")
                if c.get("menuButton"): btn.append("MENU")
                if c.get("axisClick"): btn.append("STICK")
                lines.append(
                    f"CTRL-{label} " + fmt_pose(c.get("pos"), c.get("quat"))
                    + f" trig={c.get('trigger', 0):.2f} grip={c.get('grip', 0):.2f}"
                    + f" axis=({c.get('axisX', 0):+.2f},{c.get('axisY', 0):+.2f})"
                    + (" " + ",".join(btn) if btn else ""))
                ax = fmt_axes(c.get("quat"))
                if ax:
                    lines.append(ax)
                if isinstance(c.get("pos"), list):
                    positions[side] = c.get("pos")
    # 世界系距离: 头↔手柄、手柄↔手柄; 某值长期恒定=该源卡死/未跟踪
    seg = []
    dl = _dist(head_pos, positions.get("left"))
    dr = _dist(head_pos, positions.get("right"))
    dlr = _dist(positions.get("left"), positions.get("right"))
    if dl is not None: seg.append(f"|头-左|={dl * 100:.1f}cm")
    if dr is not None: seg.append(f"|头-右|={dr * 100:.1f}cm")
    if dlr is not None: seg.append(f"|左-右|={dlr * 100:.1f}cm")
    if seg:
        lines.append("距离(世界系): " + "  ".join(seg))
    if retargeted is not None:
        lw = retargeted["left_wrist_rel"]
        rw = retargeted["right_wrist_rel"]
        lines.append("RETARGET(机器人头系 相对位姿):")
        lines.append("  L手腕 " + fmt_pose(lw["pos"], lw["quat"]))
        lines.append("  R手腕 " + fmt_pose(rw["pos"], rw["quat"]))
        lines.append("  torso " + fmt_pose(retargeted["torso_rel"]["pos"],
                                            retargeted["torso_rel"]["quat"]))
    for hk, label in (("leftHand", "左手势"), ("rightHand", "右手势")):
        h = d.get(hk)
        if isinstance(h, dict):
            lines.append(f"{label}: active={h.get('isActive')} joints={h.get('count')}")
    body = d.get("Body")
    if isinstance(body, dict):
        lines.append(f"全身动捕: joints={len(body.get('joints', []))}")
    motion = d.get("Motion")
    if isinstance(motion, dict):
        lines.append(f"Motion Tracker: n={motion.get('len')} sn={motion.get('sn')}")
    return "\n".join(lines)


class DeviceConn(threading.Thread):
    def __init__(self, conn, addr, print_hz, on_online=None, retarget=True,
                 viz=None):
        super().__init__(daemon=True)
        self.conn = conn
        self.addr = addr
        self.sn = None
        self.on_online = on_online
        self.retargeter = Retargeter() if retarget else None
        self.viz = viz          # 可选 ControllerVisualizer (--viz / --viz-raw)
        self.last_print = 0.0
        self.print_interval = 1.0 / print_hz if print_hz > 0 else float("inf")

    def send_control_json(self, obj: dict):
        """PC -> 头显 下发控制 JSON (cmd 0x5F)，value 为字符串化内层 JSON"""
        payload = json.dumps(obj, ensure_ascii=False).encode()
        self.conn.sendall(pack_frame(HEAD_SERVER, CMD_SERVER_CONTROL_JSON, payload))

    def log(self, rec, recv_wall_ns=None, recv_qpc_ns=None):
        # 落盘经全局 Recorder 门控。时间戳在完整协议帧从 TCP 缓冲区取出后、
        # JSON 解析和磁盘写入前生成。
        if RECORDER is not None:
            RECORDER.write(rec, recv_wall_ns=recv_wall_ns,
                           recv_qpc_ns=recv_qpc_ns)

    def handle_payload(self, cmd, payload: bytes,
                       recv_wall_ns=None, recv_qpc_ns=None):
        name = CMD_NAMES.get(cmd, f"0x{cmd:02X}")
        if cmd in (CMD_CONNECT, CMD_BATTERY, CMD_SENSOR) and self.sn is None:
            self.sn = payload.split(b"|")[0].decode("utf-8", "replace")
            with state_lock:
                devices[self.sn] = {"frames": 0, "last_rx": time.time(),
                                    "addr": f"{self.addr[0]}:{self.addr[1]}"}
                device_conns[self.sn] = self
            event(f">>> 设备上线 SN={self.sn} from {self.addr[0]} (cmd={name})")
            if self.on_online:
                try:
                    self.on_online(self)
                except Exception as e:
                    print(f"[!] on_online 回调失败: {e}")
            return

        if cmd == CMD_HEARTBEAT:
            return

        if cmd == CMD_BYTES_TO_PC:
            self.log({"dev": self.sn, "cmd": name, "bytes_hex": payload.hex()},
                     recv_wall_ns, recv_qpc_ns)
            return

        if cmd == CMD_STATE_JSON:
            try:
                outer = json.loads(payload.decode("utf-8", "replace"))
            except json.JSONDecodeError:
                self.log({"dev": self.sn, "cmd": name, "raw": payload.hex()},
                         recv_wall_ns, recv_qpc_ns)
                return
            fn = outer.get("functionName")
            value = outer.get("value")
            if isinstance(value, str):
                try:
                    value = json.loads(value)
                except json.JSONDecodeError:
                    pass
            parsed = parse_tracking(value) if isinstance(value, dict) else value
            retargeted = None
            now = time.time()
            if self.retargeter is not None and fn == "Tracking":
                try:
                    retargeted = self.retargeter.process(parsed, now)
                except Exception as e:
                    event(f"[!] retarget 异常(不影响采集): {e!r}")
            rec = {"dev": self.sn, "cmd": name, "functionName": fn, "data": parsed}
            if retargeted is not None:
                rec["retarget"] = retargeted
            self.log(rec, recv_wall_ns, recv_qpc_ns)
            if self.viz is not None and isinstance(parsed, dict) and fn == "Tracking":
                try:
                    self.viz.update_from_pico(parsed, retargeted)
                except Exception as e:
                    event(f"[!] viz 更新失败: {e}")
            if isinstance(parsed, dict) and self.sn:
                with state_lock:
                    dev = devices.setdefault(self.sn, {"frames": 0})
                    dev["frames"] += 1
                    dev["last_rx"] = now
                    dev["latest"] = parsed
                    frames = dev["frames"]
                if now - self.last_print >= self.print_interval:
                    self.last_print = now
                    try:
                        render_status(summarize(self.sn, parsed, retargeted, frames))
                    except Exception as e:
                        event(f"[!] 显示异常(不影响采集): {e!r}")
            return

        self.log({"dev": self.sn, "cmd": name,
                  "payload_text": payload.decode("utf-8", "replace")},
                 recv_wall_ns, recv_qpc_ns)

    def run(self):
        buf = bytearray()
        try:
            while True:
                chunk = self.conn.recv(65536)
                if not chunk:
                    break
                buf += chunk
                # 分帧: 扫 0x3F -> 6B头 -> length -> payload -> 9B尾(校验 0xA5)
                while True:
                    i = buf.find(HEAD_CLIENT)
                    if i < 0:
                        buf.clear()
                        break
                    if i > 0:
                        del buf[:i]
                    if len(buf) < 6:
                        break
                    length = struct.unpack_from("<I", buf, 2)[0]
                    total = 15 + length
                    if length > 16 * 1024 * 1024:   # 异常长度，重新同步
                        del buf[0]
                        continue
                    if len(buf) < total:
                        break
                    if buf[total - 1] != TAIL:      # 尾校验失败，跳过一个字节重新扫
                        del buf[0]
                        continue
                    cmd = buf[1]
                    payload = bytes(buf[6:6 + length])
                    del buf[:total]
                    recv_qpc_ns = time.perf_counter_ns()
                    recv_wall_ns = time.time_ns()
                    try:
                        self.handle_payload(cmd, payload, recv_wall_ns, recv_qpc_ns)
                    except Exception as e:
                        # 任何处理异常都不能杀死接收线程/断开连接
                        event(f"[!] 帧处理异常(已跳过, cmd=0x{cmd:02X}): {e!r}")
        except (ConnectionResetError, BrokenPipeError, OSError):
            pass
        except Exception as e:
            event(f"[!] 接收线程异常: {e!r}")
        finally:
            try:
                self.conn.close()
            except OSError:
                pass
            if self.sn:
                with state_lock:
                    devices.pop(self.sn, None)
                    device_conns.pop(self.sn, None)
                event(f"<<< 设备离线 SN={self.sn}")


def recv_exact(conn, n):
    """从 socket 精确读 n 字节，连接断开返回 None"""
    buf = bytearray()
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return bytes(buf)


class VideoDiskWriter:
    """VST 专用异步写盘器；接收线程只打时间戳和入队。"""

    def __init__(self, video_path: str):
        self.video_path = video_path
        self.wall_path = VideoReceiver._sidecar_path(video_path)
        self.qpc_path = VideoReceiver._qpc_sidecar_path(video_path)
        self.queue = queue.Queue(maxsize=600)
        self.error = None
        self.max_backlog = 0
        self.picture_frames = 0
        self.bytes_written = 0
        self._thread = threading.Thread(
            target=self._run, name="pico-vst-writer", daemon=True
        )
        self._thread.start()

    def submit(self, au: bytes, is_picture: bool,
               recv_wall_ns: int, recv_qpc_ns: int) -> None:
        if self.error is not None:
            raise OSError(f"VST writer failed: {self.error}")
        try:
            self.queue.put((au, is_picture, recv_wall_ns, recv_qpc_ns), timeout=1.0)
        except queue.Full as exc:
            raise OSError("VST writer queue full") from exc
        self.max_backlog = max(self.max_backlog, self.queue.qsize())

    def close(self) -> None:
        if not self._thread.is_alive():
            raise OSError(f"VST writer failed: {self.error or 'writer exited'}")
        try:
            self.queue.put(None, timeout=2.0)
        except queue.Full as exc:
            raise OSError("VST writer queue did not drain") from exc
        self._thread.join(timeout=15.0)
        if self._thread.is_alive():
            raise OSError("VST writer drain timeout")
        if self.error is not None:
            raise OSError(f"VST writer failed: {self.error}")

    def _run(self) -> None:
        fp = ts_fp = qpc_fp = None
        try:
            # Never append a retry to a stale/partial stream. Joining two H.264
            # sessions can look non-empty while being undecodable.
            fp = open(self.video_path, "wb")
            ts_fp = open(self.wall_path, "w", encoding="utf-8")
            qpc_fp = open(self.qpc_path, "w", encoding="utf-8")
            last_flush = time.monotonic()
            while True:
                item = self.queue.get()
                try:
                    if item is None:
                        break
                    au, is_picture, recv_wall_ns, recv_qpc_ns = item
                    fp.write(au)
                    self.bytes_written += len(au)
                    if is_picture:
                        ts_fp.write(f"{recv_wall_ns}\n")
                        qpc_fp.write(f"{recv_qpc_ns}\n")
                        self.picture_frames += 1
                    now = time.monotonic()
                    if now - last_flush >= 0.5:
                        fp.flush(); ts_fp.flush(); qpc_fp.flush()
                        last_flush = now
                finally:
                    self.queue.task_done()
            fp.flush(); ts_fp.flush(); qpc_fp.flush()
        except Exception as exc:  # noqa: BLE001
            self.error = repr(exc)
        finally:
            for stream in (fp, ts_fp, qpc_fp):
                if stream is not None:
                    try:
                        stream.close()
                    except OSError:
                        pass


class VideoReceiver(threading.Thread):
    """VST 视频接收: 头显作为 TCP client 连到本端口,
    码流格式 [4B 大端 len][一个 H.264 Annex-B access unit] 连续写入。
    录制写盘走独立队列，接收线程在完整 AU 到达后立即打 QPC/墙钟；
    预览(ffplay)走独立队列+线程, 解码慢时丢预览帧, 绝不反堵接收。"""

    def __init__(self, port, save_path=None, view=True,
                 width=4096, height=1536, eye="both", fix_aspect=False):
        super().__init__(daemon=True)
        self.port = port
        self.save_path = save_path
        self.view = view
        self.width = width
        self.height = height
        self.eye = eye                # both / left / right (只影响预览, 录制始终全幅)
        self.fix_aspect = fix_aspect  # 预览把纵向按 4/3 拉回 (810 -> 1080)
        self.stop_event = threading.Event()
        self.ready = threading.Event()   # 已开始监听（可以发触发命令了）
        self.frames = 0
        self.bytes_rx = 0
        self.preview_dropped = 0
        self.last_rx = 0.0               # 最近一帧到达 wall 秒（防呆用）
        self.streaming = False           # 当前是否有 TCP 视频流
        self.writer_errors = 0
        self.reconnect_attempts = 0
        self.protocol_errors = 0
        self.last_bad_header = ""
        self.max_write_backlog = 0
        self._disk_writer_lock = threading.Lock()
        self._disk_writer = None
        self._record_ready = threading.Event()
        self._record_pending = False
        self._record_error = None
        self._cached_sps = None
        self._cached_pps = None

    @staticmethod
    def _annexb_nals(access_unit: bytes):
        """Return complete Annex-B NAL units, including their start codes."""
        starts = []
        i = 0
        size = len(access_unit)
        while i + 3 < size:
            if access_unit[i:i + 4] == b"\x00\x00\x00\x01":
                starts.append((i, i + 4))
                i += 4
            elif access_unit[i:i + 3] == b"\x00\x00\x01":
                starts.append((i, i + 3))
                i += 3
            else:
                i += 1
        result = []
        for index, (start, header) in enumerate(starts):
            end = starts[index + 1][0] if index + 1 < len(starts) else size
            if header < end:
                result.append((access_unit[header] & 0x1F, access_unit[start:end]))
        return result

    def start_recording(self, video_path: str) -> None:
        """Attach a writer to the warm stream and begin at a decodable IDR."""
        free_bytes = shutil.disk_usage(str(Path(video_path).parent)).free
        if free_bytes < 2 * 1024 ** 3:
            raise OSError(
                f"insufficient free space for VST capture: {free_bytes} bytes")
        writer = VideoDiskWriter(video_path)
        with self._disk_writer_lock:
            if self._disk_writer is not None:
                try:
                    writer.close()
                except OSError:
                    pass
                raise OSError("VST session writer is already active")
            self.save_path = video_path
            self._record_error = None
            self._record_pending = True
            self._record_ready.clear()
            self._disk_writer = writer

    def wait_recording_ready(self, timeout: float) -> bool:
        return self._record_ready.wait(timeout)

    def recording_pending(self) -> bool:
        with self._disk_writer_lock:
            return self._disk_writer is not None and self._record_pending

    def reset_codec_config(self) -> None:
        with self._disk_writer_lock:
            self._cached_sps = None
            self._cached_pps = None

    def stop_recording(self) -> tuple[int, int]:
        """Atomically detach, drain and close the current session writer."""
        with self._disk_writer_lock:
            writer = self._disk_writer
            self._disk_writer = None
            self._record_pending = False
            self.save_path = None
        if writer is None:
            return 0, 0
        writer.close()
        return writer.picture_frames, writer.bytes_written

    @staticmethod
    def _has_vcl_nal(access_unit: bytes) -> bool:
        """Return whether an Annex-B access unit contains a coded picture.

        PICO may send an initial SPS/PPS-only packet before the first IDR. That
        packet belongs in the H.264 stream, but it is not a decoded video frame
        and therefore must not receive a per-frame timestamp.
        """
        i = 0
        size = len(access_unit)
        while i + 3 < size:
            if access_unit[i:i + 4] == b"\x00\x00\x00\x01":
                nal = i + 4
                i = nal
            elif access_unit[i:i + 3] == b"\x00\x00\x01":
                nal = i + 3
                i = nal
            else:
                i += 1
                continue
            if nal < size and 1 <= (access_unit[nal] & 0x1F) <= 5:
                return True
        return False

    def _ffplay_cmd(self):
        filters = []
        out_w, out_h = self.width, self.height
        if self.eye in ("left", "right"):
            x = 0 if self.eye == "left" else self.width // 2
            filters.append(f"crop={self.width // 2}:{self.height}:{x}:0")
            out_w = self.width // 2
        if self.fix_aspect:
            filters.append(f"scale={out_w}:{out_h * 4 // 3}")
        cmd = ["ffplay", "-f", "h264", "-fflags", "nobuffer", "-flags", "low_delay",
               "-probesize", "32", "-analyzeduration", "0",
               "-window_title", f"PICO VST ({self.eye})"]
        if filters:
            cmd += ["-vf", ",".join(filters)]
        cmd += ["-i", "pipe:0"]
        return cmd

    @staticmethod
    def _feed_player(player, q):
        """预览喂流线程: 队列里取帧写给 ffplay"""
        while True:
            au = q.get()
            if au is None or player.poll() is not None:
                return
            try:
                player.stdin.write(au)
                player.stdin.flush()
            except (BrokenPipeError, OSError):
                return

    def run(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", self.port))
        srv.listen(2)
        srv.settimeout(1.0)
        self.ready.set()
        event(f"视频端口监听 0.0.0.0:{self.port}")
        try:
            while not self.stop_event.is_set():
                try:
                    conn, addr = srv.accept()
                except socket.timeout:
                    continue
                event(f"视频流连入 {addr[0]}:{addr[1]}")
                self.streaming = True
                try:
                    self.handle_stream(conn)
                finally:
                    self.streaming = False
                event(f"视频流断开, 共 {self.frames} 帧 {self.bytes_rx / 1e6:.1f} MB"
                      + (f" (预览丢帧 {self.preview_dropped})" if self.preview_dropped else ""))
        finally:
            srv.close()

    @staticmethod
    def _sidecar_path(save_path: str) -> str:
        """逐帧时间戳 sidecar 路径: vst_x.h264 -> vst_x.ts.jsonl"""
        if save_path.endswith(".h264"):
            return save_path[:-5] + ".ts.jsonl"
        return save_path + ".ts.jsonl"

    @staticmethod
    def _qpc_sidecar_path(save_path: str) -> str:
        if save_path.endswith(".h264"):
            return save_path[:-5] + ".qpc.ts.jsonl"
        return save_path + ".qpc.ts.jsonl"

    def wait_recording_idle(self, timeout: float = 10.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._disk_writer_lock:
                if self._disk_writer is None:
                    return True
            time.sleep(0.05)
        return False

    def handle_stream(self, conn):
        conn.settimeout(None)
        player = None
        feed_q = None
        try:
            if self.view:
                player = subprocess.Popen(self._ffplay_cmd(),
                                          stdin=subprocess.PIPE,
                                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                feed_q = queue.Queue(maxsize=90)   # ~1.5s@60fps 的预览缓冲
                threading.Thread(target=self._feed_player, args=(player, feed_q),
                                 daemon=True).start()
            while not self.stop_event.is_set():
                hdr = recv_exact(conn, 4)
                if hdr is None:
                    break
                n = struct.unpack(">I", hdr)[0]
                if not (0 < n <= 8 * 1024 * 1024):
                    self.protocol_errors += 1
                    self.last_bad_header = hdr.hex()
                    event(f"[!] 视频包长度异常 {n} header={hdr.hex()}, 断开重等")
                    break
                au = recv_exact(conn, n)
                if au is None:
                    break
                recv_qpc_ns = time.perf_counter_ns()
                recv_ns = time.time_ns()
                is_picture = self._has_vcl_nal(au)
                nals = self._annexb_nals(au)
                nal_types = {nal_type for nal_type, _ in nals}
                if is_picture:
                    self.frames += 1
                self.bytes_rx += n
                self.last_rx = recv_ns / 1e9
                try:
                    with self._disk_writer_lock:
                        for nal_type, nal in nals:
                            if nal_type == 7:
                                self._cached_sps = nal
                            elif nal_type == 8:
                                self._cached_pps = nal
                        active_writer = self._disk_writer
                        if active_writer is not None and self._record_pending:
                            if (5 in nal_types and self._cached_sps is not None
                                    and self._cached_pps is not None):
                                prefix = b""
                                if 7 not in nal_types:
                                    prefix += self._cached_sps
                                if 8 not in nal_types:
                                    prefix += self._cached_pps
                                active_writer.submit(prefix + au, True,
                                                     recv_ns, recv_qpc_ns)
                                self._record_pending = False
                                self._record_ready.set()
                                event("[VST] session recording started at SPS/PPS + IDR")
                        elif active_writer is not None:
                            active_writer.submit(au, is_picture,
                                                 recv_ns, recv_qpc_ns)
                        if active_writer is not None:
                            self.max_write_backlog = max(
                                self.max_write_backlog,
                                active_writer.max_backlog,
                            )
                except OSError as exc:
                    self.writer_errors += 1
                    self._record_error = repr(exc)
                    self._record_ready.set()
                    event(f"[!] VST asynchronous writer failed: {exc}")
                    break
                if feed_q is not None and player.poll() is None:
                    if feed_q.full():     # 预览: 丢最旧帧保实时, 不影响录制
                        try:
                            feed_q.get_nowait()
                            self.preview_dropped += 1
                        except queue.Empty:
                            pass
                    feed_q.put(au)
        except (ConnectionResetError, BrokenPipeError, OSError):
            pass
        finally:
            with self._disk_writer_lock:
                if self._disk_writer is not None and not self._record_pending:
                    self._record_pending = True
                    self._record_ready.clear()
                    event("[VST] stream disconnected; waiting for a fresh IDR")
            try:
                conn.close()
            except OSError:
                pass
            if feed_q is not None:
                feed_q.put(None)
            if player and player.poll() is None:
                try:
                    player.stdin.close()
                except OSError:
                    pass

    def stop(self):
        self.stop_event.set()


def get_local_ipv4():
    """取本机所有非 loopback IPv4（用于定向广播）"""
    ips = set()
    try:
        for fam, _, _, _, sockaddr in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = sockaddr[0]
            if not ip.startswith("127."):
                ips.add(ip)
    except socket.gaierror:
        pass
    if not ips:  # 兜底：UDP 连外网拿出口 IP（不产生真实流量）
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ips.add(s.getsockname()[0])
            s.close()
        except OSError:
            pass
    return sorted(ips)


def broadcast_loop(stop):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    while not stop.is_set():
        for ip in get_local_ipv4():
            frame = pack_frame(HEAD_SERVER, BCAST_CMD_TCPIP, ip.encode())
            targets = {".".join(ip.split(".")[:3]) + ".255", "255.255.255.255"}
            for t in targets:
                try:
                    s.sendto(frame, (t, BCAST_UDP_PORT))
                except OSError:
                    pass
        stop.wait(BCAST_INTERVAL)


def main():
    ap = argparse.ArgumentParser(description="XRoboToolkit PICO 位姿 + VST 视频接收端")
    ap.add_argument("--log-dir", default="data/sessions",
                    help="任务会话根目录 (默认 data/sessions; 每会话 raw/ 子目录)")
    ap.add_argument("--print-hz", type=float, default=5.0,
                    help="屏幕摘要刷新率 (默认 5Hz, 可设 30/120; 0=关闭。注意: 数据接收和落盘始终是全速的, 此项只影响屏幕显示)")
    ap.add_argument("--no-broadcast", action="store_true", help="关闭 UDP 发现广播（头显手动填 IP 时用）")
    ap.add_argument("--port", type=int, default=TCP_PORT, help=f"TCP 监听端口 (默认 {TCP_PORT})")
    ap.add_argument("--video", type=int, nargs="?", const=VIDEO_DEFAULT_PORT, default=None,
                    metavar="PORT", help=f"开启 VST 视频接收, PORT 为本地监听端口 (不填默认 {VIDEO_DEFAULT_PORT})")
    ap.add_argument("--video-save", metavar="FILE", default=None, help="把 H.264 裸流追加保存到文件")
    ap.add_argument("--no-view", action="store_true", help="视频只用--video-save存文件, 不开 ffplay 窗口")
    ap.add_argument("--video-size", default="4096x1536",
                    help="请求的分辨率 宽x高 (默认 4096x1536 双目拼接)")
    ap.add_argument("--video-eye", choices=["both", "left", "right"], default="both",
                    help="预览只看单镜头 (默认 both; 录制始终是全幅双目)")
    ap.add_argument("--no-fix-aspect", action="store_true",
                    help="旧 2160x810 预览不做纵向 4/3 拉伸；4096x1536 从不拉伸")
    ap.add_argument("--video-fps", type=int, default=30,
                    help="请求的采集帧率 (默认 30)")
    ap.add_argument("--no-retarget", action="store_true",
                    help="关闭 retarget 后处理 (默认开启: 按 pico_tracking_retarget_node.cpp 管线输出头相对位姿)")
    ap.add_argument("--viz", action="store_true",
                    help="开启 MeshCat 3D 可视化 (默认 world 世界系绝对位姿: 头/左/右各在真实位置)")
    ap.add_argument("--viz-raw", action="store_true",
                    help="等价于默认的 world 世界系绝对位姿视图 (兼容旧名; 隐含 --viz)")
    ap.add_argument("--viz-retarget", action="store_true",
                    help="改用 retarget 头系视图 (head 钉在原点, 画相对头腕位姿; 隐含 --viz)")
    ap.add_argument("--viz-hz", type=float, default=30.0, help="可视化刷新率 (默认 30)")
    ap.add_argument("--viz-overlap", action="store_true",
                    help="仅调试朝向: 左右手柄轴平移清零后画到同一原点 (会人为让两者重合, 不代表真实空间位置)")
    ap.add_argument("--viz-no-skeleton", action="store_true",
                    help="不画连线火柴人骨架(头/躯干/双臂/腕), 只保留坐标轴")
    ap.add_argument("--viz-wrist", "--viz-ctrl-raw", dest="viz_wrist", action="store_true",
                    help="额外画遥操腕系细轴(含 (1,0,1)180°); 默认粗轴=头/手柄统一世界系。"
                         "--viz-ctrl-raw 为旧别名")
    ap.add_argument("--viz-calib", default=None,
                    help="MANUS 对齐外参 JSON (PICO参考系->MANUS腕系); 缺省单位阵")
    ap.add_argument("--manus-bridge", nargs="?", const="",
                    help="无 ROS: 启动 manus_ndjson_bridge 实时叠加 MANUS 手到 PICO 手柄系, "
                         "验证腕对齐。可跟桥可执行文件路径(缺省用默认路径)。dongle 同机占用, "
                         "此时勿再另跑 manus_collector.py")
    ap.add_argument("--service", action="store_true",
                    help="常驻服务模式: 头显连一次保持不断, 录制由 pico_record.py "
                         "START/STOP 控制(不再每次采集都要重连头显)")
    ap.add_argument("--control-port", type=int, default=None,
                    help=f"service 模式控制端口 (默认 {PICO_CONTROL_PORT}, 仅监听本机)")
    ap.add_argument("--hands", default="both", choices=list(HANDS_CHOICES),
                    help="默认只录/只画哪只手柄: left / right / both。"
                         "service 下可被 pico_record.py start --hands 覆盖")
    args = ap.parse_args()

    global PRINT_HZ0
    PRINT_HZ0 = args.print_hz <= 0

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)

    # ---- MeshCat 可视化 (可选) ----
    viz = None
    manus_src = None
    if args.viz or args.viz_raw or args.viz_retarget:
        try:
            from pico_controller_viz import ControllerVisualizer
            # 默认 world(=raw 绝对世界系); 仅 --viz-retarget 时才用头系相对视图。
            mode = "retarget" if args.viz_retarget else "raw"
            viz = ControllerVisualizer(mode=mode, hz=args.viz_hz,
                                       calib_path=args.viz_calib,
                                       overlap=args.viz_overlap,
                                       skeleton=not args.viz_no_skeleton,
                                       show_wrist=args.viz_wrist,
                                       hands=args.hands)
            label = "retarget 头系(head=原点)" if mode == "retarget" else "world 世界系绝对位姿"
            print(f"[*] 可视化已开启 (mode={mode}: {label}, "
                  f"hands={hands_label(parse_hands(args.hands))}); "
                  f"浏览器打开上面的 MeshCat 地址")
        except Exception as e:
            print(f"[!] 可视化启动失败, 继续无可视化运行: {e}")
            viz = None
        if viz is not None and args.manus_bridge is not None:
            try:
                from manus_ndjson_source import ManusNdjsonSource
                bridge = args.manus_bridge or None  # 空串 -> 用默认路径
                manus_src = ManusNdjsonSource(viz, bridge=bridge, on_event=event)
                manus_src.start()
                print("[*] MANUS(无ROS)源已启动: 手套腕部/手指将叠加到对应 PICO 手柄原点; "
                      "看 [world] HUD 的 MANUS手套腕↔手柄原点 距离/夹角来验证对齐")
            except Exception as e:
                print(f"[!] MANUS(无ROS)源启动失败(可视化仍可用 PICO): {e}")
                manus_src = None

    # ---- VST 视频: 先起监听, 设备上线后自动下发 RequestVRCamera ----
    video_rx = None
    on_online = None
    video_inner = None
    if args.video is not None:
        view = not args.no_view
        if view and shutil.which("ffplay") is None:
            print("[!] 未找到 ffplay, 视频将不落屏" + ("" if args.video_save else " (也未指定 --video-save!)"))
            view = False
        try:
            w, h = (int(x) for x in args.video_size.lower().split("x"))
        except ValueError:
            ap.error("--video-size 格式应为 宽x高, 如 4096x1536")
        video_rx = VideoReceiver(
            args.video, save_path=args.video_save, view=view,
            width=w, height=h, eye=args.video_eye,
            # 日常 4096x1536（每眼 2048x1536）已经是正常 4:3；
            # 纵向拉伸仅保留给明确的旧版 2160x810 预览。
            fix_aspect=(not args.no_fix_aspect and (w, h) == (2160, 810)),
        )
        video_rx.start()
        video_rx.ready.wait()
        video_inner = {"port": args.video, "width": w, "height": h,
                       "fps": args.video_fps, "bitrate": 20971520, "captureRenderMode": 2}

        # 设备上线即推流: 非 service 落盘; service 先暖机(不落盘), START 时再挂文件。
        def on_online(dev):
            dev.send_control_json({"functionName": "RequestVRCamera",
                                   "value": json.dumps({"on": 1, **video_inner})})
            event(f"已向 {dev.sn} 下发 RequestVRCamera "
                  f"(port={args.video} {w}x{h}@{args.video_fps}"
                  f"{'; 暖机' if args.service else ''})")

    # ---- 录制器: 门控落盘 ----
    global RECORDER
    RECORDER = Recorder(log_dir, video_rx=video_rx, video_inner=video_inner,
                        video_autosave=args.service, default_hands=args.hands)

    # ---- service 模式: 控制端口; 非 service: 立即开录(旧行为) ----
    control = None
    if args.service:
        def _handler(cmd, arg):
            if cmd == "PING":
                return "PONG"
            if cmd == "START":
                session, hands = parse_start_arg(
                    arg.strip() or datetime.now().strftime("%Y%m%d_%H%M%S"),
                    default_hands=RECORDER.default_hands)
                if not session:
                    session = datetime.now().strftime("%Y%m%d_%H%M%S")
                record_video = "video=0" not in arg.lower().split()
                return RECORDER.start(session, hands=hands,
                                      record_video=record_video)
            if cmd == "STOP":
                return RECORDER.stop()
            if cmd == "VST_OFF":
                RECORDER._video_cmd(on=False)
                return "OK vst_off"
            if cmd == "STATUS":
                return RECORDER.status()
            return f"ERR unknown command {cmd}"
        cport = args.control_port or PICO_CONTROL_PORT
        control = ControlServer(cport, _handler)
        control.start()
    else:
        RECORDER.start(datetime.now().strftime("%Y%m%d_%H%M%S"),
                       hands=args.hands)

    stop = threading.Event()
    if video_rx is not None:
        def _vst_reconnect_watchdog():
            # The headset may answer the first RequestVRCamera before its camera
            # page/native service is ready. Retry with bounded exponential
            # backoff until actual frame bytes arrive; START also performs its
            # own immediate off/on transition.
            backoff_s = 3.0
            if stop.wait(backoff_s):
                return
            while not stop.is_set():
                if RECORDER._video_recovery_active:
                    if stop.wait(0.5):
                        return
                    continue
                now = time.time()
                healthy = (
                    video_rx.streaming and video_rx.last_rx > 0 and
                    now - video_rx.last_rx <= 3.0
                )
                with state_lock:
                    online = bool(device_conns)
                if healthy:
                    backoff_s = 3.0
                elif online:
                    video_rx.reconnect_attempts += 1
                    attempt = video_rx.reconnect_attempts
                    event(f"[VST] 未收到新帧，自动重连 attempt={attempt}")
                    RECORDER._video_cmd(on=False)
                    if stop.wait(0.35):
                        return
                    RECORDER._video_cmd(on=True)
                    backoff_s = min(30.0, max(3.0, backoff_s * 1.7))
                if stop.wait(backoff_s):
                    return

        threading.Thread(
            target=_vst_reconnect_watchdog,
            name="pico-vst-reconnect",
            daemon=True,
        ).start()
    if not args.no_broadcast:
        threading.Thread(target=broadcast_loop, args=(stop,), daemon=True).start()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", args.port))
    srv.listen(8)

    ips = get_local_ipv4()
    discovery_text = (
        f"独立发现广播 UDP :{BCAST_UDP_PORT}"
        if args.no_broadcast
        else f"UDP广播 :{BCAST_UDP_PORT}/{BCAST_INTERVAL:g}s"
    )
    INFO_LINES.extend([
        f"TCP :{args.port} | {discovery_text}"
        + (f" | 视频 :{args.video}" if args.video is not None else ""),
        f"本机 IP: {', '.join(ips) or '(未获取到)'}  <- 在头显 App 里选这个",
        (f"service 模式: 控制口 {args.control_port or PICO_CONTROL_PORT}; "
         f"默认 hands={hands_label(parse_hands(args.hands))}; "
         f"录制: python3 pico_record.py start [名]  (同一终端 Ctrl+C/Enter 停)"
         if args.service
         else f"日志目录: {log_dir} (连上即录, session={RECORDER.session})"),
        "坐标: 落盘保留原始左手系 pose; 训练/可视化用右手系 X前Y左Z上 (管线内转换)",
    ])
    for l in INFO_LINES:
        print(f"[*] {l}")
    if args.service:
        print("[*] 服务常驻中… 本窗口 Ctrl+C = 停服务(会先停录)。日常请另开终端跑 pico_record。")
    else:
        print("[*] 等待头显连接... (Ctrl+C 退出并保存)")

    try:
        while True:
            conn, addr = srv.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            event(f"TCP 连接来自 {addr[0]}:{addr[1]}")
            DeviceConn(conn, addr, args.print_hz, on_online=on_online,
                       retarget=not args.no_retarget, viz=viz).start()
    except KeyboardInterrupt:
        print("\n[*] 退出")
    finally:
        stop.set()
        if control is not None:
            control.stop()
        try:
            RECORDER.stop()
        except Exception:
            pass
        if manus_src is not None:
            manus_src.stop()
        if video_rx is not None:
            # 通知所有在线头显停止推流
            with state_lock:
                conns = list(device_conns.values())
            for dev in conns:
                try:
                    inner = {"on": 0}
                    dev.send_control_json({"functionName": "RequestVRCamera",
                                           "value": json.dumps(inner)})
                except OSError:
                    pass
            video_rx.stop()
        srv.close()


if __name__ == "__main__":
    main()
