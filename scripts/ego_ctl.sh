#!/usr/bin/env bash
# ego_ctl.sh — 一键启停 / 状态 / 健康检查 / 离线管线复验（采集机 181）
#
# 用法:
#   bash scripts/ego_ctl.sh start          # 停旧进程 → 交互配对触觉 → 起三路常驻服务
#   bash scripts/ego_ctl.sh stop           # 停服务（含手工起的残留）
#   bash scripts/ego_ctl.sh status         # 录制状态 + 进程 + 端口
#   bash scripts/ego_ctl.sh check          # 数据链路健康检查（不录新数据）
#   bash scripts/ego_ctl.sh pipeline [名]  # 对已有 session 复验 align/quality/export
#   bash scripts/ego_ctl.sh doctor         # start 前环境自检（依赖/文件/端口）
#
# 起好后日常采集(同一终端, Ctrl+C / Enter 停录):
#   python3 pico_record.py start <任务名>
#   python3 pico_record.py start <任务名> --hands left
# 远程停(可选): python3 pico_record.py stop
# 旧行为(发完即退): python3 pico_record.py start <名> --detach
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${HERE}"
RUN_DIR="${HERE}/.run"
LOG_DIR="${HERE}/logs"                 # 仅运行期杂项；原始数据不在此
DATA_ROOT="${HERE}/data"
SESSIONS_DIR="${DATA_ROOT}/sessions"    # 新采集：<任务>/<序号>/raw + 同目录产物
RAW_DIR="${DATA_ROOT}/raw"              # 旧布局只读兼容
TACTILE_RAW_DIR="${DATA_ROOT}/tactile_raw"
ALIGNED_DIR="${DATA_ROOT}/aligned"
EXPORT_DIR="${DATA_ROOT}/export"
REVIEW_DIR="${DATA_ROOT}/review"
PY="${PY:-python3}"
TACTILE_PHASE_TIMEOUT="${TACTILE_PHASE_TIMEOUT:-30}"
TACTILE_RATE_HZ="${TACTILE_RATE_HZ:-60}"
mkdir -p "${RUN_DIR}" "${LOG_DIR}" "${SESSIONS_DIR}"

PICO_CTRL=63910
MANUS_CTRL=63911
TACTILE_CTRL=63912
PICO_TRACK_PORT=63901
PICO_VIDEO_PORT=63902
# 注意: 环境变量 PICO_VIDEO=0/1 控制是否收 VST，勿与端口常量同名

red()  { printf '\033[31m%s\033[0m\n' "$*"; }
grn()  { printf '\033[32m%s\033[0m\n' "$*"; }
ylw()  { printf '\033[33m%s\033[0m\n' "$*"; }
hdr()  { printf '\n======== %s ========\n' "$*"; }

TACTILE_PAIR_FIFO=""
TACTILE_FIFO_OPEN=0
STARTUP_IN_PROGRESS=0
ROLLBACK_IN_PROGRESS=0

_kill_pidfile() {
  local name="$1"
  local expected="$2"
  local grace_steps="${3:-5}"
  local pidf="${RUN_DIR}/${name}.pid"
  if [[ -f "${pidf}" ]]; then
    local pid cmd step
    pid="$(cat "${pidf}" 2>/dev/null || true)"
    if [[ "${pid}" =~ ^[0-9]+$ ]] && kill -0 "${pid}" 2>/dev/null; then
      cmd="$(ps -ww -p "${pid}" -o args= 2>/dev/null || true)"
      if [[ -n "${expected}" && "${cmd}" != *"${expected}"* ]]; then
        ylw "[stop] ${name}.pid=${pid} 已不是 ${expected}，拒绝误杀，仅清理陈旧 pidfile"
        rm -f "${pidf}" || ylw "[stop] 无法清理陈旧 pidfile: ${pidf}"
        return 0
      fi
      kill "${pid}" 2>/dev/null || true
      for ((step = 0; step < grace_steps; step++)); do
        kill -0 "${pid}" 2>/dev/null || break
        cmd="$(ps -ww -p "${pid}" -o args= 2>/dev/null || true)"
        if [[ "${cmd}" != *"${expected}"* ]]; then
          ylw "[stop] ${name} pid=${pid} 在等待期间身份已变化；不再向该 PID 发信号"
          break
        fi
        sleep 0.2
      done
      if kill -0 "${pid}" 2>/dev/null; then
        cmd="$(ps -ww -p "${pid}" -o args= 2>/dev/null || true)"
        if [[ "${cmd}" == *"${expected}"* ]]; then
          ylw "[stop] ${name} 未在宽限期退出，发送 SIGKILL"
          kill -9 "${pid}" 2>/dev/null || true
        else
          ylw "[stop] ${name} pid=${pid} 已被复用；跳过 SIGKILL"
        fi
      fi
      echo "[stop] ${name} pid=${pid}"
    fi
    if [[ "$(cat "${pidf}" 2>/dev/null || true)" == "${pid}" ]]; then
      rm -f "${pidf}" || ylw "[stop] 无法清理 pidfile: ${pidf}"
    fi
  fi
  return 0
}

_kill_by_identity() {
  # 只处理本项目绝对路径对应的进程；SIGKILL 前重新核验，避免 PID 复用误杀。
  local expected_path="$1"
  local required_arg="${2:-}"
  local grace_steps="${3:-10}"
  local pids pid cmd step any_live
  local killed=()
  pids="$(pgrep -f -- "$(basename "${expected_path}")" 2>/dev/null || true)"
  for pid in ${pids}; do
    [[ "${pid}" == "$$" ]] && continue
    cmd="$(ps -ww -p "${pid}" -o args= 2>/dev/null || true)"
    if [[ "${cmd}" == *"${expected_path}"* \
        && ( -z "${required_arg}" || "${cmd}" == *"${required_arg}"* ) ]]; then
      echo "[stop] matched pid=${pid}: ${expected_path} ${required_arg}"
      kill "${pid}" 2>/dev/null || true
      killed+=("${pid}")
    fi
  done
  for ((step = 0; step < grace_steps; step++)); do
    any_live=0
    for pid in "${killed[@]}"; do
      cmd="$(ps -ww -p "${pid}" -o args= 2>/dev/null || true)"
      if kill -0 "${pid}" 2>/dev/null \
          && [[ "${cmd}" == *"${expected_path}"* \
          && ( -z "${required_arg}" || "${cmd}" == *"${required_arg}"* ) ]]; then
        any_live=1
      fi
    done
    [[ "${any_live}" == "0" ]] && break
    sleep 0.2
  done
  for pid in "${killed[@]}"; do
    cmd="$(ps -ww -p "${pid}" -o args= 2>/dev/null || true)"
    if kill -0 "${pid}" 2>/dev/null \
        && [[ "${cmd}" == *"${expected_path}"* \
        && ( -z "${required_arg}" || "${cmd}" == *"${required_arg}"* ) ]]; then
      ylw "[stop] pid=${pid} 未在宽限期退出，发送 SIGKILL"
      kill -9 "${pid}" 2>/dev/null || true
    fi
  done
  return 0
}

_close_pairing_channel() {
  if [[ "${TACTILE_FIFO_OPEN:-0}" == "1" ]]; then
    exec 7<&- || true
    exec 8>&- || true
    TACTILE_FIFO_OPEN=0
  fi
  if [[ -n "${TACTILE_PAIR_FIFO:-}" && -p "${TACTILE_PAIR_FIFO}" ]]; then
    rm -f -- "${TACTILE_PAIR_FIFO}" \
      || ylw "[tactile] 无法清理配对 FIFO: ${TACTILE_PAIR_FIFO}"
  fi
  TACTILE_PAIR_FIFO=""
}

_cleanup_stale_pairing_fifos() {
  local fifo
  for fifo in "${RUN_DIR}"/tactile_pairing.*.fifo; do
    if [[ -p "${fifo}" ]]; then
      rm -f -- "${fifo}" || ylw "[tactile] 无法清理陈旧 FIFO: ${fifo}"
    fi
  done
  return 0
}

_stop_service_processes() {
  _close_pairing_channel || true
  _kill_pidfile tactile "${HERE}/tactile_collector.py --service" 100
  _kill_pidfile pico "${HERE}/pico_receiver.py --service" 10
  _kill_pidfile manus "${HERE}/manus_collector.py --service" 10
  _kill_by_identity "${HERE}/tactile_collector.py" "--service" 100
  _kill_by_identity "${HERE}/pico_receiver.py" "--service" 10
  _kill_by_identity "${HERE}/manus_collector.py" "--service" 10
  # bridge 由 collector 拉起；残留则清掉
  _kill_by_identity "${HERE}/manus_ndjson_bridge/manus_ndjson_bridge.out" "" 10
  _cleanup_stale_pairing_fifos
  return 0
}

stop_all() {
  hdr "STOP"
  # 若还在录，先停录（忽略失败）
  if ! "${PY}" "${HERE}/pico_record.py" stop >/dev/null 2>&1; then
    if [[ "${1:-manual}" == "manual" ]]; then
      ylw "[stop] 三路协调 STOP 未完整确认（也可能原本未录/未启动）；继续清理进程，请检查残留 .partial/meta"
    fi
  fi
  _stop_service_processes
  sleep 0.5
  grn "[stop] 服务进程清理完成"
  status_brief || true
}

_pid_is_live() {
  local pid="$1"
  local state
  kill -0 "${pid}" 2>/dev/null || return 1
  state="$(ps -p "${pid}" -o stat= 2>/dev/null || true)"
  [[ -n "${state}" && "${state}" != Z* ]]
}

_wait_tactile_log() {
  local pid="$1"
  local pattern="$2"
  local description="$3"
  local started=${SECONDS}
  while true; do
    if ! _pid_is_live "${pid}"; then
      red "[tactile] 进程在等待 ${description} 时退出"
      return 1
    fi
    if grep -Fq -- "${pattern}" "${RUN_DIR}/tactile_service.log" 2>/dev/null; then
      return 0
    fi
    if (( SECONDS - started >= TACTILE_PHASE_TIMEOUT )); then
      red "[tactile] 等待 ${description} 超时 (${TACTILE_PHASE_TIMEOUT}s)"
      return 1
    fi
    sleep 0.1
  done
}

_tactile_status_ready() {
  local mode="${1:-idle}"
  "${PY}" - "${TACTILE_CTRL}" "${mode}" >/dev/null 2>&1 <<'PY'
import re
import sys

from record_control import send_command

port = int(sys.argv[1])
mode = sys.argv[2]
try:
    reply = send_command(port, "STATUS", timeout=2.0)
except Exception:
    raise SystemExit(1)

def field(name, default=""):
    match = re.search(r"\b{}=([^\s]+)".format(re.escape(name)), reply)
    return match.group(1) if match else default

allowed_states = {"PAIRED_IDLE"} if mode == "idle" else {"PAIRED_IDLE", "RECORDING_RAW"}
required = (
    reply.startswith("OK ")
    and field("state") in allowed_states
    and field("paired") == "1"
    and field("raw_stream_valid") == "1"
    and field("capture_scope") == "tactile_only"
    and field("pipeline_ready") == "0"
    and field("gaps") == "0"
    and field("errors") == "0"
    and field("fault") == "none"
)
if mode == "idle":
    required = required and field("start_ready") == "1"
ages = re.search(r"\bage_ms=l=(-?\d+),r=(-?\d+)\b", reply)
if not ages or any(not 0 <= int(value) <= 2000 for value in ages.groups()):
    required = False
raise SystemExit(0 if required else 1)
PY
}

_show_start_logs() {
  echo "  tail -50 ${RUN_DIR}/pico_service.log"
  echo "  tail -50 ${RUN_DIR}/manus_service.log"
  echo "  tail -50 ${RUN_DIR}/tactile_service.log"
  if [[ -f "${RUN_DIR}/tactile_service.log" ]]; then
    echo "--- tactile_service.log (tail) ---"
    tail -30 "${RUN_DIR}/tactile_service.log" || true
  fi
}

_startup_rollback() {
  [[ "${STARTUP_IN_PROGRESS}" == "1" ]] || return 0
  [[ "${ROLLBACK_IN_PROGRESS}" == "0" ]] || return 0
  ROLLBACK_IN_PROGRESS=1
  trap - EXIT
  trap '' INT TERM
  red "[svc] 启动未完成，回滚本次三路服务"
  _close_pairing_channel || true
  "${PY}" "${HERE}/pico_record.py" stop >/dev/null 2>&1 || true
  _stop_service_processes || true
  STARTUP_IN_PROGRESS=0
  ROLLBACK_IN_PROGRESS=0
  return 0
}

_startup_interrupted() {
  trap '' INT TERM
  echo
  ylw "[svc] 用户中断启动"
  _startup_rollback
  trap - EXIT INT TERM
  exit 130
}

_lock_lifecycle() {
  if ! command -v flock >/dev/null 2>&1; then
    red "缺 flock（util-linux）；无法安全串行化 start/stop"
    return 1
  fi
  exec 9>"${RUN_DIR}/ego_ctl.lock"
  if ! flock -n 9; then
    red "另一个 ego_ctl start/stop 正在运行；请等待其结束"
    return 1
  fi
}

_ensure_bridge() {
  local BRIDGE="${HERE}/manus_ndjson_bridge/manus_ndjson_bridge.out"
  if [[ ! -x "${BRIDGE}" || "${HERE}/manus_ndjson_bridge/manus_ndjson_bridge.cpp" -nt "${BRIDGE}" ]]; then
    echo "[svc] bridge 缺失，编译..."
    bash "${HERE}/scripts/setup_manus_bridge.sh"
  fi
  export MANUS_CALIB_LEFT="${HERE}/config/manus_left.mcal"
  export MANUS_CALIB_RIGHT="${HERE}/config/manus_right.mcal"
  [[ -s "${MANUS_CALIB_LEFT}" ]] || { red "[svc] 缺左手个人标定: config/manus_left.mcal"; return 1; }
  [[ -s "${MANUS_CALIB_RIGHT}" ]] || { red "[svc] 缺右手个人标定: config/manus_right.mcal"; return 1; }
  echo "[svc] MANUS_CALIB_LEFT=${MANUS_CALIB_LEFT:-"(none)"}"
  echo "[svc] MANUS_CALIB_RIGHT=${MANUS_CALIB_RIGHT:-"(none)"}"
}

start_all() {
  hdr "START"
  if [[ ! -t 0 || ! -r /dev/tty || ! -w /dev/tty ]]; then
    red "[svc] 触觉启动必须在可交互终端运行（需要两次人工确认）；当前没有可用 TTY"
    return 1
  fi
  if ! doctor_quiet; then
    red "[svc] 启动前自检失败；尚未停止当前已有服务"
    return 1
  fi

  STARTUP_IN_PROGRESS=1
  trap '_startup_rollback' EXIT
  trap '_startup_interrupted' INT TERM
  stop_all startup
  _ensure_bridge

  local vopt=()
  if [[ "${PICO_VIDEO:-1}" == "1" ]]; then
    vopt=(--video --no-view)
    echo "[svc] VST 视频: ON"
  else
    echo "[svc] VST 视频: OFF (PICO_VIDEO=0)"
  fi

  : > "${RUN_DIR}/pico_service.log"
  : > "${RUN_DIR}/manus_service.log"
  : > "${RUN_DIR}/tactile_service.log"

  echo "[svc] 启动 PICO service..."
  nohup "${PY}" "${HERE}/pico_receiver.py" --service --print-hz 0 --log-dir "${SESSIONS_DIR}" "${vopt[@]}" \
    9>&- > "${RUN_DIR}/pico_service.log" 2>&1 &
  local pico_pid=$!
  printf '%s\n' "${pico_pid}" > "${RUN_DIR}/pico.pid"

  echo "[svc] 启动 MANUS service (hand-motion=none; 正式采集勿改 imu)..."
  nohup "${PY}" "${HERE}/manus_collector.py" --service --print-hz 0 --hand-motion none \
    --log-dir "${SESSIONS_DIR}" \
    9>&- > "${RUN_DIR}/manus_service.log" 2>&1 &
  local manus_pid=$!
  printf '%s\n' "${manus_pid}" > "${RUN_DIR}/manus.pid"

  # 先证明原有两路已起；触觉控制口要等人工配对完成后才会监听。
  local i ok=0
  for i in {1..20}; do
    if ! _pid_is_live "${pico_pid}" || ! _pid_is_live "${manus_pid}"; then
      break
    fi
    if "${PY}" "${HERE}/pico_record.py" ping --no-tactile >/dev/null 2>&1; then
      ok=1
      break
    fi
    sleep 0.5
  done
  if [[ "${ok}" != "1" ]]; then
    red "[svc] PICO/MANUS 控制口未就绪"
    _show_start_logs
    _startup_rollback
    trap - EXIT INT TERM
    return 1
  fi

  TACTILE_PAIR_FIFO="${RUN_DIR}/tactile_pairing.$$.fifo"
  rm -f -- "${TACTILE_PAIR_FIFO}"
  if ! (umask 077 && mkfifo "${TACTILE_PAIR_FIFO}"); then
    red "[tactile] 无法创建配对 FIFO: ${TACTILE_PAIR_FIFO}"
    _startup_rollback
    trap - EXIT INT TERM
    return 1
  fi
  # FD8 是父进程 RDWR keeper/writer，先打开它可避免后续只读端在 open() 阶段阻塞。
  if exec 8<>"${TACTILE_PAIR_FIFO}"; then
    TACTILE_FIFO_OPEN=1
  else
    red "[tactile] 无法打开配对 FIFO"
    _startup_rollback
    trap - EXIT INT TERM
    return 1
  fi
  # 子进程只继承只读 FD7；若父脚本异常消失，它能收到 EOF 并退出配对。
  if ! exec 7<"${TACTILE_PAIR_FIFO}"; then
    red "[tactile] 无法打开配对 FIFO 读端"
    _startup_rollback
    trap - EXIT INT TERM
    return 1
  fi

  echo "[svc] 启动 TACTILE service；请按当前终端提示完成左手配对..."
  nohup "${PY}" -u "${HERE}/tactile_collector.py" --service \
    --log-dir "${SESSIONS_DIR}" \
    --rate-hz "${TACTILE_RATE_HZ}" \
    9>&- <&7 7>&- 8>&- > "${RUN_DIR}/tactile_service.log" 2>&1 &
  local tactile_pid=$!
  printf '%s\n' "${tactile_pid}" > "${RUN_DIR}/tactile.pid"

  if ! _wait_tactile_log "${tactile_pid}" "[pairing_baseline]" "基线提示"; then
    _show_start_logs
    _startup_rollback
    trap - EXIT INT TERM
    return 1
  fi
  printf '\n[触觉配对 1/2] 请松开两只触觉指套并保持静止；准备好后按 Enter：' > /dev/tty
  if ! IFS= read -r _pairing_ack < /dev/tty; then
    red "[tactile] 无法读取基线确认"
    _startup_rollback
    trap - EXIT INT TERM
    return 1
  fi
  if ! printf '\n' >&8; then
    red "[tactile] 无法发送基线确认"
    _show_start_logs
    _startup_rollback
    trap - EXIT INT TERM
    return 1
  fi
  printf '[触觉配对] 正在采集 2 秒静止基线...\n' > /dev/tty

  if ! _wait_tactile_log "${tactile_pid}" "[left_press]" "左手按压提示"; then
    _show_start_logs
    _startup_rollback
    trap - EXIT INT TERM
    return 1
  fi
  printf '\n[触觉配对 2/2] 按 Enter 后等待标准3秒倒计时；看到“开始采集”再按住左手任意一个或多个触觉区域，右手保持不动，并持续到显示配对成功（活动窗口4秒）：' > /dev/tty
  if ! IFS= read -r _pairing_ack < /dev/tty; then
    red "[tactile] 无法读取左手按压确认"
    _startup_rollback
    trap - EXIT INT TERM
    return 1
  fi
  if ! printf '\n' >&8; then
    red "[tactile] 无法发送左手按压确认"
    _show_start_logs
    _startup_rollback
    trap - EXIT INT TERM
    return 1
  fi
  printf '  已开始；请继续按住左手，直到显示配对成功。\n' > /dev/tty

  if ! _wait_tactile_log "${tactile_pid}" "[tactile] service 控制端口" "触觉服务就绪"; then
    _show_start_logs
    _startup_rollback
    trap - EXIT INT TERM
    return 1
  fi

  ok=0
  for ((i = 0; i < 25; i++)); do
    if ! _pid_is_live "${tactile_pid}"; then
      break
    fi
    if _tactile_status_ready idle \
        && "${PY}" "${HERE}/pico_record.py" ping >/dev/null 2>&1; then
      ok=1
      break
    fi
    sleep 0.2
  done
  if [[ "${ok}" != "1" ]]; then
    red "[tactile] 配对进程未达到严格 PAIRED_IDLE/双路健康状态"
    _show_start_logs
    _startup_rollback
    trap - EXIT INT TERM
    return 1
  fi

  _close_pairing_channel
  STARTUP_IN_PROGRESS=0
  trap - EXIT INT TERM

  grn "[svc] PICO + MANUS + TACTILE 已启动，触觉左右绑定仅在本进程有效"
  status_brief || true
  cat <<EOF

下一步:
  1. 保持两只触觉 USB 在线；不要重插（重插后须 restart 重新配对）
  2. 头显连本机 IP，App 开 Send + Head + Controller
  3. 另开终端采集(同一窗口即可结束):
       python3 pico_record.py start <任务名>
       # 可选 --hands left|right ; 结束: Enter 或 Ctrl+C
  4. 查状:  bash scripts/ego_ctl.sh status / python3 pico_record.py status
  5. 复验:  bash scripts/ego_ctl.sh pipeline <任务名>

坐标防呆: 训练/3D = 右手系 X前Y左Z上; PICO 原始左手系只在 pose 字段保留。
EOF
}

status_brief() {
  local fail=0
  echo "--- record status ---"
  if ! "${PY}" "${HERE}/pico_record.py" status 2>&1; then
    red "[status] 控制口无响应或协议不完整（服务未起？）"
    fail=1
  fi
  echo "--- processes ---"
  ps -eo pid,etime,cmd | grep -E "pico_receiver|manus_collector|tactile_collector|manus_ndjson_bridge" | grep -v grep || echo "(none)"
  echo "--- control ports ---"
  if command -v ss >/dev/null 2>&1; then
    ss -lntp 2>/dev/null | grep -E ":${PICO_CTRL}|:${MANUS_CTRL}|:${TACTILE_CTRL}|:${PICO_TRACK_PORT}|:${PICO_VIDEO_PORT}" || true
  else
    ylw "[status] 缺 ss（iproute2），跳过端口列表"
  fi
  return "${fail}"
}

doctor_quiet() {
  local fail=0
  if ! command -v "${PY}" >/dev/null 2>&1; then
    red "缺 ${PY}"
    return 1
  fi
  for command_name in nohup mkfifo flock pgrep ps grep tail; do
    command -v "${command_name}" >/dev/null 2>&1 \
      || { red "缺系统命令 ${command_name}"; fail=1; }
  done
  "${PY}" -c "import numpy,cv2,h5py,serial" 2>/dev/null \
    || { red "缺 numpy/cv2/h5py/pyserial"; fail=1; }
  [[ -f "${HERE}/pico_receiver.py" ]] || { red "缺 pico_receiver.py"; fail=1; }
  [[ -f "${HERE}/manus_collector.py" ]] || { red "缺 manus_collector.py"; fail=1; }
  for tactile_file in tactile_protocol.py tactile_probe.py tactile_pairing.py tactile_collector.py \
                       config/tactile_pairing.json; do
    [[ -f "${HERE}/${tactile_file}" ]] \
      || { red "缺 ${tactile_file}"; fail=1; }
  done
  if [[ "${fail}" -eq 0 ]]; then
    "${PY}" "${HERE}/tactile_pairing.py" --check-config >/dev/null 2>&1 \
      || { red "触觉配对配置校验失败"; fail=1; }
    "${PY}" - <<'PY' || fail=1
import os

from tactile_pairing import _load_pyserial
from tactile_probe import discover_candidates

try:
    _, list_ports = _load_pyserial()
    candidates, _ = discover_candidates(list_ports, ())
except Exception as exc:
    print("[doctor] 触觉串口枚举失败: {}: {}".format(type(exc).__name__, exc))
    raise SystemExit(1)

summary = ", ".join(candidate.device for candidate in candidates) or "none"
print("[doctor] 触觉候选: {} ({})".format(len(candidates), summary))
if len(candidates) != 2:
    print("[doctor] 必须恰好连接两只 VID:PID=1A86:7523 触觉设备")
    raise SystemExit(1)
denied = [candidate.device for candidate in candidates if not os.access(candidate.device, os.R_OK | os.W_OK)]
if denied:
    print("[doctor] 当前用户无串口读写权限: {}；检查 dialout/udev".format(",".join(denied)))
    raise SystemExit(1)
PY
  fi
  local timeout_decimal=""
  if [[ "${TACTILE_PHASE_TIMEOUT}" =~ ^[0-9]+$ ]]; then
    timeout_decimal=$((10#${TACTILE_PHASE_TIMEOUT}))
    TACTILE_PHASE_TIMEOUT="${timeout_decimal}"
  fi
  if [[ -z "${timeout_decimal}" ]] \
      || (( timeout_decimal < 5 || timeout_decimal > 300 )); then
    red "TACTILE_PHASE_TIMEOUT 必须是 5..300 的整数秒"
    fail=1
  fi
  [[ -x "${HERE}/manus_ndjson_bridge/manus_ndjson_bridge.out" ]] \
    || ylw "[doctor] bridge 未编译（start 时会自动编）"
  [[ -s "${HERE}/config/manus_left.mcal" ]] || { red "[doctor] 缺 config/manus_left.mcal"; fail=1; }
  [[ -s "${HERE}/config/manus_right.mcal" ]] || { red "[doctor] 缺 config/manus_right.mcal"; fail=1; }
  [[ -f "${HERE}/config/calib_wrist.json" ]] || ylw "[doctor] 缺 calib_wrist.json"
  return "${fail}"
}

doctor() {
  hdr "DOCTOR"
  local fail=0
  doctor_quiet || fail=1
  if ! command -v "${PY}" >/dev/null 2>&1; then
    return 1
  fi
  echo "python: $(${PY} -V 2>&1)"
  "${PY}" - <<'PY'
mods = ["numpy", "cv2", "h5py", "meshcat", "serial"]
for m in mods:
    try:
        mod = __import__(m)
        ver = getattr(mod, "__version__", getattr(getattr(mod, "version", None), "version", "?"))
        print(f"  OK  {m} {ver}")
    except Exception as e:
        print(f"  --  {m}: {e}")
PY
  echo "calib files:"
  ls -lah config/*.mcal config/calib_wrist.json config/pico_cam/vst_cam.json 2>&1 | sed 's/^/  /' || true
  echo "key scripts:"
  for f in pico_receiver.py manus_collector.py pico_record.py align_pico_manus.py \
           export_dataset.py analyze_quality.py pico_controller_viz.py \
           tactile_protocol.py tactile_probe.py tactile_pairing.py tactile_collector.py; do
    if [[ -f "$f" ]]; then echo "  OK $f"; else red "  MISS $f"; fi
  done
  status_brief || true
  return "${fail}"
}

_health_check() {
  hdr "HEALTH CHECK"
  local fail=0
  local st=""

  echo "[1] 服务探活"
  if "${PY}" "${HERE}/pico_record.py" ping >/dev/null 2>&1; then
    grn "  PICO+MANUS+TACTILE control PONG"
  else
    red "  控制口失败 — 先 bash scripts/ego_ctl.sh start"
    fail=1
  fi
  if ! st="$("${PY}" "${HERE}/pico_record.py" status 2>&1)"; then
    fail=1
  fi
  printf '%s\n' "${st}" | sed 's/^/  /'

  echo "[2] 端口监听"
  for p in ${PICO_CTRL} ${MANUS_CTRL} ${TACTILE_CTRL} ${PICO_TRACK_PORT}; do
    if ss -lntp 2>/dev/null | grep -q ":${p}"; then
      grn "  :${p} LISTEN"
    else
      red "  :${p} 未监听"
      fail=1
    fi
  done
  if ss -lntp 2>/dev/null | grep -q ":${PICO_VIDEO_PORT}"; then
    grn "  :${PICO_VIDEO_PORT} LISTEN (VST)"
  else
    ylw "  :${PICO_VIDEO_PORT} 未监听（未开 --video 时正常）"
  fi

  echo "[3] 触觉配对 / 双路原始流"
  if _tactile_status_ready health; then
    grn "  左手按压绑定有效；右手排除绑定有效；双路流健康"
  else
    red "  触觉不是健康的 PAIRED_IDLE/RECORDING_RAW 状态"
    fail=1
  fi
  local tactile_line
  tactile_line="$(printf '%s\n' "${st}" | grep '^\[TACTILE\]' | head -1 || true)"
  [[ -n "${tactile_line}" ]] && echo "  ${tactile_line}"

  echo "[4] MANUS bridge / 手套"
  if pgrep -f "manus_ndjson_bridge.out" >/dev/null; then
    grn "  bridge 进程在跑"
  else
    ylw "  bridge 未跑（手套未连或 collector 刚起）"
  fi
  if echo "${st}" | grep -qi "gloves=left,right"; then
    grn "  双手套在线"
  elif echo "${st}" | grep -qi "gloves="; then
    ylw "  手套状态: $(echo "${st}" | grep MANUS)"
  else
    ylw "  手套状态未知（看 manus_service.log）"
  fi

  echo "[5] 头显设备"
  if echo "${st}" | grep -qi "devices=(none)"; then
    ylw "  头显未连接 — 明天戴上 App 连本机后应出现 SN"
  elif echo "${st}" | grep -qi "devices="; then
    grn "  $(echo "${st}" | grep PICO | head -1)"
  fi

  echo "[6] 最近会话文件配对"
  local latest sess
  latest="$(ls -1td "${SESSIONS_DIR}"/*/raw/pico.jsonl "${SESSIONS_DIR}"/*/*/raw/pico.jsonl \
    2>/dev/null | head -1 || true)"
  if [[ -z "${latest}" ]]; then
    # 兼容旧 logs/pico_*.jsonl
    latest="$(ls -1t "${LOG_DIR}"/pico_*.jsonl 2>/dev/null | head -1 || true)"
    if [[ -z "${latest}" ]]; then
      ylw "  data/sessions/ 尚无会话"
    else
      local base
      base="$(basename "${latest}")"
      sess="${base#pico_}"; sess="${sess%.jsonl}"
      echo "  latest session (legacy logs/): ${sess}"
      for kind in pico manus; do
        local path="${LOG_DIR}/${kind}_${sess}.jsonl"
        [[ -f "${path}" ]] && grn "  OK ${path}" || { red "  缺 ${path}"; fail=1; }
      done
      [[ -f "${LOG_DIR}/vst_${sess}.h264" ]] && grn "  OK vst" || ylw "  缺 vst"
      ylw "  legacy logs 会话没有统一触觉资产约束"
    fi
  else
    local session_dir rel
    session_dir="$(dirname "${latest}")"
    local task_dir="$(dirname "${session_dir}")"
    rel="${task_dir#${SESSIONS_DIR}/}"
    if [[ "${rel}" == */* ]]; then
      sess="${rel%%/*}_${rel#*/}"
    else
      sess="${rel}"
    fi
    echo "  latest session: ${sess}  (${session_dir}/)"
    for f in pico.jsonl manus.jsonl vst.h264 vst.ts.jsonl; do
      local path="${session_dir}/${f}"
      if [[ -f "${path}" ]]; then
        grn "  OK ${path} ($(du -h "${path}" | awk '{print $1}'))"
      else
        if [[ "${f}" == vst* ]]; then
          ylw "  缺 ${path}"
        else
          red "  缺 ${path}"; fail=1
        fi
      fi
    done
    local tactile_dir="${session_dir}"
    if [[ -f "${tactile_dir}/tactile.jsonl" && -f "${tactile_dir}/tactile.meta.json" ]]; then
      grn "  OK ${tactile_dir}/tactile.jsonl"
      grn "  OK ${tactile_dir}/tactile.meta.json"
    elif [[ -f "${tactile_dir}/tactile.jsonl.partial" ]]; then
      ylw "  触觉会话仍在录制或未完整封存: ${tactile_dir}/tactile.jsonl.partial"
    else
      ylw "  缺触觉资产（旧会话或使用了 --no-tactile）: ${tactile_dir}"
    fi
  fi

  echo "[7] 语法 / 关键模块"
  local f compile_fail=0
  for f in pico_receiver.py manus_collector.py pico_record.py align_pico_manus.py \
           export_dataset.py analyze_quality.py record_control.py pico_retarget.py \
           tactile_protocol.py tactile_probe.py tactile_pairing.py tactile_collector.py; do
    if ${PY} -m py_compile "${f}" 2>/dev/null; then
      : # ok
    else
      red "  py_compile FAIL ${f}"
      compile_fail=1
      fail=1
    fi
  done
  if [[ "${compile_fail}" -eq 0 ]]; then
    grn "  核心 .py 可编译"
  fi

  if [[ "${fail}" -eq 0 ]]; then
    grn "[health] PASS（头显未连时 devices=(none) 属预期）"
  else
    red "[health] 有失败项，见上"
  fi
  return "${fail}"
}

_pipeline() {
  local sess="${1:-}"
  if [[ -z "${sess}" ]]; then
    local latest
    latest="$(ls -1td "${SESSIONS_DIR}"/*/raw/pico.jsonl "${SESSIONS_DIR}"/*/*/raw/pico.jsonl \
      2>/dev/null | head -1 || true)"
    if [[ -n "${latest}" ]]; then
      local latest_dir latest_rel
      latest_dir="$(dirname "$(dirname "${latest}")")"
      latest_rel="${latest_dir#${SESSIONS_DIR}/}"
      if [[ "${latest_rel}" == */* ]]; then
        sess="${latest_rel%%/*}_${latest_rel#*/}"
      else
        sess="${latest_rel}"
      fi
    else
      latest="$(ls -1t "${LOG_DIR}"/pico_*.jsonl 2>/dev/null | head -1 || true)"
      [[ -n "${latest}" ]] || { red "无会话，请指定 session"; exit 1; }
      sess="$(basename "${latest}")"; sess="${sess#pico_}"; sess="${sess%.jsonl}"
    fi
  fi
  hdr "PIPELINE session=${sess}"

  local pico manus aligned hdf5 report tactile tactile_meta tactile_dir
  local batch_prefix="" batch_index="" grouped_raw=""
  if [[ "${sess}" =~ ^(.+)_([0-9]{3,4})$ ]]; then
    batch_prefix="${BASH_REMATCH[1]}"
    batch_index="${BASH_REMATCH[2]}"
    grouped_raw="${RAW_DIR}/${batch_prefix}/${batch_index}"
  fi
  local task_dir=""
  if [[ -n "${batch_prefix}" ]]; then
    task_dir="${SESSIONS_DIR}/${batch_prefix}/${batch_index}"
  else
    task_dir="${SESSIONS_DIR}/${sess}"
  fi
  if [[ -f "${task_dir}/raw/pico.jsonl" ]]; then
    pico="${task_dir}/raw/pico.jsonl"
    manus="${task_dir}/raw/manus.jsonl"
    aligned="${task_dir}/aligned.jsonl"
    hdf5="${task_dir}/dataset.hdf5"
    report="${task_dir}/pipeline_report.txt"
    tactile_dir="${task_dir}/raw"
  elif [[ -n "${grouped_raw}" && -f "${grouped_raw}/pico.jsonl" ]]; then
    pico="${grouped_raw}/pico.jsonl"
    manus="${grouped_raw}/manus.jsonl"
    aligned="${ALIGNED_DIR}/${batch_prefix}/${batch_index}.jsonl"
    hdf5="${EXPORT_DIR}/${batch_prefix}/${batch_index}.hdf5"
    report="${EXPORT_DIR}/${batch_prefix}/pipeline_${batch_index}_report.txt"
    tactile_dir="${TACTILE_RAW_DIR}/${batch_prefix}/${batch_index}"
    mkdir -p "$(dirname "${aligned}")" "$(dirname "${hdf5}")"
  elif [[ -f "${RAW_DIR}/${sess}/pico.jsonl" ]]; then
    pico="${RAW_DIR}/${sess}/pico.jsonl"
    manus="${RAW_DIR}/${sess}/manus.jsonl"
    aligned="${ALIGNED_DIR}/${sess}.jsonl"
    hdf5="${EXPORT_DIR}/${sess}_check.hdf5"
    report="${EXPORT_DIR}/pipeline_${sess}_report.txt"
    tactile_dir="${TACTILE_RAW_DIR}/${sess}"
  else
    pico="${LOG_DIR}/pico_${sess}.jsonl"
    manus="${LOG_DIR}/manus_${sess}.jsonl"
    aligned="${LOG_DIR}/data/aligned/aligned_${sess}.jsonl"
    hdf5="${LOG_DIR}/data/export/dataset_${sess}_check.hdf5"
    report="${LOG_DIR}/data/export/pipeline_${sess}_report.txt"
    tactile_dir="${TACTILE_RAW_DIR}/${sess}"
    mkdir -p "$(dirname "${aligned}")" "$(dirname "${hdf5}")"
  fi
  [[ -f "${pico}" ]] || { red "缺 ${pico}"; exit 1; }
  [[ -f "${manus}" ]] || { red "缺 ${manus}"; exit 1; }
  tactile="${tactile_dir}/tactile.jsonl"
  tactile_meta="${tactile_dir}/tactile.meta.json"
  local tactile_args=()
  if [[ -f "${tactile}" && -f "${tactile_meta}" ]]; then
    tactile_args+=(--tactile "${tactile}" --tactile-meta "${tactile_meta}")
    grn "[pipeline] 将对齐并导出同名双手触觉: ${tactile}"
  elif [[ -e "${tactile}" || -e "${tactile_meta}" || \
          -e "${tactile}.partial" || -e "${tactile_meta}.partial" ]]; then
    red "[pipeline] 触觉资产不完整，拒绝导出: ${tactile_dir}"
    exit 1
  else
    ylw "[pipeline] 未找到同名触觉资产；按旧会话/--no-tactile 兼容模式导出"
  fi
  mkdir -p "${ALIGNED_DIR}" "${EXPORT_DIR}" "${REVIEW_DIR}"

  {
    echo "=== pipeline ${sess} $(date -Is) ==="
    echo
    echo "--- analyze_quality ---"
    ${PY} analyze_quality.py "${pico}" "${manus}"
    echo
    echo "--- align ---"
    ${PY} align_pico_manus.py "${pico}" "${manus}" -o "${aligned}" \
      --full --max-skew-ms 30 --gate-ms 40 --tactile-gate-ms 40 \
      "${tactile_args[@]}"
    echo
    echo "--- export egodex_v1 ---"
    local vst_args=()
    if [[ -f "$(dirname "${pico}")/vst.h264" ]]; then
      vst_args+=(--vst "$(dirname "${pico}")/vst.h264")
    fi
      if [[ -f "$(dirname "${pico}")/vst.ts.jsonl" ]]; then
        vst_args+=(--vst-ts "$(dirname "${pico}")/vst.ts.jsonl")
      fi
      if [[ -f "$(dirname "${pico}")/vst.qpc.ts.jsonl" ]]; then
        vst_args+=(--vst-qpc-ts "$(dirname "${pico}")/vst.qpc.ts.jsonl")
      fi
    ${PY} export_dataset.py "${pico}" "${manus}" -o "${hdf5}" \
      --calib config/calib_wrist.json \
      --gate-ms 40 --video-gate-ms 40 --tactile-gate-ms 40 \
      --min-hand-coverage 0.95 --min-video-coverage 0.95 \
      --min-tactile-coverage 0.95 --min-complete-coverage 0.95 \
      --max-p95-skew-ms 30 \
      --max-skew-ms 40 --fps 30 \
      "${tactile_args[@]}" "${vst_args[@]}"
    echo
    echo "--- catalog ---"
    ${PY} data_catalog.py "${sess}" --write-manifest || true
    echo
    echo "aligned: ${aligned}"
    echo "hdf5:    ${hdf5}"
    if (( ${#tactile_args[@]} > 0 )); then
      echo "tactile: ${tactile}"
    fi
    ls -lah "${aligned}" "${hdf5}" 2>/dev/null || true
  } 2>&1 | tee "${report}"

  grn "[pipeline] 报告: ${report}"
  echo "文档: README.md | docs/PIPELINE_ZH.md | docs/CALIB_QA_ZH.md"
}

case "${1:-}" in
  start|restart) _lock_lifecycle; start_all ;;
  stop)     _lock_lifecycle; stop_all ;;
  status)   status_brief ;;
  check|health) _health_check ;;
  pipeline) _pipeline "${2:-}" ;;
  doctor)   doctor ;;
  *)
    cat <<EOF
用法: $0 {start|restart|stop|status|check|pipeline|doctor} [session]

  start/restart   交互完成左手触觉配对，并启动 PICO+MANUS+TACTILE
  stop / status / check / doctor
  pipeline [名]   align + export(egodex_v1,含视频帧号/同名触觉) + catalog

新数据目录: data/sessions/<任务前缀>/<序号>/（旧类型优先目录继续兼容读取）
文档仅三份: README.md | docs/PIPELINE_ZH.md | docs/CALIB_QA_ZH.md

环境变量: PICO_VIDEO=0 关闭 VST；PY=python3；TACTILE_PHASE_TIMEOUT=30；TACTILE_RATE_HZ=60
EOF
    exit 1
    ;;
esac
