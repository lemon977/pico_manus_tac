#!/usr/bin/env bash
# setup_manus_bridge.sh — 准备并编译无 ROS 的 MANUS 采集桥。
#
# 前置：MANUS Linux SDK 默认放在:
#   优先 ~/pico_controller/vendor/manus_sdk/
#   若已编译过, bridge 内 ManusSDK/ 已够用, 不必再拷。
# 也可用环境变量 MANUS_SDK_MIN 指定其它路径。
#
# 用法：bash scripts/setup_manus_bridge.sh [--udev]
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BRIDGE_DIR="${HERE}/manus_ndjson_bridge"
SDK_MIN="${MANUS_SDK_MIN:-${HERE}/vendor/manus_sdk}"

INSTALL_UDEV="false"
for a in "$@"; do
  case "$a" in
    --udev) INSTALL_UDEV="true" ;;
  esac
done

echo "[setup] 桥目录: ${BRIDGE_DIR}"

if [[ -d "${SDK_MIN}" ]]; then
  echo "[setup] SDK 源: ${SDK_MIN}"
  cp -r "${SDK_MIN}/ManusSDK" "${BRIDGE_DIR}/ManusSDK"
  for f in ClientPlatformSpecific.cpp ClientPlatformSpecific.hpp \
           ClientPlatformSpecificTypes.hpp ClientLogging.hpp; do
    if [[ -f "${SDK_MIN}/${f}" ]]; then
      cp "${SDK_MIN}/${f}" "${BRIDGE_DIR}/${f}"
    else
      echo "[setup] 警告：缺少 ${f}" >&2
    fi
  done
elif [[ -x "${BRIDGE_DIR}/manus_ndjson_bridge.out" \
      && -f "${BRIDGE_DIR}/ManusSDK/lib/libManusSDK_Integrated.so" ]]; then
  echo "[setup] 使用已有 bridge 内 ManusSDK（未找到 ${SDK_MIN}）"
else
  echo "[setup] 未找到 SDK 目录: ${SDK_MIN}" >&2
  echo "        且 bridge 内也没有可用 ManusSDK；请拷 SDK 或设 MANUS_SDK_MIN。" >&2
  exit 1
fi

if [[ ! -f "${BRIDGE_DIR}/ManusSDK/lib/libManusSDK_Integrated.so" ]]; then
  echo "[setup] 警告：ManusSDK/lib 下没有 libManusSDK_Integrated.so" >&2
fi

if [[ "${INSTALL_UDEV}" == "true" ]]; then
  echo "[setup] 安装 MANUS dongle udev 规则（需要 sudo）"
  RULE=/etc/udev/rules.d/70-manus-hid.rules
  sudo bash -c "cat > ${RULE}" <<'EOF'
# MANUS Metaglove/Quantum Dongle (glove data + SDK license, gen2 single dongle)
SUBSYSTEMS=="usb", ATTRS{idVendor}=="3325", MODE:="0666"
# (可选) 老款分离式 Nordic 授权狗
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="83fd", MODE:="0666"
# HIDAPI/hidraw
KERNEL=="hidraw*", ATTRS{idVendor}=="3325", MODE:="0666"
EOF
  sudo udevadm control --reload-rules
  sudo udevadm trigger --attr-match=idVendor=3325 || true
  sudo udevadm trigger || true
  echo "[setup] 已写入 ${RULE}"
  echo "[setup] 若 dongle 已插着仍不生效，重插一次 dongle 即可。"
fi

echo "[setup] 编译桥…"
make -C "${BRIDGE_DIR}" clean >/dev/null 2>&1 || true
make -C "${BRIDGE_DIR}"

echo "[setup] 完成：${BRIDGE_DIR}/manus_ndjson_bridge.out"
echo "[setup] 试运行：LD_LIBRARY_PATH=${BRIDGE_DIR}/ManusSDK/lib ${BRIDGE_DIR}/manus_ndjson_bridge.out"
