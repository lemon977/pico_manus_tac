#!/usr/bin/env bash
# 兼容入口：生命周期逻辑只维护在 scripts/ego_ctl.sh，避免两套脚本并发或行为漂移。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "${SCRIPT_DIR}/scripts/ego_ctl.sh" "$@"
