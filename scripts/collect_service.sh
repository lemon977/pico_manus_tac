#!/usr/bin/env bash
# 兼容旧入口：转发到 ego_ctl.sh
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "${HERE}/ego_ctl.sh" "${1:-status}"
