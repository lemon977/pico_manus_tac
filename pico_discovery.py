#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PICO XRoboToolkit discovery broadcaster.

This process is deliberately independent from ``pico_receiver.py``.  It keeps
advertising every usable local IPv4 address so a headset can rediscover the PC
after sleep, a cable reconnect, or a Windows network-interface change.
"""

from __future__ import annotations

import argparse
import ipaddress
import json
import locale
import logging
from logging.handlers import RotatingFileHandler
import os
import re
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Iterable


HEAD_SERVER = 0xCF
TAIL = 0xA5
BCAST_CMD_TCPIP = 0x7E
BCAST_UDP_PORT = 29888
DEFAULT_INTERVAL = 1.0
DEFAULT_RESCAN_INTERVAL = 5.0
DEFAULT_HEARTBEAT_INTERVAL = 30.0
IPV4_PATTERN = re.compile(
    r"(?<![0-9])(?:25[0-5]|2[0-4][0-9]|1?[0-9]?[0-9])"
    r"(?:\.(?:25[0-5]|2[0-4][0-9]|1?[0-9]?[0-9])){3}(?![0-9])"
)
LOGGER = logging.getLogger("pico_discovery")


def configure_logging(log_file: str = "") -> None:
    handlers: list[logging.Handler] = []
    if sys.stderr is not None:
        handlers.append(logging.StreamHandler())
    if log_file:
        path = Path(log_file)
        path.parent.mkdir(parents=True, exist_ok=True)
        handlers.append(
            RotatingFileHandler(
                path, maxBytes=2 * 1024 * 1024, backupCount=3, encoding="utf-8"
            )
        )
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        handlers=handlers,
        force=True,
    )


def pack_discovery_frame(ip: str, timestamp_s: int | None = None) -> bytes:
    """Pack the PC->PICO discovery frame used by XRoboToolkit."""
    payload = ip.encode("ascii")
    timestamp = int(time.time()) if timestamp_s is None else int(timestamp_s)
    return (
        struct.pack("<BBI", HEAD_SERVER, BCAST_CMD_TCPIP, len(payload))
        + payload
        + struct.pack("<QB", timestamp, TAIL)
    )


def is_usable_ipv4(value: str) -> bool:
    try:
        address = ipaddress.IPv4Address(value)
    except ipaddress.AddressValueError:
        return False
    return not (
        address.is_loopback
        or address.is_unspecified
        or address.is_multicast
        or address == ipaddress.IPv4Address("255.255.255.255")
    )


def _hostname_ipv4s() -> set[str]:
    found: set[str] = set()
    try:
        answers = socket.getaddrinfo(
            socket.gethostname(), None, socket.AF_INET, socket.SOCK_DGRAM
        )
    except OSError:
        return found
    for answer in answers:
        value = answer[4][0]
        if is_usable_ipv4(value):
            found.add(value)
    return found


def _route_probe_ipv4s() -> set[str]:
    """Ask the routing table for likely egress addresses without sending data."""
    found: set[str] = set()
    for target in (("1.1.1.1", 80), ("8.8.8.8", 80)):
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.connect(target)
            value = probe.getsockname()[0]
            if is_usable_ipv4(value):
                found.add(value)
        except OSError:
            pass
        finally:
            probe.close()
    return found


def parse_ipconfig_ipv4s(output: str) -> set[str]:
    """Extract interface IPv4 addresses from localized Windows ipconfig output."""
    found: set[str] = set()
    for line in output.splitlines():
        # "IPv4" remains stable in both Chinese and English ipconfig output.
        if "IPv4" not in line:
            continue
        for value in IPV4_PATTERN.findall(line):
            if is_usable_ipv4(value):
                found.add(value)
    return found


def _windows_ipconfig_ipv4s() -> set[str]:
    if os.name != "nt":
        return set()
    try:
        result = subprocess.run(
            ["ipconfig.exe"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            check=False,
            timeout=4.0,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        )
    except (OSError, subprocess.SubprocessError):
        return set()
    encoding = locale.getpreferredencoding(False) or "utf-8"
    output = result.stdout.decode(encoding, errors="replace")
    return parse_ipconfig_ipv4s(output)


def discover_local_ipv4s() -> list[str]:
    """Return all usable addresses, including USB/RNDIS adapters on Windows."""
    values = _hostname_ipv4s() | _route_probe_ipv4s() | _windows_ipconfig_ipv4s()
    return sorted(values, key=lambda item: int(ipaddress.IPv4Address(item)))


def broadcast_targets(ip: str) -> tuple[str, ...]:
    """Return limited plus legacy /24 directed-broadcast destinations."""
    octets = ip.split(".")
    directed = ".".join((*octets[:3], "255"))
    if directed == "255.255.255.255":
        return (directed,)
    return (directed, "255.255.255.255")


def send_broadcast_cycle(
    ips: Iterable[str],
    port: int = BCAST_UDP_PORT,
    socket_factory: Callable[..., socket.socket] = socket.socket,
) -> tuple[int, int]:
    """Broadcast each advertised IP through the same source interface."""
    sent = 0
    errors = 0
    for ip in ips:
        broadcaster = socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            broadcaster.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            # Binding prevents a multi-NIC Windows host from advertising one
            # adapter's address through a different adapter.
            broadcaster.bind((ip, 0))
            frame = pack_discovery_frame(ip)
            for target in broadcast_targets(ip):
                try:
                    broadcaster.sendto(frame, (target, port))
                    sent += 1
                except OSError:
                    errors += 1
        except OSError:
            errors += 1
        finally:
            broadcaster.close()
    return sent, errors


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def write_status(path: Path, state: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps(state, ensure_ascii=False, separators=(",", ":")),
        encoding="utf-8",
    )
    temporary.replace(path)


def run_broadcaster(args: argparse.Namespace) -> int:
    stop = threading.Event()

    def request_stop(_signum=None, _frame=None):
        stop.set()

    for signum in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(signum, request_stop)
        except (OSError, ValueError):
            pass

    ips: list[str] = []
    previous_ips: tuple[str, ...] | None = None
    cycles = 0
    total_sent = 0
    total_errors = 0
    next_scan = 0.0
    next_heartbeat = 0.0
    next_status = 0.0
    status_path = Path(args.status_file) if args.status_file else None
    started_utc = _utc_now()

    LOGGER.info(
        f"[pico-discovery] started udp={args.port} interval={args.interval:g}s "
        f"rescan={args.rescan_interval:g}s"
    )
    while not stop.is_set():
        now = time.monotonic()
        if now >= next_scan:
            ips = discover_local_ipv4s()
            next_scan = now + args.rescan_interval
            current_ips = tuple(ips)
            if current_ips != previous_ips:
                label = ",".join(ips) if ips else "(none)"
                LOGGER.info(f"[pico-discovery] local_ipv4={label}")
                previous_ips = current_ips

        if args.dry_run:
            cycle_sent, cycle_errors = 0, 0
        else:
            cycle_sent, cycle_errors = send_broadcast_cycle(ips, args.port)
        cycles += 1
        total_sent += cycle_sent
        total_errors += cycle_errors
        now = time.monotonic()

        if now >= next_heartbeat:
            label = ",".join(ips) if ips else "(none)"
            LOGGER.info(
                f"[pico-discovery] heartbeat ips={label} cycles={cycles} "
                f"sent={total_sent} errors={total_errors}"
            )
            next_heartbeat = now + args.heartbeat_interval

        if status_path is not None and now >= next_status:
            try:
                write_status(
                    status_path,
                    {
                        "schema": "pico_discovery_status_v1",
                        "pid": os.getpid(),
                        "started_utc": started_utc,
                        "updated_utc": _utc_now(),
                        "ips": ips,
                        "cycles": cycles,
                        "sent": total_sent,
                        "errors": total_errors,
                        "dry_run": bool(args.dry_run),
                    },
                )
            except OSError as exc:
                total_errors += 1
                LOGGER.warning(f"[pico-discovery] status_write_error={exc}")
            next_status = now + args.status_interval

        if args.once:
            break
        stop.wait(args.interval)

    LOGGER.info(
        f"[pico-discovery] stopped cycles={cycles} sent={total_sent} "
        f"errors={total_errors}"
    )
    return 0


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return parsed


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="持续广播本机 IP，供 PICO 自动发现和重连")
    parser.add_argument("--port", type=int, default=BCAST_UDP_PORT)
    parser.add_argument("--interval", type=positive_float, default=DEFAULT_INTERVAL)
    parser.add_argument(
        "--rescan-interval", type=positive_float, default=DEFAULT_RESCAN_INTERVAL
    )
    parser.add_argument(
        "--heartbeat-interval", type=positive_float,
        default=DEFAULT_HEARTBEAT_INTERVAL,
    )
    parser.add_argument("--status-interval", type=positive_float, default=5.0)
    parser.add_argument("--status-file", default="")
    parser.add_argument("--log-file", default="")
    parser.add_argument("--once", action="store_true", help="仅执行一个广播周期")
    parser.add_argument("--dry-run", action="store_true", help="只检测地址，不发送广播")
    args = parser.parse_args(argv)
    if not (1 <= args.port <= 65535):
        parser.error("--port must be between 1 and 65535")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    configure_logging(args.log_file)
    try:
        return run_broadcaster(args)
    except Exception:
        LOGGER.exception("[pico-discovery] fatal_error")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
