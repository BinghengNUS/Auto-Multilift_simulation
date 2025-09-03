"""
Lightweight IPC transport abstraction to support both shared-memory (intra-host)
and UDP multicast socket (inter-process / inter-host) communication.

This module is designed to be imported by nodes that wish to exchange small
planning/MPC payloads without introducing heavy dependencies. It intentionally
keeps the API minimal and synchronous to reduce integration complexity.

Usage (UDP example):
  bus = UdpTransport(group="239.255.0.1", port=50000, iface_ip="0.0.0.0")
  bus.send({"topic": "xq_fb", "drone": 0, "data": xq_fb.tolist()})
  msgs = bus.recv_nonblock(max_msgs=16)
"""
from __future__ import annotations

import socket
import struct
import json
from typing import Any, Dict, List, Optional


class Transport:
    """Abstract transport interface."""

    def send(self, payload: Dict[str, Any]) -> None:  # pragma: no cover - interface
        raise NotImplementedError

    def recv_nonblock(self, max_msgs: int = 16) -> List[Dict[str, Any]]:  # pragma: no cover - interface
        raise NotImplementedError


class UdpTransport(Transport):
    """
    UDP multicast transport.

    - Payloads are JSON-encoded dicts (utf-8). For arrays, pass list() to avoid
      numpy dependency at this layer; callers可在外层自行转换。
    - Non-blocking recv: returns at most max_msgs packets currently queued.
    - Designed for LAN/loopback prototyping；真实部署可按需替换为更强健的中间件。
    """

    def __init__(self, group: str = "239.255.0.1", port: int = 50000, iface_ip: str = "0.0.0.0") -> None:
        self.group = group
        self.port = port
        self.iface_ip = iface_ip

        # sender
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.tx.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)

        # receiver
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.rx.bind(("", port))  # all interfaces
        except OSError:
            # some systems require binding to group
            self.rx.bind((group, port))
        mreq = struct.pack("4sl", socket.inet_aton(group), socket.INADDR_ANY)
        self.rx.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        self.rx.setblocking(False)

    def send(self, payload: Dict[str, Any]) -> None:
        data = json.dumps(payload, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
        self.tx.sendto(data, (self.group, self.port))

    def recv_nonblock(self, max_msgs: int = 16) -> List[Dict[str, Any]]:
        out: List[Dict[str, Any]] = []
        for _ in range(max_msgs):
            try:
                buf, _ = self.rx.recvfrom(65535)
            except BlockingIOError:
                break
            try:
                out.append(json.loads(buf.decode("utf-8")))
            except Exception:
                # drop malformed packet
                continue
        return out


