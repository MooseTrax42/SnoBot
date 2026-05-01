"""
Unified message schema for SBCP v0.3.0.
Defines a common envelope and helpers for legacy compatibility.
"""

from typing import Any, Dict, Optional, Tuple
from sbcp.types import PROTOCOL_VERSION

Envelope = Dict[str, Any]

def make_envelope(msg_type: str, data: Dict[str, Any], seq: Optional[int] = None, ts_ms: Optional[int] = None) -> Envelope:
    """
    Create a standard message envelope.

    Args:
        msg_type: Message type string (CMD, INTENT, STAT, ACK, FAULT, etc.)
        data: Message payload dictionary
        seq: Optional sequence number
        ts_ms: Optional timestamp in ms
    """
    env: Envelope = {
        "type": msg_type,
        "ver": PROTOCOL_VERSION,
        "data": data,
    }
    if seq is not None:
        env["seq"] = seq
    if ts_ms is not None:
        env["ts"] = ts_ms
    return env


def parse_envelope(msg: Dict[str, Any]) -> Tuple[str, Dict[str, Any], Dict[str, Any]]:
    """
    Parse a message and return (type, data, meta).
    Requires the unified envelope format.
    """
    if "type" not in msg or "data" not in msg:
        raise ValueError("Invalid message: expected envelope with 'type' and 'data'")

    meta = {k: v for k, v in msg.items() if k not in ("data",)}
    return msg["type"], msg["data"], meta
