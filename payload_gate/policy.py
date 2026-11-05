"""The payload's entry, from the vessel's platform config (`payloads[]`, keelson #360's
connectors/platform/config-schema.json), plus the gate's own timings. Platform registry data, which
changes only in maintenance (payload_control `not_solved`, "The values")."""

from __future__ import annotations

import json
from pathlib import Path

from .model import SAFE_STATES, Policy

KINDS = ("camera_gimbal", "winch", "crane", "sonar", "sampler", "launcher", "other")


def parse(platform: dict, payload_id: str, vessel_id: str, gate: dict | None = None) -> Policy:
    """`platform` is the vessel's platform config; `payload_id` picks the entry. A missing entry, an
    unknown kind or a missing / unknown safe-state policy refuses to start: a gate that does not know
    what its safe state is must not run."""
    gate = gate or {}
    entries = [p for p in (platform.get("payloads") or []) if p.get("id") == payload_id]
    if len(entries) != 1:
        raise ValueError(f"the platform declares no payload {payload_id!r} (or declares it twice)")
    p = entries[0]
    if p.get("kind") not in KINDS:
        raise ValueError(f"payload {payload_id!r}: unknown kind {p.get('kind')!r}")
    safe = p.get("payload_safe_state_policy")
    if safe not in SAFE_STATES:
        raise ValueError(f"payload {payload_id!r}: payload_safe_state_policy must be one of {', '.join(SAFE_STATES)}")
    axes = tuple((a["id"], bool(a.get("unipolar", False))) for a in (p.get("axes") or []))
    if not axes:
        raise ValueError(f"payload {payload_id!r} declares no axes")
    return Policy(
        vessel_id=vessel_id,
        payload_id=payload_id,
        kind=p["kind"],
        axes=axes,
        affects_stability=bool(p.get("affects_stability", False)),
        affects_navigation=bool(p.get("affects_navigation", False)),
        safe_state=safe,
        max_axis_age_ms=int(gate.get("max_axis_age_ms", 1000)),
        safety_ttl_ms=int(gate.get("safety_ttl_ms", 15000)),
        require_safety_state=bool(gate.get("require_safety_state", True)),
        motion_fence_ttl_ms=int(gate.get("motion_fence_ttl_ms", 2000)),
        ship_ttl_ms=int(gate.get("ship_ttl_ms", 5000)),
    )


def load(platform_path: Path, payload_id: str, vessel_id: str, gate_path: Path | None = None) -> Policy:
    gate = json.loads(Path(gate_path).read_text()) if gate_path else {}
    return parse(json.loads(Path(platform_path).read_text()), payload_id, vessel_id, gate)
