"""Slipway's modelled gimbal as the driver (crowsnest-dev#300, Ted 2026-11-06): the same `Driver` seam as
the simulated one, so the gate does not change.

THE DRIVE IS A RAW, SIMULATOR-ONLY RECORD, standing in for the wire to the gimbal's motor controller, the
way Slipway's `set_lockout` stands in for the switch on board. Every cycle the driver publishes
`raw_json/gimbal_drive/{payload_id}` on the vessel, a keelson TimestampedString of JSON:

    {"mode": "DRIVEN", "axes": {"pan": 40.0, "tilt": -10.0}}   # per axis, percent of its range
    {"mode": "HOLD_POSITION"}                                    # a safe state: stop where it is

Slipway slews toward the target at the axis rate, holds where it is on a safe state, and holds on its own
if the drive goes silent (its motor controller's watchdog). It is never a second gate: payload_control/v1
is answered here and nowhere else.

THE ANGLES ARE WHAT SLIPWAY SAYS, from her truth record (`raw_json/truth`, `gimbals.{payload_id}`), never
what was asked: `read()` reports the last record heard, or nothing before one.
"""

from __future__ import annotations

import json
import threading
from collections.abc import Callable


class SlipwayGimbal:
    def __init__(self, payload_id: str, publish: Callable[[str], None]):
        self.payload_id = payload_id
        self._publish = publish  # (json text) -> None: puts the drive record on the bus
        self._lock = threading.Lock()
        self._heard: dict | None = None

    def apply(self, values: dict, dt_s: float) -> None:
        """`values` is axis key (`pan/<payload_id>`) -> %. Only this payload's axes are driven."""
        axes = {}
        for key, pct in values.items():
            axis, _, pid = key.partition("/")
            if pid in ("", self.payload_id):
                axes[axis] = float(pct)
        self._publish(json.dumps({"mode": "DRIVEN", "axes": axes}))

    def safe_state(self, action: str, dt_s: float) -> None:
        """HOLD_POSITION / INHIBIT (and any other policy): Slipway stops the gimbal where it is."""
        self._publish(json.dumps({"mode": action}))

    def on_truth(self, record: dict) -> None:
        """Her truth record, as decoded JSON. Keeps this payload's gimbal entry, if she reports one."""
        g = (record.get("gimbals") or {}).get(self.payload_id)
        if isinstance(g, dict):
            with self._lock:
                self._heard = dict(g)

    def read(self) -> dict:
        with self._lock:
            return dict(self._heard) if self._heard is not None else {}
