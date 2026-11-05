"""An in-process camera gimbal, for developing and testing the gate with no hardware (crowsnest-dev
#89, Ted 2026-11-05). Slipway models no gimbal yet; when it does, a Slipway driver implements the same
`Driver` seam and the gate does not change.

The gimbal has pan and tilt. An axis command is a PERCENTAGE of the axis's range (-100..100 maps
linearly onto [min_deg, max_deg]); each axis slews toward its target at a limited rate.

A SAFE STATE IS AN ACTION, NOT "IDLE". HOLD_POSITION holds the angles the gimbal has now (it stops
slewing where it is), and INHIBIT freezes the drive outright. Neither moves the gimbal to a
"neutral" pose: driving to zero is a movement, and the gate's whole point is that nothing moves.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol


class Driver(Protocol):
    def apply(self, values: dict, dt_s: float) -> None: ...

    def safe_state(self, action: str, dt_s: float) -> None: ...

    def read(self) -> dict: ...


@dataclass(frozen=True)
class AxisSpec:
    min_deg: float
    max_deg: float
    rate_dps: float


DEFAULT_AXES = {
    "pan": AxisSpec(-170.0, 170.0, 60.0),
    "tilt": AxisSpec(-90.0, 30.0, 40.0),
}


@dataclass
class SimGimbal:
    payload_id: str
    axes: dict = field(default_factory=lambda: dict(DEFAULT_AXES))
    angle: dict = field(default_factory=dict)  # axis -> current deg
    target: dict = field(default_factory=dict)  # axis -> commanded deg
    mode: str = "HOLD_POSITION"  # "DRIVEN" or the safe state in force

    def __post_init__(self):
        for a in self.axes:
            self.angle.setdefault(a, 0.0)
            self.target.setdefault(a, self.angle[a])

    def _deg(self, axis: str, pct: float) -> float:
        s = self.axes[axis]
        return s.min_deg + (max(-100.0, min(100.0, pct)) + 100.0) / 200.0 * (s.max_deg - s.min_deg)

    def _slew(self, dt_s: float) -> None:
        for a, s in self.axes.items():
            d = self.target[a] - self.angle[a]
            step = s.rate_dps * dt_s
            self.angle[a] += max(-step, min(step, d))

    def apply(self, values: dict, dt_s: float) -> None:
        """`values` is axis key (`pan/<payload_id>`) -> %. Unknown keys are ignored."""
        self.mode = "DRIVEN"
        for key, pct in values.items():
            axis = key.split("/", 1)[0]
            if axis in self.axes:
                self.target[axis] = self._deg(axis, pct)
        self._slew(dt_s)

    def safe_state(self, action: str, dt_s: float) -> None:
        """HOLD_POSITION / INHIBIT (and any other policy, conservatively): stop where it is."""
        self.mode = action
        for a in self.axes:
            self.target[a] = self.angle[a]

    def read(self) -> dict:
        return {"mode": self.mode, **{f"{a}_deg": round(v, 3) for a, v in self.angle.items()}}

