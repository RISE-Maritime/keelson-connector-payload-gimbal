"""The payload gate's decisions, and nothing else. Pure: no clock, no store, no Zenoh.

Normative sources: keelson `protocols/payload_control.yaml`, `interfaces/PayloadControl.proto` and
`PayloadClearance.proto` (keelson #360), `CommandAuthority.proto` (the payload slot) and
`SafetyState.proto` (#332); crowsnest-dev docs/COMMAND-ARCHITECTURE.md §1.2 and §4. The work item is
crowsnest-dev #89.

THE GATE IS THE ENFORCER FOR ONE PAYLOAD. An axis value reaches the payload's driver only through
`step`, and only while every one of these holds, on the gate's own clock, every cycle:

    fresh ∧ leased ∧ fenced ∧ permitted   (and, for a payload that affects stability or navigation,
                                            a clearance whose every binding still holds)

On any failure the payload goes to ITS `payload_safe_state_policy` state -- never a bare "idle" --
and the mapping is dropped: the holder maps it again once the cause has gone.

IT OWNS ITS FENCE, `payload_epoch`. The term is a random 128-bit value drawn when the store is
(re)initialised. The generation changes when PAYLOAD AUTHORITY changes: a new holder's lease, or the
holder's lease lapsing (the gate increments alone). It does not change on a mapping, so a clearance
naming the payload_epoch outlives the holder mapping and re-mapping, and ends when the holder does.

A PAYLOAD AXIS IS NEVER A MOTION DOF. Nothing here is a vessel DOF, and nothing reaches the motion gate.

Every function takes the current `State` and returns a new one; nothing is mutated. Times are this
process's monotonic milliseconds; a source timestamp is never read here.
"""

from __future__ import annotations

import secrets
from dataclasses import dataclass, field, replace

# keelson.SafetyState.State
SAFETY_CLEAR = 1

SAFE_STATES = ("HOLD_POSITION", "BRAKE_HOLD", "HOLD_LOAD", "HOLD_WINCH", "RETRACT_OR_HOLD", "INHIBIT")

# keelson.interfaces.common.CommandResult, by name, so the model needs no keelson import.
ACCEPTED, TEMPORARILY_REJECTED, DENIED = "ACCEPTED", "TEMPORARILY_REJECTED", "DENIED"


def new_term() -> str:
    return secrets.token_hex(16)


# ---------------------------------------------------------------------------- records


@dataclass(frozen=True)
class Fence:
    term: str
    generation: int


@dataclass(frozen=True)
class Axis:
    """One mapped axis: `key` is `<axis>/<payload_id>` (keelson #327's naming); the rest is the
    ControlAxis it reads -- which subject carries its percentage, and how to read it."""

    key: str
    entity_id: str
    subject: str
    source_id: str
    unipolar: bool = False
    invert: bool = False


@dataclass(frozen=True)
class Caller:
    controller_id: str
    authority_token: str
    fence: Fence | None


@dataclass(frozen=True)
class MappingRequest:
    payload_id: str
    caller: Caller
    axes: tuple[Axis, ...]
    min_interval_s: float = 0.0
    max_axis_age_s: float = 0.0


@dataclass(frozen=True)
class Mapping:
    axes: tuple[Axis, ...]
    holder: str
    lease_token: str
    max_axis_age_ms: int
    min_interval_s: float
    clearance_id: str = ""
    mapped_ms: int = 0  # when it was mapped: a first sample has the dead-man window from here to arrive


@dataclass(frozen=True)
class Heard:
    """A lease as last heard, stamped with LOCAL receipt."""

    holder: str
    token: str
    ttl_ms: int
    heard_ms: int

    def live(self, now_ms: int) -> bool:
        return now_ms - self.heard_ms <= self.ttl_ms


@dataclass(frozen=True)
class SafetySeen:
    state: int
    local_lockout: bool
    heard_ms: int


@dataclass(frozen=True)
class ValidWhile:
    """The conditions a clearance holds under. None = no bound."""

    min_speed_knots: float | None = None
    max_speed_knots: float | None = None
    vehicle_modes: tuple[str, ...] = ()
    max_heel_deg: float | None = None


@dataclass(frozen=True)
class Clearance:
    clearance_id: str
    conn_lease_id: str
    motion_epoch: Fence | None
    payload_epoch: Fence | None
    valid_while: ValidWhile
    withdrawn: bool
    granted_by: str = ""


@dataclass(frozen=True)
class Reading:
    value: float | str
    heard_ms: int


@dataclass(frozen=True)
class AxisSample:
    value_pct: float
    recv_ms: int


@dataclass(frozen=True)
class Output:
    """One cycle: `values` (axis key -> %) when the payload may move, else the safe state and why."""

    values: dict | None
    safe_state: str | None
    reason: str


@dataclass(frozen=True)
class Policy:
    vessel_id: str
    payload_id: str
    kind: str
    axes: tuple[tuple[str, bool], ...]  # (axis id, unipolar), as payloads[] declares them
    affects_stability: bool
    affects_navigation: bool
    safe_state: str
    max_axis_age_ms: int = 1000  # the responder's own maximum: the dead-man's ceiling
    safety_ttl_ms: int = 15000
    require_safety_state: bool = True
    motion_fence_ttl_ms: int = 2000
    ship_ttl_ms: int = 5000

    @property
    def needs_clearance(self) -> bool:
        return self.affects_stability or self.affects_navigation

    @property
    def axis_keys(self) -> tuple[str, ...]:
        return tuple(f"{a}/{self.payload_id}" for a, _ in self.axes)


@dataclass(frozen=True)
class State:
    # ---- durable (store.py persists these; a reinitialised store draws a new term)
    vessel_id: str
    term: str
    generation: int = 1
    # ---- volatile: a restart starts in the safe state with no mapping, by design
    mapping: Mapping | None = None
    safe_reason: str = "no mapping"
    leases: dict = field(default_factory=dict)  # "payload" | "conn" -> Heard
    last_payload_token: str = ""  # the holder's token the current generation belongs to
    safety: SafetySeen | None = None
    clearance: Clearance | None = None
    motion_fence: tuple | None = None  # (Fence, heard_ms), from the motion gate's resolved_control
    ship: dict = field(default_factory=dict)  # "speed" | "heel" | "mode" -> Reading
    samples: dict = field(default_factory=dict)  # axis key -> AxisSample

    @property
    def fence(self) -> Fence:
        return Fence(self.term, self.generation)


def fresh(vessel_id: str) -> State:
    """A new incarnation: a new term, no mapping, the safe state."""
    return State(vessel_id=vessel_id, term=new_term())


# ---------------------------------------------------------------------------- what the gate hears


def observe_lease(st: State, slot: str, holder: str, token: str, ttl_ms: int, released: bool, now_ms: int) -> tuple[State, bool]:
    """A lease record. For the payload slot, a NEW holder's token is new payload authority: the
    generation moves (returns True, so the caller persists it), and any mapping under the old holder
    is dropped. A released record removes the lease."""
    leases = dict(st.leases)
    if released or not token:
        leases.pop(slot, None)
        return replace(st, leases=leases), False
    leases[slot] = Heard(holder, token, ttl_ms, now_ms)
    st = replace(st, leases=leases)
    if slot == "payload" and token != st.last_payload_token:
        # Any token this incarnation has not seen is new authority -- the first after a restart too,
        # since nothing says it is the holder from before. A caller then reads the new epoch.
        st = replace(st, last_payload_token=token, generation=st.generation + 1)
        return _to_safe(st, "a new payload holder"), True
    return st, False


def observe_safety(st: State, state: int, local_lockout: bool, now_ms: int) -> State:
    return replace(st, safety=SafetySeen(int(state), bool(local_lockout), now_ms))


def observe_clearance(st: State, c: Clearance | None) -> State:
    return replace(st, clearance=c)


def observe_motion_fence(st: State, fence: Fence | None, now_ms: int) -> State:
    return replace(st, motion_fence=None if fence is None else (fence, now_ms))


def observe_ship(st: State, what: str, value, now_ms: int) -> State:
    ship = dict(st.ship)
    ship[what] = Reading(value, now_ms)
    return replace(st, ship=ship)


def observe_axis(st: State, key: str, value_pct: float, now_ms: int) -> State:
    if st.mapping is None or key not in {a.key for a in st.mapping.axes}:
        return st
    samples = dict(st.samples)
    samples[key] = AxisSample(float(value_pct), now_ms)
    return replace(st, samples=samples)


# ---------------------------------------------------------------------------- the tests


def actuation_allowed(st: State, pol: Policy, now_ms: int) -> tuple[bool, str]:
    """ACTUATION_ALLOWED ⇒ CLEAR ∧ ¬local_lockout. A silent or stale supervisor is not a CLEAR one."""
    s = st.safety
    if s is None:
        return (not pol.require_safety_state), "no safety_state heard"
    if now_ms - s.heard_ms > pol.safety_ttl_ms:
        return False, "the safety_state is stale"
    if s.local_lockout:
        return False, "the local lockout is set"
    if s.state != SAFETY_CLEAR:
        return False, "the vessel is not CLEAR"
    return True, ""


def _ship(st: State, pol: Policy, what: str, now_ms: int):
    r = st.ship.get(what)
    return None if r is None or now_ms - r.heard_ms > pol.ship_ttl_ms else r.value


def clearance_holds(st: State, pol: Policy, now_ms: int) -> tuple[bool, str]:
    """Every binding, every cycle (payload_control `recheck`): not withdrawn; the live conn lease is the
    one it was granted under; the motion gate's fence is its motion_epoch; the payload's own fence is
    its payload_epoch; the ship is inside valid_while. An unknown reading never satisfies a bound."""
    c = st.clearance
    if c is None:
        return False, "no clearance"
    if c.withdrawn:
        return False, "the clearance was withdrawn"
    conn = st.leases.get("conn")
    if conn is None or not conn.live(now_ms):
        return False, "no live conn lease"
    if conn.token != c.conn_lease_id:
        return False, "the conn has moved since the clearance"
    mf = st.motion_fence
    if mf is None or now_ms - mf[1] > pol.motion_fence_ttl_ms:
        return False, "the motion gate's fence is unknown"
    if mf[0] != c.motion_epoch:
        return False, "a new motion_epoch since the clearance"
    if c.payload_epoch != st.fence:
        return False, "a new payload_epoch since the clearance"
    v = c.valid_while
    if v.min_speed_knots is not None or v.max_speed_knots is not None:
        sp = _ship(st, pol, "speed", now_ms)
        if sp is None:
            return False, "the ship's speed is unknown"
        if v.min_speed_knots is not None and sp < v.min_speed_knots:
            return False, "below the clearance's speed band"
        if v.max_speed_knots is not None and sp > v.max_speed_knots:
            return False, "above the clearance's speed band"
    if v.max_heel_deg is not None:
        heel = _ship(st, pol, "heel", now_ms)
        if heel is None:
            return False, "the ship's heel is unknown"
        if abs(heel) > v.max_heel_deg:
            return False, "heeled beyond the clearance"
    if v.vehicle_modes:
        mode = _ship(st, pol, "mode", now_ms)
        if mode is None or mode not in v.vehicle_modes:
            return False, "not in a mode the clearance allows"
    return True, ""


def _lease_ok(st: State, now_ms: int) -> tuple[bool, str]:
    lease = st.leases.get("payload")
    if lease is None or not lease.live(now_ms):
        return False, "no live PAYLOAD_AUTHORITY lease"
    return True, ""


def _to_safe(st: State, reason: str) -> State:
    return replace(st, mapping=None, safe_reason=reason, samples={})


# ---------------------------------------------------------------------------- payload_control/v1


def set_mapping(st: State, pol: Policy, req: MappingRequest, now_ms: int) -> tuple[State, str, str, Mapping | None]:
    """`set_payload_mapping`: atomic. Returns (state, result, detail, mapping in force)."""
    if req.payload_id != pol.payload_id:
        return st, DENIED, f"this gate is payload {pol.payload_id!r}'s", st.mapping
    ok, why = _lease_ok(st, now_ms)
    if not ok:
        return st, DENIED, why, st.mapping
    lease = st.leases["payload"]
    if req.caller.authority_token != lease.token or req.caller.controller_id != lease.holder:
        return st, DENIED, "not the payload's PAYLOAD_AUTHORITY holder", st.mapping
    if req.caller.fence != st.fence:
        return st, DENIED, "a stale payload_epoch: read the mapping again", st.mapping
    if not req.axes:
        return _to_safe(st, "released by the holder"), ACCEPTED, "released", None
    keys = [a.key for a in req.axes]
    unknown = sorted(set(keys) - set(pol.axis_keys))
    if unknown or len(set(keys)) != len(keys):
        return st, DENIED, f"not an axis of this payload: {', '.join(unknown) or 'a duplicate'}", st.mapping
    allowed, why = actuation_allowed(st, pol, now_ms)
    if not allowed:
        return st, TEMPORARILY_REJECTED, why, st.mapping
    clearance_id = ""
    if pol.needs_clearance:
        holds, why = clearance_holds(st, pol, now_ms)
        if not holds:
            return st, DENIED, f"this payload needs a clearance that holds: {why}", st.mapping
        clearance_id = st.clearance.clearance_id
    # The dead-man cannot be disabled: 0 or absent is the responder's own maximum, and nothing exceeds it.
    asked = int(req.max_axis_age_s * 1000) if req.max_axis_age_s and req.max_axis_age_s > 0 else pol.max_axis_age_ms
    m = Mapping(tuple(req.axes), lease.holder, lease.token, min(asked, pol.max_axis_age_ms), req.min_interval_s, clearance_id, now_ms)
    return replace(st, mapping=m, safe_reason="", samples={}), ACCEPTED, "mapped", m


# ---------------------------------------------------------------------------- the cycle


def tick(st: State, now_ms: int) -> tuple[State, str]:
    """The payload lease's own watchdog: a lapsed holder ends its authority -- the gate increments its
    generation ALONE (persist it) and goes to the safe state. Returns why it fenced, or ""."""
    lease = st.leases.get("payload")
    if lease is not None and not lease.live(now_ms):
        leases = dict(st.leases)
        leases.pop("payload")
        st = replace(st, leases=leases, generation=st.generation + 1, last_payload_token="")
        return _to_safe(st, "the PAYLOAD_AUTHORITY lease lapsed"), "the PAYLOAD_AUTHORITY lease lapsed"
    return st, ""


def step(st: State, pol: Policy, now_ms: int) -> tuple[State, Output]:
    """One cycle. Every test, in order; the first that fails puts the payload in its safe state."""
    m = st.mapping
    if m is None:
        return st, Output(None, pol.safe_state, st.safe_reason)
    ok, why = _lease_ok(st, now_ms)
    if ok and (st.leases["payload"].token != m.lease_token):
        ok, why = False, "the mapping's holder no longer holds the payload"
    if ok:
        ok, why = actuation_allowed(st, pol, now_ms)
    if ok and pol.needs_clearance:
        ok, why = clearance_holds(st, pol, now_ms)
        if ok and st.clearance.clearance_id != m.clearance_id:
            ok, why = False, "the clearance it was mapped under is no longer the one in force"
    values = {}
    waiting = None
    if ok:
        for a in m.axes:
            s = st.samples.get(a.key)
            if s is None:
                # A first sample gets the dead-man window from the mapping to arrive. Until then the
                # payload stays in its safe state -- an unheard axis is never a zero -- and after it,
                # the dead-man drops the mapping as for a stale one.
                if now_ms - m.mapped_ms <= m.max_axis_age_ms:
                    waiting = f"{a.key}: waiting for its first sample"
                    continue
                ok, why = False, f"{a.key}: no sample within the dead-man window"
                break
            if now_ms - s.recv_ms > m.max_axis_age_ms:
                ok, why = False, f"{a.key}: stale (the dead-man)"
                break
            v = -s.value_pct if a.invert else s.value_pct
            lo = 0.0 if a.unipolar else -100.0
            values[a.key] = max(lo, min(100.0, v))
    if not ok:
        st = _to_safe(st, why)
        return st, Output(None, pol.safe_state, why)
    if waiting:
        return st, Output(None, pol.safe_state, waiting)
    return st, Output(values, None, "")
