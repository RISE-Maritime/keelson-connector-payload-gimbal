"""The gate's sequence: boot, the payload_control/v1 calls, observations, samples, the cycle. No Zenoh
here; bin/ wires it.

ONE LOCK DECIDES: every call -- an RPC on a queryable thread, an observation from a subscriber, the
cycle -- takes the same lock, so the gate's state moves one decision at a time.

PERSIST BEFORE REPLY: a generation the gate moved (a new holder, a lapsed lease) is saved before the
next reply or the next drive under it.
"""

from __future__ import annotations

import logging
import threading
import time

from . import model
from .model import Fence, MappingRequest, Output, Policy

log = logging.getLogger(__name__)


def mono_ms() -> int:
    return int(time.monotonic() * 1000)


class PayloadGate:
    def __init__(self, *, store, policy: Policy, driver, now_ms=mono_ms, on_mapping_change=None):
        self.store = store
        self.policy = policy
        self.driver = driver
        self.now_ms = now_ms
        self.lock = threading.RLock()
        self.state: model.State | None = None
        self.events: list[str] = []
        self.last_output: Output | None = None
        self._last_cycle_ms: int | None = None
        # Called (outside the lock) with the axes now mapped, so bin/ can (re)subscribe their subjects.
        self.on_mapping_change = on_mapping_change or (lambda axes: None)

    def boot(self) -> model.State:
        with self.lock:
            st, why = self.store.load(self.policy.vessel_id, self.policy.payload_id)
            if why:
                log.warning("REINITIALISED with a new term (%s): every payload_epoch of the old term is dead", why)
                self.store.save(st, self.policy.payload_id)
                self.events.append(f"reinitialised: {why}")
            self.state = st
            log.info("payload %s: payload_epoch (%s…, %d), safe state %s", self.policy.payload_id, st.term[:8], st.generation,
                     self.policy.safe_state)
            return st

    # ------------------------------------------------------------------ payload_control/v1

    def set_mapping(self, req: MappingRequest) -> tuple[str, str, Fence, int]:
        with self.lock:
            st, result, detail, m = model.set_mapping(self.state, self.policy, req, self.now_ms())
            self.state = st
            axes = m.axes if m is not None else ()
            log.info("set_payload_mapping by %s: %s — %s", req.caller.controller_id, result, detail)
            out = (result, detail, st.fence, m.max_axis_age_ms if m is not None else self.policy.max_axis_age_ms)
        self.on_mapping_change(axes)
        return out

    def get_mapping(self) -> tuple[model.Mapping | None, Fence]:
        with self.lock:
            return self.state.mapping, self.state.fence

    # ------------------------------------------------------------------ what the gate hears

    def observe_lease(self, slot, holder, token, ttl_ms, released):
        with self.lock:
            had = self.state.mapping
            self.state, moved = model.observe_lease(self.state, slot, holder, token, ttl_ms, released, self.now_ms())
            if moved:
                self.store.save(self.state, self.policy.payload_id)
                self.events.append(f"new payload holder {holder}: generation {self.state.generation}")
                log.warning("NEW PAYLOAD HOLDER %s: payload_epoch generation %d", holder, self.state.generation)
            dropped = had is not None and self.state.mapping is None
        if dropped:
            self.on_mapping_change(())

    def observe_safety(self, state, local_lockout):
        with self.lock:
            self.state = model.observe_safety(self.state, state, local_lockout, self.now_ms())

    def observe_clearance(self, c):
        with self.lock:
            self.state = model.observe_clearance(self.state, c)

    def observe_motion_fence(self, fence):
        with self.lock:
            self.state = model.observe_motion_fence(self.state, fence, self.now_ms())

    def observe_ship(self, what, value):
        with self.lock:
            self.state = model.observe_ship(self.state, what, value, self.now_ms())

    def observe_axis(self, key, value_pct):
        with self.lock:
            self.state = model.observe_axis(self.state, key, value_pct, self.now_ms())

    # ------------------------------------------------------------------ the cycle

    def cycle(self) -> Output:
        """The lease watchdog, then the tests, then the driver: the values, or the safe state."""
        with self.lock:
            now = self.now_ms()
            dt_s = 0.0 if self._last_cycle_ms is None else (now - self._last_cycle_ms) / 1000.0
            self._last_cycle_ms = now
            had = self.state.mapping
            st, fenced = model.tick(self.state, now)
            if fenced:
                self.store.save(st, self.policy.payload_id)  # before anything is driven under the new fence
                self.events.append(f"fenced alone: {fenced}")
                log.warning("FENCED ALONE at generation %d: %s", st.generation, fenced)
            st, out = model.step(st, self.policy, now)
            self.state = st
            dropped = had is not None and st.mapping is None
            if out.values is not None:
                self.driver.apply(out.values, dt_s)
            else:
                self.driver.safe_state(out.safe_state, dt_s)
            if self.last_output is None or (self.last_output.safe_state, self.last_output.reason) != (out.safe_state, out.reason):
                log.info("payload %s: %s", self.policy.payload_id, "DRIVEN" if out.values is not None else f"{out.safe_state} — {out.reason}")
            self.last_output = out
        if dropped:
            self.on_mapping_change(())
        return out
