"""The payload gate's decisions (payload_gate/model.py) and its sequence (service.py), with no network.

Literal values throughout. The gimbal is the first target (it affects neither stability nor
navigation, so it needs no clearance); a crane fixture exercises the clearance and every binding."""

from dataclasses import replace

import pytest

from payload_gate import model
from payload_gate.driver.sim_gimbal import SimGimbal
from payload_gate.model import ACCEPTED, DENIED, TEMPORARILY_REJECTED, Axis, Caller, Clearance, Fence, MappingRequest, ValidWhile
from payload_gate.policy import parse
from payload_gate.service import PayloadGate
from payload_gate.store import Store

V = "sim-usv-alpha"
GIMBAL = {"payloads": [{"id": "gimbal", "kind": "camera_gimbal", "axes": [{"id": "pan"}, {"id": "tilt"}],
                        "payload_safe_state_policy": "HOLD_POSITION"}]}
CRANE = {"payloads": [{"id": "crane-1", "kind": "crane", "axes": [{"id": "slew"}, {"id": "hoist", "unipolar": True}],
                       "affects_stability": True, "payload_safe_state_policy": "HOLD_LOAD"}]}
PG = parse(GIMBAL, "gimbal", V)
PC = parse(CRANE, "crane-1", V)
HOLDER, TOKEN = "ted@ROC-1", "tok-payload-1"
CONN_TOKEN = "tok-conn-1"
MOTION = Fence("m" * 32, 7)


def axis(key, **kw):
    return Axis(key, "roc-1", "joystick_x_pct", "helm-1", **kw)


def ready(pol, *, now=0):
    """A state with a live payload lease, a CLEAR supervisor and the conn and motion fence heard."""
    st = model.State(vessel_id=V, term="a" * 32)
    st, _ = model.observe_lease(st, "payload", HOLDER, TOKEN, 5000, False, now)
    st, _ = model.observe_lease(st, "conn", "anna@ROC-1", CONN_TOKEN, 2000, False, now)
    st = model.observe_safety(st, model.SAFETY_CLEAR, False, now)
    st = model.observe_motion_fence(st, MOTION, now)
    return st


def req(st, pol, axes=None, **kw):
    axes = axes if axes is not None else [axis(k) for k in pol.axis_keys]
    return MappingRequest(pol.payload_id, Caller(HOLDER, TOKEN, st.fence), tuple(axes), **kw)


def mapped(pol, st=None, now=0):
    st = st or ready(pol, now=now)
    st, result, detail, m = model.set_mapping(st, pol, req(st, pol), now)
    assert result == ACCEPTED, detail
    for a in m.axes:
        st = model.observe_axis(st, a.key, 40.0, now)
    return st


def clearance(st, **kw):
    base = dict(clearance_id="clr-1", conn_lease_id=CONN_TOKEN, motion_epoch=MOTION, payload_epoch=st.fence,
                valid_while=ValidWhile(max_speed_knots=3.0, max_heel_deg=5.0), withdrawn=False, granted_by="anna@ROC-1")
    base.update(kw)
    return Clearance(**base)


def with_ship(st, now=0, speed=1.0, heel=1.0, mode=None):
    st = model.observe_ship(st, "speed", speed, now)
    st = model.observe_ship(st, "heel", heel, now)
    if mode is not None:
        st = model.observe_ship(st, "mode", mode, now)
    return st


# ---------------------------------------------------------------------------- policy


def test_the_policy_reads_payloads_and_refuses_what_it_cannot_honour():
    assert (PG.kind, PG.safe_state, PG.axis_keys, PG.needs_clearance) == ("camera_gimbal", "HOLD_POSITION", ("pan/gimbal", "tilt/gimbal"), False)
    assert PC.needs_clearance and PC.safe_state == "HOLD_LOAD"
    with pytest.raises(ValueError):
        parse(GIMBAL, "winch-9", V)
    for bad in ({"payload_safe_state_policy": "IDLE"}, {"payload_safe_state_policy": None}, {"kind": "turret"}, {"axes": []}):
        entry = {**GIMBAL["payloads"][0], **bad}
        with pytest.raises(ValueError):
            parse({"payloads": [entry]}, "gimbal", V)


# ---------------------------------------------------------------------------- set_payload_mapping


def test_a_gimbal_maps_with_no_clearance_and_drives():
    st = mapped(PG)
    st, out = model.step(st, PG, 100)
    assert out.values == {"pan/gimbal": 40.0, "tilt/gimbal": 40.0} and out.safe_state is None


@pytest.mark.parametrize("change,why", [
    (lambda st, r: (st, replace(r, payload_id="crane-1")), "this gate is payload"),
    (lambda st, r: (st, replace(r, caller=replace(r.caller, authority_token="other"))), "not the payload's PAYLOAD_AUTHORITY holder"),
    (lambda st, r: (st, replace(r, caller=replace(r.caller, controller_id="mallory@ROC-9"))), "not the payload's PAYLOAD_AUTHORITY holder"),
    (lambda st, r: (st, replace(r, caller=replace(r.caller, fence=Fence(st.term, st.generation - 1)))), "a stale payload_epoch"),
    (lambda st, r: (st, replace(r, caller=replace(r.caller, fence=None))), "a stale payload_epoch"),
    (lambda st, r: (st, replace(r, axes=(axis("zoom/gimbal"),))), "not an axis of this payload"),
    (lambda st, r: (st, replace(r, axes=(axis("pan/gimbal"), axis("pan/gimbal")))), "not an axis of this payload"),
    (lambda st, r: (model.State(vessel_id=V, term="a" * 32), r), "no live PAYLOAD_AUTHORITY lease"),
])
def test_each_map_time_test_refuses_alone(change, why):
    st = ready(PG)
    st, r = change(st, req(st, PG))
    _, result, detail, m = model.set_mapping(st, PG, r, 0)
    assert result == DENIED and why in detail and m is None


def test_not_clear_is_temporarily_rejected_and_a_silent_supervisor_is_not_clear():
    st = ready(PG)
    for s in (model.observe_safety(st, 2, False, 0), model.observe_safety(st, model.SAFETY_CLEAR, True, 0),
              replace(st, safety=None), replace(st, safety=model.SafetySeen(model.SAFETY_CLEAR, False, -20000))):
        _, result, detail, _ = model.set_mapping(s, PG, req(s, PG), 0)
        assert result == TEMPORARILY_REJECTED, detail


def test_empty_axes_is_a_release_to_the_safe_state():
    st = mapped(PG)
    st, result, detail, m = model.set_mapping(st, PG, req(st, PG, axes=[]), 10)
    assert (result, detail, m, st.mapping) == (ACCEPTED, "released", None, None)
    _, out = model.step(st, PG, 20)
    assert (out.values, out.safe_state, out.reason) == (None, "HOLD_POSITION", "released by the holder")


def test_the_dead_man_cannot_be_disabled():
    st = ready(PG)
    for asked, got in ((0.0, 1000), (0.25, 250), (60.0, 1000)):
        _, _, _, m = model.set_mapping(st, PG, req(st, PG, max_axis_age_s=asked), 0)
        assert m.max_axis_age_ms == got


# ---------------------------------------------------------------------------- every cycle


def test_a_stale_axis_trips_the_dead_man_and_never_idles():
    st = mapped(PG)
    st, out = model.step(st, PG, 1000)
    assert out.values is not None
    st, out = model.step(st, PG, 1001)
    assert (out.values, out.safe_state) == (None, "HOLD_POSITION") and "stale (the dead-man)" in out.reason
    assert st.mapping is None  # mapped again before it moves again
    _, out = model.step(model.observe_axis(st, "pan/gimbal", 10.0, 1002), PG, 1002)
    assert out.values is None


def test_an_axis_never_heard_is_not_a_zero():
    st = ready(PG)
    st, _, _, _ = model.set_mapping(st, PG, req(st, PG), 0)
    # Inside the dead-man window from the mapping: held in the safe state, the mapping kept.
    st, out = model.step(st, PG, 1000)
    assert (out.values, out.safe_state) == (None, "HOLD_POSITION") and "waiting for its first sample" in out.reason
    assert st.mapping is not None
    # Its first sample arrives: driven.
    st2 = model.observe_axis(model.observe_axis(st, "pan/gimbal", 20.0, 1000), "tilt/gimbal", -20.0, 1000)
    _, out2 = model.step(st2, PG, 1000)
    assert out2.values == {"pan/gimbal": 20.0, "tilt/gimbal": -20.0}
    # Never heard past the window: the dead-man drops it.
    st, out = model.step(st, PG, 1001)
    assert out.values is None and "no sample within the dead-man window" in out.reason and st.mapping is None


def test_values_are_clamped_and_inverted_and_unipolar_floors_at_zero():
    st = ready(PC)
    st = model.observe_clearance(st, clearance(st))
    st = with_ship(st)
    axes = [axis("slew/crane-1", invert=True), axis("hoist/crane-1", unipolar=True)]
    st, result, detail, _ = model.set_mapping(st, PC, req(st, PC, axes=axes), 0)
    assert result == ACCEPTED, detail
    st = model.observe_axis(st, "slew/crane-1", 150.0, 0)
    st = model.observe_axis(st, "hoist/crane-1", -40.0, 0)
    _, out = model.step(st, PC, 10)
    assert out.values == {"slew/crane-1": -100.0, "hoist/crane-1": 0.0}


def test_a_lapsed_lease_fences_alone_and_goes_safe():
    st = mapped(PG)
    g = st.generation
    st, why = model.tick(st, 5001)
    assert why == "the PAYLOAD_AUTHORITY lease lapsed" and st.generation == g + 1 and st.mapping is None
    _, out = model.step(st, PG, 5001)
    assert out.safe_state == "HOLD_POSITION"


def test_a_new_holder_is_a_new_payload_epoch_and_drops_the_old_mapping():
    st = mapped(PG)
    g = st.generation
    st, moved = model.observe_lease(st, "payload", "bo@ROC-2", "tok-payload-2", 5000, False, 10)
    assert moved and st.generation == g + 1 and st.mapping is None
    # The same holder renewing the same token moves nothing.
    st2, moved2 = model.observe_lease(st, "payload", "bo@ROC-2", "tok-payload-2", 5000, False, 20)
    assert not moved2 and st2.generation == st.generation


def test_the_lockout_or_a_stop_mid_cycle_goes_safe():
    for observe in (lambda s: model.observe_safety(s, model.SAFETY_CLEAR, True, 50), lambda s: model.observe_safety(s, 2, False, 50)):
        st = observe(mapped(PG))
        _, out = model.step(st, PG, 60)
        assert out.values is None and out.safe_state == "HOLD_POSITION"


# ---------------------------------------------------------------------------- the clearance (a crane)


def test_a_crane_is_denied_without_a_clearance_and_maps_with_one():
    st = with_ship(ready(PC))
    _, result, detail, _ = model.set_mapping(st, PC, req(st, PC), 0)
    assert result == DENIED and "needs a clearance that holds: no clearance" in detail
    st = model.observe_clearance(st, clearance(st))
    st, result, detail, m = model.set_mapping(st, PC, req(st, PC), 0)
    assert result == ACCEPTED and m.clearance_id == "clr-1"


@pytest.mark.parametrize("breaks,why", [
    (lambda st: model.observe_clearance(st, replace(st.clearance, withdrawn=True)), "withdrawn"),
    (lambda st: model.observe_lease(st, "conn", "bo@ROC-2", "tok-conn-2", 2000, False, 10)[0], "the conn has moved"),
    (lambda st: model.observe_motion_fence(st, Fence("m" * 32, 8), 10), "a new motion_epoch"),
    (lambda st: model.observe_ship(st, "speed", 4.5, 10), "above the clearance's speed band"),
    (lambda st: model.observe_ship(st, "heel", -6.0, 10), "heeled beyond the clearance"),
    (lambda st: replace(st, ship={}), "the ship's speed is unknown"),
    (lambda st: replace(st, motion_fence=None), "the motion gate's fence is unknown"),
])
def test_every_binding_ends_the_clearance_and_the_crane_holds_its_load(breaks, why):
    st = with_ship(ready(PC))
    st = model.observe_clearance(st, clearance(st))
    st = mapped(PC, st)
    _, out = model.step(st, PC, 5)
    assert out.values is not None
    st = breaks(st)
    _, out = model.step(st, PC, 10)
    assert (out.values, out.safe_state) == (None, "HOLD_LOAD") and why in out.reason


def test_a_new_payload_holder_ends_the_clearance_too():
    st = with_ship(ready(PC))
    st = model.observe_clearance(st, clearance(st))
    st, _ = model.observe_lease(st, "payload", "bo@ROC-2", "tok-payload-2", 5000, False, 1)
    holds, why = model.clearance_holds(st, PC, 2)
    assert not holds and "a new payload_epoch" in why


def test_a_mode_bound_needs_the_mode_heard():
    st = with_ship(ready(PC))
    st = model.observe_clearance(st, clearance(st, valid_while=ValidWhile(vehicle_modes=("DP", "HOLD"))))
    assert model.clearance_holds(st, PC, 0) == (False, "not in a mode the clearance allows")
    assert model.clearance_holds(model.observe_ship(st, "mode", "DP", 0), PC, 0) == (True, "")


def test_a_gimbal_runs_the_four_tests_without_a_clearance():
    st = mapped(PG)
    assert st.clearance is None
    _, out = model.step(st, PG, 10)
    assert out.values is not None


# ---------------------------------------------------------------------------- the service and the store


class Clock:
    def __init__(self):
        self.t = 0

    def __call__(self):
        return self.t


def test_the_service_persists_a_moved_generation_and_survives_a_restart(tmp_path):
    clock = Clock()
    g = PayloadGate(store=Store(tmp_path / "g.sqlite"), policy=PG, driver=SimGimbal("gimbal"), now_ms=clock)
    st0 = g.boot()
    g.observe_lease("payload", HOLDER, TOKEN, 5000, False)
    g.observe_safety(model.SAFETY_CLEAR, False)
    assert g.state.generation == st0.generation + 1
    g.store.close()
    g2 = PayloadGate(store=Store(tmp_path / "g.sqlite"), policy=PG, driver=SimGimbal("gimbal"), now_ms=clock)
    st = g2.boot()
    assert (st.term, st.generation) == (st0.term, st0.generation + 1) and st.mapping is None


def test_the_service_drives_the_gimbal_and_holds_it_on_the_dead_man(tmp_path):
    clock = Clock()
    gimbal = SimGimbal("gimbal")
    changes = []
    g = PayloadGate(store=Store(tmp_path / "g.sqlite"), policy=PG, driver=gimbal, now_ms=clock, on_mapping_change=changes.append)
    g.boot()
    g.observe_lease("payload", HOLDER, TOKEN, 60000, False)
    g.observe_safety(model.SAFETY_CLEAR, False)
    result, detail, fence, age = g.set_mapping(req(g.state, PG))
    assert result == ACCEPTED and age == 1000 and [a.key for a in changes[-1]] == ["pan/gimbal", "tilt/gimbal"]
    for t in range(0, 1100, 100):
        clock.t = t
        g.observe_axis("pan/gimbal", 100.0)
        g.observe_axis("tilt/gimbal", 0.0)
        g.observe_safety(model.SAFETY_CLEAR, False)
        g.cycle()
    assert gimbal.mode == "DRIVEN" and gimbal.angle["pan"] == pytest.approx(60.0)  # 1 s at 60 deg/s toward +170
    held = gimbal.angle["pan"]
    clock.t = 2500  # no axis sample for 1.4 s
    g.observe_safety(model.SAFETY_CLEAR, False)
    out = g.cycle()
    assert out.safe_state == "HOLD_POSITION" and "dead-man" in out.reason and changes[-1] == ()
    clock.t = 3500
    g.cycle()
    assert gimbal.angle["pan"] == held and gimbal.mode == "HOLD_POSITION"  # held, not driven to a neutral pose


def test_a_corrupt_store_is_a_new_term_in_the_safe_state(tmp_path):
    path = tmp_path / "g.sqlite"
    s = Store(path)
    st, _ = s.load(V, "gimbal")
    s.save(st, "gimbal")
    s.db.execute("UPDATE meta SET v=replace(v, '\"generation\":1', '\"generation\":9') WHERE k='state'")
    st2, why = s.load(V, "gimbal")
    assert why == "the stored state does not match its hash" and st2.term != st.term and st2.mapping is None


def test_a_mapping_is_never_driven_under_another_holders_lease():
    """Defence in depth: observe_lease already drops the mapping on a new holder, but a state in which
    the lease in force is not the one the mapping was made under is still refused every cycle."""
    st = mapped(PG)
    leases = dict(st.leases)
    leases["payload"] = model.Heard("bo@ROC-2", "tok-payload-2", 5000, 0)
    _, out = model.step(replace(st, leases=leases), PG, 10)
    assert out.values is None and out.reason == "the mapping's holder no longer holds the payload"
