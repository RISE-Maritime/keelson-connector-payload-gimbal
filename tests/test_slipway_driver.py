"""The Slipway driver (crowsnest-dev#300): the same Driver seam, a raw drive record out, the angles from
her truth record in -- never what was asked. No network."""

import json

from payload_gate.driver.slipway_gimbal import SlipwayGimbal


def make():
    sent = []
    return SlipwayGimbal("gimbal", lambda text: sent.append(json.loads(text))), sent


def test_a_mapped_axis_is_a_drive_record_for_this_payload_only():
    d, sent = make()
    d.apply({"pan/gimbal": 40, "tilt/gimbal": -10, "pan/other": 99}, 0.1)
    assert sent == [{"mode": "DRIVEN", "axes": {"pan": 40.0, "tilt": -10.0}}]


def test_a_safe_state_is_the_action_named_never_a_drive_to_zero():
    d, sent = make()
    d.safe_state("HOLD_POSITION", 0.1)
    assert sent == [{"mode": "HOLD_POSITION"}]


def test_the_angles_are_what_her_truth_record_says():
    d, _ = make()
    assert d.read() == {}  # nothing heard: no angle is invented
    d.on_truth({"entity_id": "usv", "gimbals": {"gimbal": {"mode": "DRIVEN", "pan_deg": 12.5, "tilt_deg": -3.0}}})
    assert d.read() == {"mode": "DRIVEN", "pan_deg": 12.5, "tilt_deg": -3.0}
    d.on_truth({"entity_id": "usv"})  # a record without gimbals changes nothing
    assert d.read()["pan_deg"] == 12.5


def test_her_ptz_config_drives_pan_tilt_and_zoom_with_no_clearance():
    """config/sim-ted-bulk-alfa.platform.json (Slipway ADR-0159): her `ptz` gimbal, whose third axis is a zoom.
    The driver needs no change for it: every mapped axis of this payload goes into the drive record."""
    import pathlib

    from payload_gate.policy import load

    pol = load(pathlib.Path(__file__).resolve().parent.parent / "config" / "sim-ted-bulk-alfa.platform.json", "ptz", "sim-ted-bulk-alfa")
    assert pol.axis_keys == ("pan/ptz", "tilt/ptz", "zoom/ptz")
    assert (pol.kind, pol.safe_state, pol.needs_clearance) == ("camera_gimbal", "HOLD_POSITION", False)
    sent = []
    d = SlipwayGimbal("ptz", lambda text: sent.append(json.loads(text)))
    d.apply({"pan/ptz": 50, "tilt/ptz": 20, "zoom/ptz": 100}, 0.1)
    assert sent == [{"mode": "DRIVEN", "axes": {"pan": 50.0, "tilt": 20.0, "zoom": 100.0}}]


def test_her_gate_config_runs_without_a_supervisor_and_nowhere_else():
    """config/gate.sim-ted-bulk-alfa.json (Ted, 2026-11-25): her CoSwim deployment has no safety supervisor,
    so this one file turns require_safety_state off. The example config keeps it on."""
    import pathlib

    from payload_gate.policy import load

    root = pathlib.Path(__file__).resolve().parent.parent / "config"
    pol = load(root / "sim-ted-bulk-alfa.platform.json", "ptz", "sim-ted-bulk-alfa", root / "gate.sim-ted-bulk-alfa.json")
    assert (pol.require_safety_state, pol.max_axis_age_ms) == (False, 1000)
    assert load(root / "platform.example.json", "gimbal", "x", root / "gate.example.json").require_safety_state is True
