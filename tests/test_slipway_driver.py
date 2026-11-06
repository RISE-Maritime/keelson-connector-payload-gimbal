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
