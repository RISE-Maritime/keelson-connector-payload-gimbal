"""The payload gate driving SLIPWAY'S modelled gimbal (crowsnest-dev#300, Slipway ADR-0147): the kernel runs
from the checkout beside this one (../SLIPWAY, its own .venv) on the same ISOLATED loopback router as the
gate -- nothing reaches a shared network or a vessel.

A mapped axis moves her gimbal, as her truth record says; the dead-man holds it where it is; and her local
lockout holds it even while the helm keeps pushing."""

import json
import os
import subprocess
import threading
import time
from pathlib import Path

import keelson
import pytest
import yaml
from keelson.interfaces.PayloadControl_pb2 import SetPayloadMappingRequest, SetPayloadMappingResponse
from keelson.interfaces.VehicleCommon_pb2 import CommandResult
from keelson.payloads.CommandAuthority_pb2 import CommandAuthority
from keelson.payloads.Primitives_pb2 import TimestampedString
from keelson.payloads.SafetyState_pb2 import SafetyState

from test_e2e import BASE, PID, REALM, V, Harness, _axis_sample

pytestmark = pytest.mark.e2e

SLIPWAY = Path(os.environ.get("PAYLOAD_SLIPWAY", Path(__file__).resolve().parents[2] / "SLIPWAY"))


def scenario(tmp: Path) -> Path:
    doc = {"api_version": "slipway/v1", "kind": "Scenario", "metadata": {"id": "payload-gimbal-e2e"},
           "execution": {"realm": REALM, "seed": 1, "dt_ms": 100}, "world": {"origin": {"lat": 57.68, "lon": 11.83}},
           "vessels": [{"entity_id": V, "loa_m": 6.0, "beam_m": 2.4, "stw_knots": 0.0,
                        "initial": {"lat": 57.681, "lon": 11.825, "heading_deg": 90.0},
                        "payloads": [{"id": PID, "kind": "camera_gimbal", "axes": {
                            "pan": {"min_deg": -170, "max_deg": 170, "rate_dps": 60},
                            "tilt": {"min_deg": -90, "max_deg": 30, "rate_dps": 40}}}]}]}
    path = tmp / "scenario.yaml"
    path.write_text(yaml.safe_dump(doc))
    for d in ("vessels", "checkpoints"):
        (tmp / d).mkdir(exist_ok=True)
    return path


class Kernel:
    def __init__(self, h: Harness):
        self.h, self.gimbal, self.lock = h, {}, threading.Lock()
        self.sub = h.session.declare_subscriber(f"{BASE}/raw_json/truth", self._truth)
        self.proc = subprocess.Popen([str(SLIPWAY / ".venv/bin/python"), "-m", "slipway_kernel.main", "--mode", "client",
                                      "--connect", f"tcp/127.0.0.1:{h.port}", "--scenario", str(scenario(h.tmp)),
                                      "--vessel-dir", str(h.tmp / "vessels"), "--checkpoint-dir", str(h.tmp / "checkpoints"),
                                      "--auto-start", "--allow-edits", "--capability-timeout-s", "1"],
                                     cwd=SLIPWAY, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    def _truth(self, sample):
        m = TimestampedString()
        m.ParseFromString(keelson.uncover(sample.payload.to_bytes())[2])
        g = (json.loads(m.value).get("gimbals") or {}).get(PID)
        if g is not None:
            with self.lock:
                self.gimbal = g

    def now(self) -> dict:
        with self.lock:
            return dict(self.gimbal)

    def set_lockout(self, engaged: bool) -> None:
        ops = {"ops": [{"op": "stage_event", "event": {"action": "set_lockout", "entity_id": V, "engaged": engaged}}]}
        key = keelson.construct_rpc_key(REALM, "slipway", "scenario_edit", "v1", "commit", "sim")
        for reply in self.h.session.get(key, payload=json.dumps(ops).encode(), timeout=5):
            assert reply.ok is not None, reply.err.payload.to_bytes()
            return
        raise AssertionError("scenario_edit commit unanswered")

    def close(self):
        self.sub.undeclare()
        self.proc.terminate()
        try:
            self.proc.communicate(timeout=20)
        except subprocess.TimeoutExpired:
            self.proc.kill()


@pytest.fixture
def rig(tmp_path):
    if not (SLIPWAY / ".venv/bin/python").exists():
        pytest.skip(f"Slipway is not checked out (with its .venv) at {SLIPWAY}")
    h = Harness(tmp_path)
    k = Kernel(h)
    try:
        h.wait(lambda: k.now(), timeout=60, what="Slipway's truth record with her gimbal")
        yield h, k
    finally:
        k.close()
        h.close()


def _map(h):
    h.put(f"command_authority/{V}/payload/{PID}", CommandAuthority(vessel_id=V, controller_id="ted@ROC-1", token="tok-1",
                                                                     lease_ttl_seconds=60, payload_id=PID))
    h.put(f"safety_state/{V}", SafetyState(vessel_id=V, state=SafetyState.State.SAFETY_STATE_CLEAR))
    time.sleep(0.6)
    req = SetPayloadMappingRequest(payload_id=PID, max_axis_age_s=0.5)
    req.caller.controller_id, req.caller.authority_token = "ted@ROC-1", "tok-1"
    req.caller.fence.CopyFrom(h.mapping().payload_epoch)
    a = req.axes[f"pan/{PID}"]
    a.entity_id, a.subject, a.source_id = "roc-1", "joystick_x_pct", "helm-1"
    r = h.call("set_payload_mapping", req, SetPayloadMappingResponse)
    assert r.result == CommandResult.COMMAND_RESULT_ACCEPTED, r.detail


class Helm:
    def __init__(self, h, value):
        self.h, self.value, self.stop = h, value, threading.Event()
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        while not self.stop.wait(0.1):
            _axis_sample(self.h, self.value)


def test_the_gate_drives_her_gimbal_and_the_dead_man_holds_it_where_it_is(rig):
    h, k = rig
    assert k.now()["mode"] == "HOLD_POSITION" and k.now()["pan_deg"] == 0.0
    h.start("--driver", "slipway")
    _map(h)
    helm = Helm(h, 50.0)  # 50 % of -170..170 is 85 deg
    try:
        h.wait(lambda: k.now().get("mode") == "DRIVEN" and k.now()["pan_deg"] > 40, timeout=15, what="her gimbal slewing")
        h.wait(lambda: abs(k.now()["pan_deg"] - 85.0) < 0.5, timeout=15, what="her gimbal at 85 deg")
    finally:
        helm.stop.set()
    # The helm falls silent: the gate's dead-man puts it to its safe state, and Slipway holds where it is.
    h.wait(lambda: k.now().get("mode") == "HOLD_POSITION", timeout=10, what="the safe state")
    held = k.now()["pan_deg"]
    time.sleep(1.0)
    assert abs(k.now()["pan_deg"] - held) < 1e-6 and abs(held - 85.0) < 0.5


def test_her_local_lockout_holds_the_gimbal_while_the_helm_keeps_pushing(rig):
    h, k = rig
    h.start("--driver", "slipway")
    _map(h)
    helm = Helm(h, 30.0)
    try:
        h.wait(lambda: k.now().get("mode") == "DRIVEN" and k.now()["pan_deg"] > 5, timeout=15, what="her gimbal slewing")
        helm.value = -100.0  # a new order...
        k.set_lockout(True)  # ...and the engineer on board throws her lockout switch
        h.wait(lambda: k.now().get("mode") == "LOCKOUT", timeout=10, what="the lockout hold")
        held = k.now()["pan_deg"]
        time.sleep(1.0)
        assert abs(k.now()["pan_deg"] - held) < 1e-6  # the helm keeps pushing; nothing moves
    finally:
        helm.stop.set()
