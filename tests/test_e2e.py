"""The payload gate CLI as a real process on an ISOLATED loopback Zenoh router: multicast and gossip
scouting off everywhere, so nothing here can reach a shared network or a vessel."""

import json
import socket
import subprocess
import sys
import time
from pathlib import Path

import keelson
import pytest
import zenoh
from keelson.interfaces.PayloadControl_pb2 import GetPayloadMappingRequest, PayloadMapping, SetPayloadMappingRequest, SetPayloadMappingResponse
from keelson.interfaces.VehicleCommon_pb2 import CommandResult
from keelson.payloads.CommandAuthority_pb2 import CommandAuthority
from keelson.payloads.SafetyState_pb2 import SafetyState

pytestmark = pytest.mark.e2e

ROOT = Path(__file__).resolve().parent.parent
REALM, V, PID = "e2e", "testvessel", "gimbal"
BASE = f"{REALM}/@v0/{V}/pubsub"
SOURCE = "payload-gimbal"


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def isolated(conf: zenoh.Config, **kv):
    conf.insert_json5("scouting/multicast/enabled", "false")
    conf.insert_json5("scouting/gossip/enabled", "false")
    for k, v in kv.items():
        conf.insert_json5(k, json.dumps(v))
    return conf


class Harness:
    def __init__(self, tmp: Path):
        self.tmp = tmp
        self.port = free_port()
        self.session = zenoh.open(isolated(zenoh.Config(), mode="router", **{"listen/endpoints": [f"tcp/127.0.0.1:{self.port}"],
                                                                           "connect/endpoints": []}))
        self.zcfg = tmp / "zenoh.json5"
        self.zcfg.write_text(json.dumps({"mode": "client", "scouting": {"multicast": {"enabled": False}, "gossip": {"enabled": False}},
                                         "connect": {"endpoints": [f"tcp/127.0.0.1:{self.port}"]}, "listen": {"endpoints": []}}))
        self.proc = None

    def put(self, path, msg):
        self.session.put(f"{BASE}/{path}", keelson.enclose(msg.SerializeToString()))

    def start(self):
        platform = self.tmp / "platform.json"
        platform.write_text(json.dumps({"payloads": [{"id": PID, "kind": "camera_gimbal", "axes": [{"id": "pan"}, {"id": "tilt"}],
                                                      "payload_safe_state_policy": "HOLD_POSITION"}]}))
        self.proc = subprocess.Popen([sys.executable, str(ROOT / "bin/payload-gimbal2keelson.py"), "-r", REALM, "-e", V, "--payload-id", PID,
                                      "--platform", str(platform), "--state-dir", str(self.tmp / "state"), "--zenoh-config", str(self.zcfg)],
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        token = keelson.construct_source_liveliness_key(REALM, V, SOURCE)
        self.wait(lambda: any(True for _ in self.session.liveliness().get(token, timeout=0.5)), what="the gate's liveliness")

    def wait(self, pred, timeout=20.0, what="condition"):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.proc is not None and self.proc.poll() is not None:
                raise AssertionError(f"the gate exited:\n{self.proc.stdout.read()}")
            if pred():
                return
            time.sleep(0.1)
        raise AssertionError(f"timed out waiting for {what}")

    def call(self, procedure, request, reply_cls):
        key = keelson.construct_rpc_key(REALM, V, "payload_control", "v1", procedure, SOURCE)
        for reply in self.session.get(key, payload=request.SerializeToString(), timeout=10):
            assert reply.ok is not None, reply.err.payload.to_bytes()
            out = reply_cls()
            out.ParseFromString(reply.ok.payload.to_bytes())
            return out
        raise AssertionError(f"no answer to {procedure}")

    def mapping(self):
        return self.call("get_payload_mapping", GetPayloadMappingRequest(payload_id=PID), PayloadMapping)

    def close(self):
        if self.proc is not None:
            self.proc.terminate()
            self.proc.communicate(timeout=10)
        self.session.close()


@pytest.fixture
def h(tmp_path):
    harness = Harness(tmp_path)
    yield harness
    harness.close()


def _axis_sample(h, value):
    f = keelson.get_protobuf_message_class_from_type_name("keelson.TimestampedFloat")(value=value)
    h.session.put(f"{REALM}/@v0/roc-1/pubsub/joystick_x_pct/helm-1", keelson.enclose(f.SerializeToString()))


def test_the_gate_maps_a_gimbal_under_its_lease_and_drops_it_on_the_dead_man(h):
    h.start()
    m = h.mapping()
    assert m.payload_id == PID and not m.axes and len(m.payload_epoch.term) == 32
    # The payload's holder takes its lease; a new holder is a new payload_epoch.
    h.put(f"command_authority/{V}/payload/{PID}", CommandAuthority(vessel_id=V, controller_id="ted@ROC-1", token="tok-1",
                                                                     lease_ttl_seconds=30, payload_id=PID))
    h.wait(lambda: h.mapping().payload_epoch.generation == m.payload_epoch.generation + 1, what="the new payload_epoch")
    epoch = h.mapping().payload_epoch
    req = SetPayloadMappingRequest(payload_id=PID, max_axis_age_s=0.5)
    req.caller.controller_id, req.caller.authority_token = "ted@ROC-1", "tok-1"
    req.caller.fence.CopyFrom(epoch)
    a = req.axes["pan/gimbal"]
    a.entity_id, a.subject, a.source_id = "roc-1", "joystick_x_pct", "helm-1"
    # Not CLEAR yet: a silent supervisor is not a CLEAR one.
    r = h.call("set_payload_mapping", req, SetPayloadMappingResponse)
    assert r.result == CommandResult.COMMAND_RESULT_TEMPORARILY_REJECTED and "safety_state" in r.detail
    h.put(f"safety_state/{V}", SafetyState(vessel_id=V, state=SafetyState.State.SAFETY_STATE_CLEAR))
    time.sleep(0.5)
    r = h.call("set_payload_mapping", req, SetPayloadMappingResponse)
    assert r.result == CommandResult.COMMAND_RESULT_ACCEPTED, r.detail
    assert r.max_axis_age_s == pytest.approx(0.5)
    assert list(h.mapping().axes) == ["pan/gimbal"]
    # A stale caller fence is refused.
    req.caller.fence.generation -= 1
    assert h.call("set_payload_mapping", req, SetPayloadMappingResponse).result == CommandResult.COMMAND_RESULT_DENIED
    req.caller.fence.generation += 1
    # Samples keep coming: the mapping stays in force through several dead-man windows.
    t0 = time.time()
    while time.time() - t0 < 1.5:
        _axis_sample(h, 50.0)
        time.sleep(0.1)
    assert list(h.mapping().axes) == ["pan/gimbal"]
    # The samples stop: the dead-man drops the mapping, and the gimbal holds.
    h.wait(lambda: not h.mapping().axes, timeout=5, what="the dead-man to drop the mapping")
