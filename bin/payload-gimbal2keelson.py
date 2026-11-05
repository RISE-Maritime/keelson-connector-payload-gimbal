#!/usr/bin/env python3
"""keelson-connector-payload-gimbal -- a camera gimbal's own connector and payload gate (crowsnest-dev
docs/COMMAND-ARCHITECTURE.md §1.2, keelson protocols/payload_control.yaml).

Its own process and its own Zenoh session, beside the motion gate and never through it. It serves
`payload_control/v1` for ONE payload, and it hears:
- `command_authority` -- the payload's PAYLOAD_AUTHORITY lease (slot `{vessel}/payload/{id}`) and the
  conn (slot `{vessel}`), stamped on local receipt;
- `payload_clearance/{vessel}/{id}` -- for a payload that needs one (a gimbal does not);
- `resolved_control/{vessel}` -- the motion gate's fence, the clearance's motion_epoch binding;
- `safety_state/{vessel}` -- ACTUATION_ALLOWED;
- the ship's speed / heel / mode, from the keys the gate config names, for a clearance's valid_while;
- the mapped axes' subjects, (re)subscribed whenever the mapping changes.

Every cycle it drives the payload with the mapped values, or puts it in its payload_safe_state_policy
state. This cut's driver is the in-process simulated gimbal (--driver sim); nothing here reaches
hardware.
"""

import argparse
import json
import logging
import pathlib
import sys
import threading
import time

import keelson
import zenoh
from keelson.interfaces.PayloadControl_pb2 import GetPayloadMappingRequest, SetPayloadMappingRequest
from keelson.payloads.CommandAuthority_pb2 import CommandAuthority
from keelson.payloads.SafetyState_pb2 import SafetyState
from keelson.scaffolding import add_common_arguments, create_zenoh_config, setup_logging
from keelson.scaffolding.liveliness import declare_liveliness
from keelson.scaffolding.rpc import RpcOp, serve_rpc
from keelson.scaffolding.signals import GracefulShutdown

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from payload_gate import codec
from payload_gate.driver.sim_gimbal import SimGimbal
from payload_gate.policy import load
from payload_gate.service import PayloadGate
from payload_gate.store import Store

log = logging.getLogger("payload-gate")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="payload-gimbal2keelson", description=__doc__.split("\n")[0])
    add_common_arguments(p)
    p.add_argument("-r", "--realm", required=True, help="Keelson realm, e.g. rise")
    p.add_argument("-e", "--entity-id", required=True, help="The vessel this payload is on (the vessel_id)")
    p.add_argument("-s", "--source-id", default="payload-gimbal", help="Source id and RPC responder id")
    p.add_argument("--payload-id", required=True, help="The payloads[] id this gate is for, e.g. gimbal")
    p.add_argument("--platform", type=pathlib.Path, required=True, help="The vessel's platform config (JSON), with payloads[]")
    p.add_argument("--gate-config", type=pathlib.Path, default=None, help="Timings and the ship's input keys (JSON)")
    p.add_argument("--state-dir", type=pathlib.Path, required=True, help="Directory of the gate's store (a persistent volume)")
    p.add_argument("--driver", choices=["sim"], default="sim", help="What the gate drives (only the simulated gimbal so far)")
    p.add_argument("--rate-hz", type=float, default=10.0, help="Cycle rate (default 10 Hz)")
    return p


def _decode(sample, cls):
    msg = cls()
    msg.ParseFromString(keelson.uncover(sample.payload.to_bytes())[2])
    return msg


def _value(sample, type_name):
    """The `value` of a keelson.Timestamped* sample. The type is NAMED, never guessed: protobuf does not
    refuse a wire-type mismatch, so a string read as a float decodes as 0.0 -- a fabricated reading."""
    msg = keelson.get_protobuf_message_class_from_type_name(type_name)()
    msg.ParseFromString(keelson.uncover(sample.payload.to_bytes())[2])
    return msg.value


SHIP_TYPES = {"speed": "keelson.TimestampedFloat", "heel": "keelson.TimestampedFloat", "mode": "keelson.TimestampedString"}


def main() -> int:
    args = build_parser().parse_args()
    setup_logging(level=args.log_level)
    zenoh.init_log_from_env_or(logging.getLevelName(args.log_level))
    realm, vessel, pid = args.realm, args.entity_id, args.payload_id
    base = f"{realm}/@v0/{vessel}/pubsub"
    gate_conf = json.loads(args.gate_config.read_text()) if args.gate_config else {}
    policy = load(args.platform, pid, vessel, args.gate_config)
    if not policy.require_safety_state:
        log.warning("require_safety_state is OFF: a rig with no supervisor. Never on a vessel.")

    conf = create_zenoh_config(args.mode, args.connect, args.listen, zenoh_config=args.zenoh_config)
    with zenoh.open(conf) as session:
        wanted = {"axes": None}  # set from RPC / cycle threads, applied by the main loop
        wanted_lock = threading.Lock()

        def on_mapping_change(axes):
            with wanted_lock:
                wanted["axes"] = tuple(axes)

        gate = PayloadGate(store=Store(args.state_dir / "payload-gate.sqlite"), policy=policy, driver=SimGimbal(pid),
                           on_mapping_change=on_mapping_change)
        gate.boot()

        def guarded(fn):
            def run(sample):
                try:
                    fn(sample)
                except Exception:  # pylint: disable=broad-except
                    log.exception("could not handle %s", sample.key_expr)
            return run

        def on_authority(sample):
            path = str(sample.key_expr).split("/command_authority/", 1)[1]
            rec = _decode(sample, CommandAuthority)
            ttl = int(rec.lease_ttl_seconds) * 1000
            if path == f"{vessel}/payload/{pid}":
                gate.observe_lease("payload", rec.controller_id, rec.token, ttl, rec.released)
            elif path == vessel:
                gate.observe_lease("conn", rec.controller_id, rec.token, ttl, rec.released)

        def on_clearance(sample):
            gate.observe_clearance(codec.clearance_from_proto(_decode(sample, keelson.get_protobuf_message_class_from_type_name(
                "keelson.PayloadClearance"))))

        def on_resolved(sample):
            rc = _decode(sample, keelson.get_protobuf_message_class_from_type_name("keelson.ResolvedControl"))
            gate.observe_motion_fence(codec.fence_from_proto(rc.motion_epoch) if rc.HasField("motion_epoch") else None)

        def on_safety(sample):
            s = _decode(sample, SafetyState)
            gate.observe_safety(int(s.state), bool(s.local_lockout))

        subs = [
            session.declare_subscriber(f"{base}/command_authority/**", guarded(on_authority)),
            session.declare_subscriber(f"{base}/payload_clearance/{vessel}/{pid}", guarded(on_clearance)),
            session.declare_subscriber(f"{base}/resolved_control/{vessel}", guarded(on_resolved)),
            session.declare_subscriber(f"{base}/safety_state/{vessel}", guarded(on_safety)),
        ]
        for what, key in (gate_conf.get("ship") or {}).items():
            if what not in SHIP_TYPES:
                raise SystemExit(f"gate config: unknown ship input {what!r} (speed, heel or mode)")
            subs.append(session.declare_subscriber(key, guarded(lambda s, w=what: gate.observe_ship(w, _value(s, SHIP_TYPES[w])))))

        def parse(cls, op):
            msg = cls()
            msg.ParseFromString(op.request_bytes)
            return msg

        def set_payload_mapping(op: RpcOp):
            result, detail, fence, age = gate.set_mapping(codec.mapping_request_from_proto(parse(SetPayloadMappingRequest, op)))
            op.reply_ok(codec.set_mapping_response(result, detail, fence, age))

        def get_payload_mapping(op: RpcOp):
            r = parse(GetPayloadMappingRequest, op)
            if r.payload_id and r.payload_id != pid:
                op.reply_err(f"this gate is payload {pid!r}'s, not {r.payload_id!r}'s")
                return
            m, fence = gate.get_mapping()
            op.reply_ok(codec.payload_mapping(pid, m, fence))

        serve_rpc(session, base_path=realm, entity_id=vessel, responder_id=args.source_id, interface="payload_control", version="v1",
                  handlers={"set_payload_mapping": set_payload_mapping, "get_payload_mapping": get_payload_mapping}, log=log)

        axis_subs = []
        period = 1.0 / args.rate_hz
        with declare_liveliness(session, realm, vessel, args.source_id), GracefulShutdown() as shutdown:
            while not shutdown.is_requested():
                t0 = time.monotonic()
                with wanted_lock:
                    axes, wanted["axes"] = wanted["axes"], None
                if axes is not None:
                    for s in axis_subs:
                        s.undeclare()
                    axis_subs = [session.declare_subscriber(f"{realm}/@v0/{a.entity_id}/pubsub/{a.subject}/{a.source_id}",
                                                            guarded(lambda s, k=a.key: gate.observe_axis(k, float(_value(s, "keelson.TimestampedFloat")))))
                                 for a in axes]
                try:
                    gate.cycle()
                except Exception:  # pylint: disable=broad-except
                    log.exception("cycle failed")
                time.sleep(max(0.0, period - (time.monotonic() - t0)))
        for s in subs + axis_subs:
            s.undeclare()
    gate.store.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
