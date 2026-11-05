"""payload_control/v1 and the subjects the payload gate hears, as the model's records. The only module
that imports keelson's types, so model.py stays pure."""

from __future__ import annotations

from keelson.interfaces.PayloadControl_pb2 import PayloadMapping, SetPayloadMappingResponse
from keelson.interfaces.VehicleCommon_pb2 import CommandResult
from keelson.interfaces.VehicleControl_pb2 import ControlAxis
from keelson.payloads.Fence_pb2 import Fence as PbFence

from . import model
from .model import Axis, Caller, Clearance, Fence, MappingRequest, ValidWhile

RESULT = {
    model.ACCEPTED: CommandResult.COMMAND_RESULT_ACCEPTED,
    model.TEMPORARILY_REJECTED: CommandResult.COMMAND_RESULT_TEMPORARILY_REJECTED,
    model.DENIED: CommandResult.COMMAND_RESULT_DENIED,
}


def fence_to_proto(f: Fence) -> PbFence:
    return PbFence(term=f.term, generation=f.generation)


def fence_from_proto(p) -> Fence | None:
    return None if p is None or not p.term else Fence(p.term, int(p.generation))


def axis_from_proto(key: str, a: ControlAxis) -> Axis:
    return Axis(key, a.entity_id, a.subject, a.source_id, bool(a.unipolar), bool(a.invert))


def axis_to_proto(a: Axis) -> ControlAxis:
    return ControlAxis(entity_id=a.entity_id, subject=a.subject, source_id=a.source_id, unipolar=a.unipolar, invert=a.invert)


def mapping_request_from_proto(pb) -> MappingRequest:
    c = pb.caller
    caller = Caller(c.controller_id, c.authority_token, fence_from_proto(c.fence) if c.HasField("fence") else None)
    axes = tuple(axis_from_proto(k, pb.axes[k]) for k in sorted(pb.axes))
    return MappingRequest(pb.payload_id, caller, axes, float(pb.min_interval_s), float(pb.max_axis_age_s))


def set_mapping_response(result: str, detail: str, fence: Fence, max_axis_age_ms: int) -> SetPayloadMappingResponse:
    return SetPayloadMappingResponse(result=RESULT[result], detail=detail, payload_epoch=fence_to_proto(fence),
                                     max_axis_age_s=max_axis_age_ms / 1000.0)


def payload_mapping(payload_id: str, m: model.Mapping | None, fence: Fence) -> PayloadMapping:
    out = PayloadMapping(payload_id=payload_id, payload_epoch=fence_to_proto(fence))
    if m is not None:
        for a in m.axes:
            out.axes[a.key].CopyFrom(axis_to_proto(a))
        out.min_interval_s = m.min_interval_s
        out.max_axis_age_s = m.max_axis_age_ms / 1000.0
        out.clearance_id = m.clearance_id
    return out


def clearance_from_proto(pb) -> Clearance:
    v = pb.valid_while

    def opt(name):
        return getattr(v, name) if v.HasField(name) else None

    vw = ValidWhile(opt("min_speed_knots"), opt("max_speed_knots"), tuple(v.vehicle_modes), opt("max_heel_deg"))
    return Clearance(pb.clearance_id, pb.conn_lease_id, fence_from_proto(pb.motion_epoch) if pb.HasField("motion_epoch") else None,
                     fence_from_proto(pb.payload_epoch) if pb.HasField("payload_epoch") else None, vw, bool(pb.withdrawn), pb.granted_by)
