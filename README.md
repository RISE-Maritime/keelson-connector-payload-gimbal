# keelson-connector-payload-gimbal

A camera gimbal's own connector and **payload gate**. It is the first `keelson-connector-payload-*`.
It serves `payload_control/v1` for one payload, under that payload's PAYLOAD_AUTHORITY lease and its
own `payload_epoch`. It runs beside the motion gate, never through it, and a payload axis is never a
motion DOF.

- **Protocol:** keelson `protocols/payload_control.yaml`, `interfaces/PayloadControl.proto` and
  `PayloadClearance.proto` (keelson #360), with `CommandAuthority.proto`'s payload slot and
  `SafetyState.proto` (#332).
- **Design:** crowsnest-dev `docs/COMMAND-ARCHITECTURE.md` §1.2 and §2. The work item is
  crowsnest-dev #89.
- **keelson:** pinned exactly to `0.6.0.dev21` (experimental.21). #360 is still an open PR, and
  experimental.21 carries it.

> **Status: it gates a SIMULATED gimbal, and drives no hardware.**
> - Two drivers on the same seam (`Driver`): the in-process simulated gimbal (`--driver sim`),
>   and **Slipway's modelled one** (`--driver slipway`, crowsnest-dev#300, Slipway ADR-0147).
>   The Slipway driver publishes a raw, simulator-only drive record on
>   `raw_json/gimbal_drive/{payload_id}` and reads the actual angles back from her truth record.
>   `tests/test_slipway_e2e.py` runs the kernel from `../SLIPWAY` beside the gate, on loopback.
> - It writes no `motion_constraint`: a gimbal limits nothing about the hull. A towed sonar or a
>   crane would.
> - It publishes no gimbal telemetry: keelson has no gimbal subjects yet (keelson #372).
> - Never run this against a real vessel or a shared network.

## What it does

- **The four tests, every cycle,** on the gate's own clock, before a value reaches the driver:
  - **fresh:** every mapped axis has a sample within `max_axis_age_s`. This is the dead-man, and it
    can't be disabled: 0 or absent means the responder's own maximum, and nothing exceeds it. A
    first sample gets that window from the mapping to arrive, and an unheard axis is never a zero;
  - **leased:** the payload's PAYLOAD_AUTHORITY lease (slot `{vessel}/payload/{id}`) is live, and is
    the one the mapping was made under;
  - **fenced:** a mapping names the current `payload_epoch` (`caller.fence`);
  - **permitted:** the axes are this payload's (`<axis>/<id>`), and ACTUATION_ALLOWED holds:
    `safety_state` is CLEAR and the local lockout is not set. A silent or stale supervisor is not a
    CLEAR one.
- **The clearance**, for a payload that `affects_stability` or `affects_navigation`. A gimbal affects
  neither, so it needs none. A mapping is DENIED until a clearance holds, and every binding is
  re-checked every cycle:
  - the clearance is not withdrawn;
  - the live conn token is the clearance's `conn_lease_id`;
  - the motion gate's fence (from `resolved_control`) is its `motion_epoch`;
  - the payload's own fence is its `payload_epoch`;
  - the ship is inside `valid_while`. An unknown speed, heel or mode never satisfies a bound.
- **Never "idle".** On any failure the payload goes to its `payload_safe_state_policy` state, from
  `payloads[]`, and the mapping is dropped until the holder maps it again. For the simulated gimbal,
  HOLD_POSITION holds the angles it has now and INHIBIT freezes the drive. Neither drives to a neutral
  pose.
- **It owns `payload_epoch`.** The term is drawn when the store is (re)initialised. The generation
  changes when payload authority changes: a token it has not seen (a new holder, or the first after a
  restart), or the holder's lease lapsing, where it increments alone. It is persisted before anything
  is driven under it, and it never changes on a mapping, so a clearance outlives a re-map. A lost or
  corrupt store is a new term and the safe state.

## Running it (isolated rig only)

```bash
uv venv .venv && uv pip install -p .venv/bin/python -e ".[dev]"
.venv/bin/python bin/payload-gimbal2keelson.py -r rise -e <vessel> --payload-id gimbal \
    --platform config/platform.example.json --gate-config config/gate.example.json --state-dir state/
```

For Slipway's 60 m general cargo ship `sim-ted-bulk-alfa` and her PTZ gimbal (pan, tilt and zoom,
Slipway ADR-0159), the platform file is `config/sim-ted-bulk-alfa.platform.json`:

```bash
.venv/bin/python bin/payload-gimbal2keelson.py -r rise -e sim-ted-bulk-alfa -s payload-ptz --payload-id ptz \
    --driver slipway --platform config/sim-ted-bulk-alfa.platform.json --gate-config config/gate.example.json --state-dir state/
```

Its `payloads[]` entry must match her platform config's exactly. Crowsnest reads the axes and the safe
state from that one.

`--gate-config` carries the timings and, for a payload that needs a clearance, the keys of the
ship's speed, heel and mode (`"ship": {"speed": "<key expression>", …}`).

## Tests

```bash
.venv/bin/python -m pytest -q      # the model, the service and store, and the e2e CLI test
.venv/bin/ruff check .
```

The e2e test runs the CLI against an isolated loopback router, with scouting off. A crane fixture
exercises the clearance and every one of its bindings.
