# CLAUDE.md

A camera gimbal's connector and **payload gate** (README). **Safety-critical: every change is reviewed
by a human before it is merged or run anywhere but the isolated tests.** Every commit message carries a
`Safety-review: required` trailer naming what changed.

## Rules that must not be broken

- **A payload axis is never a motion DOF**, and nothing here passes through, or writes to, the motion
  gate. The gate serves `payload_control/v1` on its own session.
- **Only this gate mints `payload_epoch`**, both halves: the term when the store is (re)initialised,
  the generation when payload authority changes (a new holder's token, or the holder's lease lapsing).
  Never on a mapping: a clearance names the payload_epoch and must outlive a re-map.
- **Every test holds, every cycle,** before a value reaches the driver: fresh (the dead-man, never
  disableable: 0 or absent is the responder's own maximum) ∧ leased ∧ fenced ∧ permitted
  (ACTUATION_ALLOWED: CLEAR ∧ ¬local_lockout), and, for a payload that affects stability or navigation,
  a clearance whose EVERY binding still holds (conn token, motion_epoch, payload_epoch, valid_while).
- **Never "idle".** On any failure the payload goes to its `payload_safe_state_policy` state, and the
  mapping is dropped until the holder maps it again. An unheard axis is never a zero.
- **Never less inhibited by accident:** a silent supervisor is not a CLEAR one; an unknown ship reading
  never satisfies a clearance bound; a lost store is a new term and the safe state.
- **Pure decisions.** `payload_gate/model.py` has no clock, no store and no Zenoh.
- **One lock decides** (service.PayloadGate): every RPC, observation and cycle.
- **Tests never touch a network** (the e2e test runs on an isolated loopback router), and nothing runs
  against a vessel or the shared Zenoh network.
- **keelson is an exact published pin.** Never a path or a local build.

## Commands

```bash
uv venv .venv && uv pip install -p .venv/bin/python -e ".[dev]"
.venv/bin/python -m pytest -q          # all, the e2e test included
.venv/bin/python -m pytest -m e2e      # the CLI on an isolated loopback router
.venv/bin/ruff check .
```
