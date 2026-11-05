"""The gate's own durable store: the term and the generation of `payload_epoch`, in one sqlite
database (stdlib, WAL, synchronous=FULL).

PERSIST BEFORE REPLY. A generation the gate moved -- a new holder, a lapsed lease -- is persisted
before anything is answered or driven under it, so the fence it exposes after a restart is the fence
it last minted.

A STORE THAT CANNOT BE TRUSTED IS A NEW TERM. Missing, corrupt, of an unknown schema or failing its
hash: a fresh random term, no mapping, the safe state. Every caller fence from before is then from a
term that no longer exists and is refused -- the safe direction for an enforcer, as the motion gate's.
"""

from __future__ import annotations

import hashlib
import json
import sqlite3
from pathlib import Path

from .model import State, fresh

SCHEMA_VERSION = 1


def _digest(s: str) -> str:
    return hashlib.sha256(f"v{SCHEMA_VERSION}:{s}".encode()).hexdigest()


class Store:
    def __init__(self, path: Path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        # The RPC handlers run on zenoh's queryable threads; every access is under service.PayloadGate's lock.
        self.db = sqlite3.connect(self.path, isolation_level=None, check_same_thread=False)
        self.db.execute("PRAGMA journal_mode=WAL")
        self.db.execute("PRAGMA synchronous=FULL")
        self.db.execute("CREATE TABLE IF NOT EXISTS meta (k TEXT PRIMARY KEY, v TEXT NOT NULL)")

    def close(self):
        self.db.close()

    def load(self, vessel_id: str, payload_id: str) -> tuple[State, str]:
        """The stored fence, or a FRESH state with the reason it is fresh ("" when restored)."""
        try:
            if self.db.execute("PRAGMA quick_check").fetchone()[0] != "ok":
                return fresh(vessel_id), "the store failed its integrity check"
            rows = dict(self.db.execute("SELECT k, v FROM meta").fetchall())
        except sqlite3.DatabaseError as e:
            return fresh(vessel_id), f"the store is unreadable: {e}"
        if "state" not in rows:
            return fresh(vessel_id), "no stored state"
        if rows.get("schema") != str(SCHEMA_VERSION):
            return fresh(vessel_id), f"unknown store schema {rows.get('schema')!r}"
        if _digest(rows["state"]) != rows.get("state_hash"):
            return fresh(vessel_id), "the stored state does not match its hash"
        try:
            d = json.loads(rows["state"])
            if d["vessel_id"] != vessel_id or d["payload_id"] != payload_id:
                return fresh(vessel_id), f"the store is for {d['vessel_id']!r}/{d['payload_id']!r}"
            st = State(vessel_id=vessel_id, term=d["term"], generation=int(d["generation"]))
        except (ValueError, KeyError, TypeError) as e:
            return fresh(vessel_id), f"the stored state is inconsistent: {e}"
        if len(st.term) != 32:
            return fresh(vessel_id), "the stored term is not 16 bytes of hex"
        return st, ""

    def save(self, st: State, payload_id: str) -> None:
        body = json.dumps({"vessel_id": st.vessel_id, "payload_id": payload_id, "term": st.term, "generation": st.generation},
                          sort_keys=True, separators=(",", ":"))
        self.db.execute("BEGIN IMMEDIATE")
        try:
            self.db.executemany("INSERT INTO meta (k, v) VALUES (?, ?) ON CONFLICT(k) DO UPDATE SET v=excluded.v",
                                [("schema", str(SCHEMA_VERSION)), ("state", body), ("state_hash", _digest(body))])
            self.db.execute("COMMIT")
        except BaseException:
            self.db.execute("ROLLBACK")
            raise
