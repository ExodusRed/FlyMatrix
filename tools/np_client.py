"""Minimal neuPrint Cypher client.

The male-cns:v1.0 dataset is served without authentication, so this needs
nothing but `requests`. Set NEUPRINT_TOKEN if that ever changes.
"""
from __future__ import annotations

import os
import time

import requests

SERVER = "https://neuprint.janelia.org"
DATASET = "male-cns:v1.0"

_session = requests.Session()
_token = os.environ.get("NEUPRINT_TOKEN")
if _token:
    _session.headers["Authorization"] = f"Bearer {_token}"


class QueryError(RuntimeError):
    pass


def cypher(query: str, dataset: str = DATASET, timeout: int = 600, retries: int = 4):
    """Run a Cypher query, returning the raw `data` rows.

    Retries on transport errors and 5xx with exponential backoff; neuPrint is a
    shared public service and occasionally sheds load.
    """
    payload = {"cypher": " ".join(query.split()), "dataset": dataset}
    delay = 3.0
    last = None
    for attempt in range(retries):
        try:
            r = _session.post(
                f"{SERVER}/api/custom/custom", json=payload, timeout=timeout
            )
            if r.status_code == 200:
                return r.json()["data"]
            if r.status_code in (429, 500, 502, 503, 504):
                last = f"HTTP {r.status_code}: {r.text[:200]}"
            else:
                raise QueryError(f"HTTP {r.status_code}: {r.text[:500]}")
        except requests.RequestException as exc:
            last = repr(exc)
        if attempt < retries - 1:
            time.sleep(delay)
            delay *= 2
    raise QueryError(f"query failed after {retries} attempts: {last}")


def chunks(seq, n):
    for i in range(0, len(seq), n):
        yield seq[i : i + n]
