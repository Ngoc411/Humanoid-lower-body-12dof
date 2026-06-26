# Copyright (c) 2025-2026, The RoboLab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause
"""Output helpers: summary JSON, long-format Parquet time-series, manifests."""

from __future__ import annotations

import json
import os

import numpy as np


def _json_default(o):
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    raise TypeError(f"Not JSON serializable: {type(o)}")


def save_json(path: str, data: dict) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, default=_json_default, allow_nan=True)
    print(f"[eval] wrote {path}")


def save_manifest(path: str, mapping: dict) -> None:
    """env_id -> condition descriptor."""
    save_json(path, mapping)


def write_timeseries_parquet(
    path: str,
    series: dict[str, np.ndarray],
    env_ids: list[int],
    condition_ids: list[str],
    fps: float,
) -> None:
    """Write representative-env time-series in long format.

    series: dict colname -> array shaped (E, T) where E == len(env_ids).
    Produces one row per (env, step) with step/t/env_id/condition_id + columns.
    Falls back to CSV if pandas/pyarrow are unavailable.
    """
    os.makedirs(os.path.dirname(path), exist_ok=True)
    if not series:
        print(f"[eval] no time-series to write for {path}")
        return
    any_col = next(iter(series.values()))
    E, T = any_col.shape
    assert E == len(env_ids) == len(condition_ids), "env/condition length mismatch"

    base = {
        "step": np.tile(np.arange(T), E),
        "t": np.tile(np.arange(T) / fps, E),
        "env_id": np.repeat(np.asarray(env_ids), T),
        "condition_id": np.repeat(np.asarray(condition_ids, dtype=object), T),
    }
    for name, arr in series.items():
        base[name] = np.asarray(arr).reshape(E * T)

    try:
        import pandas as pd  # noqa: PLC0415
        df = pd.DataFrame(base)
        try:
            df.to_parquet(path, index=False)
            print(f"[eval] wrote {path}  ({len(df)} rows, {len(series)} signals)")
        except Exception as exc:  # pyarrow/fastparquet missing
            csv = os.path.splitext(path)[0] + ".csv"
            df.to_csv(csv, index=False)
            print(f"[eval] parquet failed ({exc}); wrote {csv} instead")
    except ImportError:
        csv = os.path.splitext(path)[0] + ".csv"
        _write_csv_fallback(csv, base)
        print(f"[eval] pandas missing; wrote {csv}")


def _write_csv_fallback(path: str, columns: dict[str, np.ndarray]) -> None:
    keys = list(columns.keys())
    n = len(columns[keys[0]])
    with open(path, "w", encoding="utf-8") as f:
        f.write(",".join(keys) + "\n")
        for i in range(n):
            f.write(",".join(str(columns[k][i]) for k in keys) + "\n")


def stack_series(series: dict[str, list]) -> dict[str, np.ndarray]:
    """Convert {col: [ (T,) per env ]} collected by phases into {col: (E, T)}."""
    return {k: np.stack(v) for k, v in series.items() if v}


def table_to_rows(table: list[dict]) -> list[dict]:
    """Round floats in a list-of-dicts table for compact, stable JSON."""
    out = []
    for row in table:
        out.append({k: (round(v, 5) if isinstance(v, float) else v) for k, v in row.items()})
    return out
