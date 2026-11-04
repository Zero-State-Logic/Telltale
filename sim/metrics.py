"""Small, dependency-free evaluation metrics."""
from __future__ import annotations

import math

import numpy as np


def auc(pos: np.ndarray, neg: np.ndarray) -> float:
    """ROC AUC as the Mann-Whitney probability P(pos > neg), ties count 1/2."""
    pos = np.asarray(pos, dtype=np.float64)
    neg = np.asarray(neg, dtype=np.float64)
    if len(pos) == 0 or len(neg) == 0:
        return float("nan")
    allv = np.concatenate([pos, neg])
    order = np.argsort(allv, kind="mergesort")
    ranks = np.empty(len(allv))
    ranks[order] = np.arange(1, len(allv) + 1)
    # average ranks over ties
    sorted_v = allv[order]
    i = 0
    while i < len(sorted_v):
        j = i
        while j + 1 < len(sorted_v) and sorted_v[j + 1] == sorted_v[i]:
            j += 1
        if j > i:
            ranks[order[i:j + 1]] = 0.5 * (i + 1 + j + 1)
        i = j + 1
    r_pos = ranks[:len(pos)].sum()
    return float((r_pos - len(pos) * (len(pos) + 1) / 2.0) / (len(pos) * len(neg)))


def threshold_for_fa(scores: np.ndarray, max_fa: float) -> float:
    """Smallest threshold t such that the fraction of scores > t is <= max_fa."""
    s = np.sort(np.asarray(scores, dtype=np.float64))
    allowed = int(math.floor(max_fa * len(s)))
    idx = len(s) - 1 - allowed
    return float(s[max(idx, 0)])


def wilson(k: int, n: int, z: float = 1.96) -> tuple[float, float]:
    """95 % Wilson score interval for a binomial proportion."""
    if n == 0:
        return (float("nan"), float("nan"))
    p = k / n
    den = 1.0 + z * z / n
    centre = (p + z * z / (2 * n)) / den
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return (max(0.0, centre - half), min(1.0, centre + half))


def rate(k: int, n: int) -> dict:
    lo, hi = wilson(k, n)
    return {"k": int(k), "n": int(n), "rate": (k / n) if n else float("nan"),
            "ci95": [round(lo, 4), round(hi, 4)]}


def percentiles(x, qs=(50, 90)) -> dict:
    x = np.asarray(x, dtype=np.float64)
    if len(x) == 0:
        return {f"p{q}": None for q in qs}
    return {f"p{q}": round(float(np.percentile(x, q)), 1) for q in qs}
