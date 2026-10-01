"""Candidate results.json vs baseline -> COMPARE.md (EKF_TEST_PLAN.md s3.7).

usage: python testing/suite/compare.py BASELINE/results.json CANDIDATE/results.json [--out COMPARE.md]
                                       [--fail-on-regression]

Per (mission, fault) and key metric: REGRESSION if its grade got worse, or it got more
than REL (20%) worse AND worse by more than the metric's absolute FLOOR; IMPROVED for
the mirror image; otherwise unchanged. NEES is scored by distance from 1 on a log scale
(|log10(nees)|), so 0.5 -> 2.0 is "unchanged" and 1 -> 5 is a regression. Recovery
"never" counts as infinitely bad. Default output: COMPARE.md next to the candidate.
"""
import argparse
import json
import math
import os
import sys

REL = 0.20
# metric -> absolute floor a change must exceed (in the metric's own units) to count
FLOOR = {"tilt_rms_deg": 0.05, "tilt_max_deg": 0.2, "yaw_rms_deg": 0.1, "pos_h_rms_m": 0.05,
         "vel_h_rms_ms": 0.02, "nees_att": 0.1, "nees_vel": 0.1, "nees_pos": 0.1,
         "tilt_drift_deg_s": 0.05, "recovery_s": 0.5, "gps_resets": 0.5, "gps_rejected": 2.5, "mag_rejected": 50}
KEYS = list(FLOOR)
RANK = {"PASS": 0, "WARN": 1, "FAIL": 2}


def badness(k, v):
    """map a metric value to 'bigger = worse' on the scale the floor applies to"""
    if v is None:
        return math.inf if k == "recovery_s" else None
    if k.startswith("nees"):
        return abs(math.log10(max(v, 1e-12)))
    if k == "tilt_drift_deg_s":
        return abs(v)
    return float(v)


def classify(k, b, c):
    bb, cb = badness(k, b), badness(k, c)
    if bb is None or cb is None or bb == cb:
        return "same", None
    if math.isinf(bb) or math.isinf(cb):
        return ("worse" if cb > bb else "better"), None
    diff = cb - bb
    rel = diff / max(abs(bb), 1e-12)
    if abs(diff) > FLOOR[k] and abs(rel) > REL:
        return ("worse" if diff > 0 else "better"), rel
    return "same", rel


def fmt(v):
    if v is None:
        return "never"
    if isinstance(v, float):
        return f"{v:.3g}"
    return str(v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("baseline")
    ap.add_argument("candidate")
    ap.add_argument("--out")
    ap.add_argument("--fail-on-regression", action="store_true")
    a = ap.parse_args()
    B, C = json.load(open(a.baseline)), json.load(open(a.candidate))
    out = a.out or os.path.join(os.path.dirname(os.path.abspath(a.candidate)), "COMPARE.md")

    warn = []
    for k in ("recordings", "thresholds", "suite_version", "truth_lag_s"):
        if B["meta"].get(k) != C["meta"].get(k):
            warn.append(f"meta `{k}` differs between baseline and candidate - comparison may not be like-for-like")
    bmap = {(r["mission"], r["fault_key"]): r for r in B["runs"]}
    cmap = {(r["mission"], r["fault_key"]): r for r in C["runs"]}
    only_b, only_c = sorted(set(bmap) - set(cmap)), sorted(set(cmap) - set(bmap))

    rows, regressions, improvements, grade_changes = [], [], [], []
    for key in [k for k in bmap if k in cmap]:
        b, c = bmap[key], cmap[key]
        flags = []
        for k in KEYS:
            bv, cv = b["metrics"].get(k), c["metrics"].get(k)
            if k not in b["metrics"] and k not in c["metrics"]:
                continue
            bg, cg = b["grades"].get(k), c["grades"].get(k)
            verdict, rel = classify(k, bv, cv)
            if bg and cg and RANK[cg] > RANK[bg]:
                verdict = "worse"
            elif bg and cg and RANK[cg] < RANK[bg]:
                verdict = "better"
            if verdict != "same" or bv != cv:
                flags.append((k, bv, cv, rel, verdict, bg, cg))
            if verdict == "worse":
                regressions.append((key, k, bv, cv, bg, cg))
            elif verdict == "better":
                improvements.append((key, k, bv, cv, bg, cg))
        if b["overall"] != c["overall"]:
            grade_changes.append((key, b["overall"], c["overall"]))
        rows.append((key, b["overall"], c["overall"], flags))

    L = ["# EKF suite comparison", "",
         f"baseline `{B['meta'].get('git')}`{' dirty' if B['meta'].get('dirty') else ''} "
         f"(filter {B['meta'].get('filter_sha1')}) vs candidate `{C['meta'].get('git')}`"
         f"{' dirty' if C['meta'].get('dirty') else ''} (filter {C['meta'].get('filter_sha1')})", "",
         f"**{len(improvements)} metric improvements, {len(regressions)} metric regressions, "
         f"{len(grade_changes)} overall-grade changes** across {len(rows)} matched runs.", ""]
    L += [f"> WARNING: {w}" for w in warn]
    if only_b or only_c:
        L.append(f"> runs only in baseline: {len(only_b)}, only in candidate: {len(only_c)}")
    L += ["", "## Regressions", ""]
    L += [f"- **{m} / {f}** `{k}`: {fmt(bv)} -> {fmt(cv)} ({bg} -> {cg})" for (m, f), k, bv, cv, bg, cg in regressions] or ["none"]
    L += ["", "## Overall grade changes", ""]
    L += [f"- {m} / {f}: {bo} -> {co}" for (m, f), bo, co in grade_changes] or ["none"]
    L += ["", "## Improvements", ""]
    L += [f"- {m} / {f} `{k}`: {fmt(bv)} -> {fmt(cv)} ({bg} -> {cg})" for (m, f), k, bv, cv, bg, cg in improvements] or ["none"]
    L += ["", "## All matched runs (metrics that changed at all)", "", "| mission | fault | overall | changes |", "|---|---|---|---|"]
    for (m, f), bo, co, flags in rows:
        ch = "; ".join(f"{k} {fmt(bv)}->{fmt(cv)}{' **'+v.upper()+'**' if v != 'same' else ''}" for k, bv, cv, _, v, _, _ in flags)
        L.append(f"| {m} | {f} | {bo}{' -> ' + co if co != bo else ''} | {ch or '-'} |")
    with open(out, "w", encoding="utf-8") as fh:
        fh.write("\n".join(L) + "\n")
    print(f"{len(improvements)} improved, {len(regressions)} regressed, {len(grade_changes)} grade changes -> {out}")
    for (m, f), k, bv, cv, bg, cg in regressions:
        print(f"  REGRESSION {m}/{f} {k}: {fmt(bv)} -> {fmt(cv)} ({bg} -> {cg})")
    if a.fail_on_regression and regressions:
        sys.exit(1)


if __name__ == "__main__":
    main()
