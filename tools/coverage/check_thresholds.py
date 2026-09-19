#!/usr/bin/env python3
"""Coverage threshold gate for GolfSim.

Reads the llvm-cov JSON export (per-file summaries) and, when present beside it,
the LCOV export (per-line / per-branch records), applies the gating scope and
exclusion markers from thresholds.json, and fails if any metric is below its
threshold. Prints a per-file table sorted by the largest gap so the next test to
write is always at the top.

    check_thresholds.py <coverage.json> <thresholds.json> [--source-dir DIR] [--no-fail]

thresholds.json:
    {
      "gated_paths":   ["src/", "include/"],        # measured & gated (repo-relative prefixes)
      "ungated_paths": ["src/main.cpp"],            # measured, reported, not gated
      "exclusion_budget_percent": 2.0,              # max share of gated lines carrying LCOV_EXCL markers
      "lines": 60.9, "regions": 50.0, "branches": 44.5, "mcdc": 33.2
    }

Metrics:
    lines / branches  come from the LCOV export when available so that
                      LCOV_EXCL_LINE and LCOV_EXCL_START/STOP markers are honoured
                      (llvm-cov itself ignores those comments); otherwise from JSON.
    regions / mcdc    come from the JSON per-file summaries (markers not applied).

See docs/refactor/07_Test_Infrastructure_Plan.md §8.
"""
from __future__ import annotations

import json
import os
import re
import sys
from collections import defaultdict
from dataclasses import dataclass, field

METRICS = ("lines", "regions", "branches", "mcdc")


@dataclass
class FileCov:
    counts: dict = field(default_factory=lambda: {m: [0, 0] for m in METRICS})  # metric -> [covered, total]
    excluded_lines: int = 0

    def pct(self, m: str) -> float | None:
        cov, tot = self.counts[m]
        return 100.0 * cov / tot if tot else None


def rel(path: str, source_dir: str) -> str:
    path = os.path.normpath(path)
    src = os.path.normpath(source_dir)
    if path.startswith(src + os.sep):
        return path[len(src) + 1:].replace(os.sep, "/")
    return path.replace(os.sep, "/")


def in_scope(relpath: str, prefixes: list[str]) -> bool:
    return any(relpath == p or relpath.startswith(p) for p in prefixes)


def excluded_line_set(source_path: str) -> set[int]:
    """Lines covered by LCOV_EXCL_LINE / LCOV_EXCL_START..STOP markers (1-based)."""
    excluded: set[int] = set()
    try:
        with open(source_path, encoding="utf-8", errors="replace") as fh:
            in_block = False
            for n, line in enumerate(fh, start=1):
                if "LCOV_EXCL_START" in line:
                    in_block = True
                if in_block or "LCOV_EXCL_LINE" in line:
                    excluded.add(n)
                if "LCOV_EXCL_STOP" in line:
                    in_block = False
    except OSError:
        pass
    return excluded


def load_json_summaries(path: str, source_dir: str) -> dict[str, FileCov]:
    with open(path, encoding="utf-8") as fh:
        data = json.load(fh)
    files: dict[str, FileCov] = {}
    for export in data.get("data", []):
        for f in export.get("files", []):
            r = rel(f["filename"], source_dir)
            fc = files.setdefault(r, FileCov())
            s = f.get("summary", {})
            for m in METRICS:
                if m in s:
                    fc.counts[m] = [s[m].get("covered", 0), s[m].get("count", 0)]
    return files


def apply_lcov(lcov_path: str, source_dir: str, files: dict[str, FileCov]) -> bool:
    """Replace lines/branches counts with LCOV-derived values honouring exclusion markers."""
    if not os.path.exists(lcov_path):
        return False
    per_file_lines: dict[str, dict[int, int]] = defaultdict(dict)          # rel -> line -> count
    per_file_branches: dict[str, list[tuple[int, int]]] = defaultdict(list)  # rel -> [(line, count)]
    current = None
    with open(lcov_path, encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            line = raw.strip()
            if line.startswith("SF:"):
                current = rel(line[3:], source_dir)
            elif line.startswith("DA:") and current:
                ln, cnt = line[3:].split(",")[:2]
                per_file_lines[current][int(ln)] = max(per_file_lines[current].get(int(ln), 0), int(cnt))
            elif line.startswith("BRDA:") and current:
                parts = line[5:].split(",")
                ln = int(parts[0])
                taken = parts[3] if len(parts) > 3 else "-"
                per_file_branches[current].append((ln, 0 if taken == "-" else int(taken)))
            elif line == "end_of_record":
                current = None

    for r, fc in files.items():
        if r not in per_file_lines and r not in per_file_branches:
            continue
        excl = excluded_line_set(os.path.join(source_dir, r))
        lines = {ln: c for ln, c in per_file_lines.get(r, {}).items() if ln not in excl}
        fc.excluded_lines = sum(1 for ln in per_file_lines.get(r, {}) if ln in excl)
        fc.counts["lines"] = [sum(1 for c in lines.values() if c > 0), len(lines)]
        branches = [(ln, c) for ln, c in per_file_branches.get(r, []) if ln not in excl]
        fc.counts["branches"] = [sum(1 for _, c in branches if c > 0), len(branches)]
    return True


def main(argv: list[str]) -> int:
    if len(argv) < 3:
        print(__doc__)
        return 2
    cov_json, thresholds_path = argv[1], argv[2]
    source_dir = os.getcwd()
    fail = True
    i = 3
    while i < len(argv):
        if argv[i] == "--source-dir" and i + 1 < len(argv):
            source_dir = argv[i + 1]
            i += 2
        elif argv[i] == "--no-fail":
            fail = False
            i += 1
        else:
            print(f"Unknown argument: {argv[i]}")
            return 2

    with open(thresholds_path, encoding="utf-8") as fh:
        th = json.load(fh)
    gated = th.get("gated_paths", ["src/", "include/"])
    ungated = th.get("ungated_paths", [])
    budget = float(th.get("exclusion_budget_percent", 2.0))

    files = load_json_summaries(cov_json, source_dir)
    used_lcov = apply_lcov(os.path.join(os.path.dirname(os.path.abspath(cov_json)), "coverage.lcov"),
                           source_dir, files)

    gated_files = {r: fc for r, fc in files.items() if in_scope(r, gated) and not in_scope(r, ungated)}
    ungated_files = {r: fc for r, fc in files.items() if in_scope(r, ungated)}

    totals = {m: [0, 0] for m in METRICS}
    excluded_total = 0
    for fc in gated_files.values():
        for m in METRICS:
            totals[m][0] += fc.counts[m][0]
            totals[m][1] += fc.counts[m][1]
        excluded_total += fc.excluded_lines

    def pct(cov: int, tot: int) -> float | None:
        return 100.0 * cov / tot if tot else None

    def fmt(p: float | None) -> str:
        return f"{p:6.1f}%" if p is not None else "   n/a "

    print(f"\nCoverage gate  (lines/branches from {'LCOV export with exclusion markers' if used_lcov else 'JSON summaries'};"
          " regions/mcdc from JSON)")
    print(f"Gated scope: {gated}   Ungated: {ungated}\n")

    header = f"{'file':52s} {'lines':>8s} {'regions':>8s} {'branches':>9s} {'mcdc':>8s}"
    print(header)
    print("-" * len(header))

    def gap(fc: FileCov) -> float:
        # largest shortfall across metrics; files with no data sort last
        gaps = [100.0 - p for m in METRICS if (p := fc.pct(m)) is not None]
        return max(gaps) if gaps else -1.0

    for r, fc in sorted(gated_files.items(), key=lambda kv: -gap(kv[1])):
        print(f"{r:52s} {fmt(fc.pct('lines'))} {fmt(fc.pct('regions'))} {fmt(fc.pct('branches')):>9s} {fmt(fc.pct('mcdc'))}")
    for r, fc in sorted(ungated_files.items()):
        print(f"{r + '  (ungated)':52s} {fmt(fc.pct('lines'))} {fmt(fc.pct('regions'))} {fmt(fc.pct('branches')):>9s} {fmt(fc.pct('mcdc'))}")

    print("-" * len(header))
    print(f"{'TOTAL (gated)':52s} " + " ".join(
        f"{fmt(pct(*totals[m])):>{9 if m == 'branches' else 8}s}" for m in METRICS))
    print()

    failures: list[str] = []
    for m in METRICS:
        if m not in th:
            continue
        p = pct(*totals[m])
        if p is None:
            continue
        need = float(th[m])
        status = "OK " if p + 1e-9 >= need else "LOW"
        print(f"  {status}  {m:9s} {p:6.2f}%  (threshold {need:.2f}%)")
        if status == "LOW":
            failures.append(f"{m}: {p:.2f}% < {need:.2f}%")

    gated_lines_total = totals["lines"][1] + excluded_total
    excl_pct = 100.0 * excluded_total / gated_lines_total if gated_lines_total else 0.0
    status = "OK " if excl_pct <= budget else "OVER"
    print(f"  {status} exclusions {excluded_total} lines = {excl_pct:.2f}% of gated lines (budget {budget:.1f}%)")
    if excl_pct > budget:
        failures.append(f"exclusion budget exceeded: {excl_pct:.2f}% > {budget:.1f}%")

    if failures:
        print("\nCOVERAGE GATE FAILED:\n  " + "\n  ".join(failures))
        return 1 if fail else 0
    print("\nCOVERAGE GATE PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
