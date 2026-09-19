"""
GolfSim IR Retroreflective-Dot Analyser
---------------------------------------
Scores camera frames for the "dots-only" imaging regime the launch monitor is
designed around: the background falls away to black and the only thing left is
a tight cluster of retroreflective glints on the ball.

Works on two kinds of input:
  --sweep  <dir>   an exposure/gain matrix from tools/ir_exposure_sweep.py
  --replay <dir>   a build/replays/stream_* or shot_* folder recorded by GolfSim.exe

For every frame it reports the background floor, the dot blobs found above
threshold, and whether those dots form a ball-sized cluster. Settings are then
ranked so the best exposure/gain pair can be read straight off the table.

Usage:
  python tools/analysis/analyze_ir_dots.py --sweep build/replays/sweep_20260919_120000
  python tools/analysis/analyze_ir_dots.py --replay build/replays/stream_20260811_201332_702
"""

import argparse
import glob
import json
import os
import re
import sys

import cv2
import numpy as np

# Geometry expectations, from PipelineTimingConfig in src/main.cpp:
#   workingDistanceMeters = 0.9144, nominalBallRadiusPx = 23.3
BALL_RADIUS_PX = 23.3
BALL_DIAMETER_PX = 2.0 * BALL_RADIUS_PX

# A retroreflective dot at this working distance is a few pixels across. Blobs
# outside this band are either sensor noise or a blown-out merged region.
MIN_DOT_AREA = 2.0
MAX_DOT_AREA = 150.0

# Dots belonging to one ball must sit inside roughly one ball diameter.
CLUSTER_RADIUS_PX = BALL_DIAMETER_PX * 0.75


def find_dots(gray, threshold):
    """Threshold and return every dot-sized bright blob as (x, y, area, peak)."""
    _, mask = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    dots, oversized = [], 0
    for contour in contours:
        area = cv2.contourArea(contour)
        # A 1-2 px contour has zero polygon area but is still a real glint.
        if area < 1.0:
            area = float(len(contour))
        if area > MAX_DOT_AREA:
            oversized += 1
            continue
        if area < MIN_DOT_AREA:
            continue
        moments = cv2.moments(contour)
        if moments["m00"] > 0.0:
            cx = moments["m10"] / moments["m00"]
            cy = moments["m01"] / moments["m00"]
        else:
            rect = cv2.boundingRect(contour)
            cx, cy = rect[0] + rect[2] / 2.0, rect[1] + rect[3] / 2.0
        rect = cv2.boundingRect(contour)
        peak = int(gray[rect[1]:rect[1] + rect[3], rect[0]:rect[0] + rect[2]].max())
        dots.append((cx, cy, float(area), peak))
    return dots, oversized, mask


def largest_cluster(dots, radius=CLUSTER_RADIUS_PX):
    """Greedy spatial clustering; returns the densest group of dots.

    Returns (members, centre_xy, spread_diameter_px).
    """
    if not dots:
        return [], (0.0, 0.0), 0.0

    points = np.array([[d[0], d[1]] for d in dots], dtype=np.float64)
    best_members = []
    for i in range(len(points)):
        distances = np.linalg.norm(points - points[i], axis=1)
        members = np.nonzero(distances <= radius)[0]
        if len(members) > len(best_members):
            best_members = members

    members = [dots[i] for i in best_members]
    coords = np.array([[d[0], d[1]] for d in members], dtype=np.float64)
    centre = coords.mean(axis=0)
    if len(members) > 1:
        spread = float(np.linalg.norm(coords - centre, axis=1).max() * 2.0)
    else:
        spread = 0.0
    return members, (float(centre[0]), float(centre[1])), spread


def analyse_frame(gray, threshold=None):
    """Full dots-only assessment of one grayscale frame."""
    p50 = float(np.percentile(gray, 50))
    p99 = float(np.percentile(gray, 99))
    p999 = float(np.percentile(gray, 99.9))
    peak = int(gray.max())
    saturated_pct = float(100.0 * np.count_nonzero(gray >= 250) / gray.size)

    # Default threshold sits between the background floor and the glint peaks.
    # Halfway in log-ish terms keeps it above noise without clipping dim dots.
    if threshold is None:
        threshold = int(max(p999 + 10, p50 + 0.45 * (peak - p50)))
        threshold = int(np.clip(threshold, 30, 250))

    dots, oversized, _ = find_dots(gray, threshold)
    members, centre, spread = largest_cluster(dots)
    bright_pct = float(100.0 * np.count_nonzero(gray > threshold) / gray.size)

    dot_areas = [d[2] for d in members]
    dot_peaks = [d[3] for d in members]

    return {
        "threshold": int(threshold),
        "background_p50": p50,
        "p99": p99,
        "p99_9": p999,
        "peak": peak,
        "saturated_pct": saturated_pct,
        "bright_pct": bright_pct,
        "dot_count_total": len(dots),
        "oversized_blobs": oversized,
        "cluster_dots": len(members),
        "cluster_centre": [round(centre[0], 1), round(centre[1], 1)],
        "cluster_spread_px": round(spread, 1),
        "cluster_dot_area_mean": round(float(np.mean(dot_areas)), 1) if dot_areas else 0.0,
        "cluster_dot_peak_mean": round(float(np.mean(dot_peaks)), 1) if dot_peaks else 0.0,
        "dots": members,
    }


def score(result):
    """Rank a frame on how cleanly it shows a dots-only ball.

    0 is useless, 100 is the ideal dots-only image. The weighting reflects what
    the detector needs: a dark field, several separated dots, and a cluster
    roughly one ball across.
    """
    if result["cluster_dots"] < 2:
        return 0.0

    # Dark background: nothing but the dots should survive the threshold.
    darkness = np.clip(1.0 - result["bright_pct"] / 0.5, 0.0, 1.0)

    # Dot count: 4-20 dots is the useful band for a centroid + spin solve.
    n = result["cluster_dots"]
    if n < 4:
        count_score = n / 4.0
    elif n <= 20:
        count_score = 1.0
    else:
        count_score = max(0.0, 1.0 - (n - 20) / 30.0)

    # Cluster spread should land near one ball diameter.
    spread_ratio = result["cluster_spread_px"] / BALL_DIAMETER_PX if BALL_DIAMETER_PX else 0.0
    spread_score = np.clip(1.0 - abs(spread_ratio - 0.8) / 0.8, 0.0, 1.0)

    # Contrast: dots should be far brighter than the floor, without blooming.
    contrast = (result["cluster_dot_peak_mean"] - result["background_p50"]) / 255.0
    contrast_score = float(np.clip(contrast / 0.55, 0.0, 1.0))
    if result["saturated_pct"] > 0.25:
        contrast_score *= 0.5  # blown out: dots merge and centroids drift

    return round(100.0 * (0.30 * darkness + 0.25 * count_score
                          + 0.20 * spread_score + 0.25 * contrast_score), 1)


def annotate(gray, result, caption):
    """Draw the detected dots and cluster onto a BGR copy for visual review."""
    canvas = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    for (cx, cy, _area, _peak) in result["dots"]:
        cv2.circle(canvas, (int(round(cx)), int(round(cy))), 6, (0, 255, 0), 1)
    if result["cluster_dots"] >= 2:
        cx, cy = result["cluster_centre"]
        cv2.circle(canvas, (int(cx), int(cy)), int(BALL_RADIUS_PX), (0, 220, 255), 2)
        cv2.drawMarker(canvas, (int(cx), int(cy)), (0, 220, 255), cv2.MARKER_CROSS, 14, 1)
    cv2.putText(canvas, caption, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    return canvas


def contact_sheet(tiles, columns=5, tile_width=320):
    """Stack labelled tiles into one grid image small enough to eyeball."""
    if not tiles:
        return None
    resized = []
    for image, label in tiles:
        scale = tile_width / image.shape[1]
        small = cv2.resize(image, (tile_width, max(1, int(image.shape[0] * scale))))
        cv2.rectangle(small, (0, small.shape[0] - 20), (tile_width, small.shape[0]), (0, 0, 0), -1)
        cv2.putText(small, label, (4, small.shape[0] - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
        resized.append(small)

    height = max(t.shape[0] for t in resized)
    rows = []
    for start in range(0, len(resized), columns):
        chunk = resized[start:start + columns]
        padded = []
        for tile in chunk:
            if tile.shape[0] != height:
                pad = np.zeros((height - tile.shape[0], tile.shape[1], 3), dtype=np.uint8)
                tile = np.vstack([tile, pad])
            padded.append(tile)
        while len(padded) < columns:
            padded.append(np.zeros((height, tile_width, 3), dtype=np.uint8))
        rows.append(np.hstack(padded))
    return np.vstack(rows)


def load_gray(path):
    return cv2.imread(path, cv2.IMREAD_GRAYSCALE)


def collect_sweep(sweep_dir):
    """Yield (label, side, exposure, gain, image_path) for a sweep folder."""
    manifest_path = os.path.join(sweep_dir, "manifest.json")
    entries = []
    if os.path.isfile(manifest_path):
        with open(manifest_path, encoding="utf-8") as fh:
            manifest = json.load(fh)
        for setting in manifest["settings"]:
            exposure = setting["exposure_requested"]
            gain = setting["gain_requested"]
            for side, names in setting["frames"].items():
                if not names:
                    continue
                path = os.path.join(sweep_dir, "raw", names[-1])
                entries.append(("e%d/g%d" % (exposure, gain), side, exposure, gain, path))
        return entries

    # No manifest: fall back to parsing the filenames.
    pattern = re.compile(r"(left|right)_e(-?\d+)_g(\d+)_(\d+)\.png$")
    for path in sorted(glob.glob(os.path.join(sweep_dir, "raw", "*.png"))):
        match = pattern.search(os.path.basename(path))
        if match:
            side, exposure, gain = match.group(1), int(match.group(2)), int(match.group(3))
            entries.append(("e%d/g%d" % (exposure, gain), side, exposure, gain, path))
    return entries


def collect_replay(replay_dir):
    """Yield (label, side, None, None, image_path) for a stream_/shot_ folder."""
    entries = []
    for side in ("left", "right"):
        for path in sorted(glob.glob(os.path.join(replay_dir, "raw", side + "_*.png"))):
            frame_id = os.path.splitext(os.path.basename(path))[0].split("_")[-1]
            entries.append(("frame %s" % frame_id, side, None, None, path))
    return entries


def main():
    parser = argparse.ArgumentParser(
        description="Score camera frames for retroreflective dots-only imaging.")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--sweep", help="Folder written by tools/ir_exposure_sweep.py")
    source.add_argument("--replay", help="build/replays/stream_* or shot_* folder")
    parser.add_argument("--thresh", type=int, default=None,
                        help="Fixed threshold; default adapts per frame")
    parser.add_argument("--top", type=int, default=10,
                        help="How many top-ranked settings to render (default: 10)")
    parser.add_argument("--out", default=None,
                        help="Report directory (default: <input>/analysis)")
    args = parser.parse_args()

    root = args.sweep or args.replay
    if not os.path.isdir(root):
        print("[Analyse] ERROR: not a directory: " + root)
        return 1

    entries = collect_sweep(root) if args.sweep else collect_replay(root)
    if not entries:
        print("[Analyse] ERROR: no frames found under " + os.path.join(root, "raw"))
        return 1

    out_dir = args.out or os.path.join(root, "analysis")
    os.makedirs(out_dir, exist_ok=True)

    print("[Analyse] Scoring %d frames from %s\n" % (len(entries), root))
    results = []
    for label, side, exposure, gain, path in entries:
        gray = load_gray(path)
        if gray is None:
            continue
        result = analyse_frame(gray, args.thresh)
        result["score"] = score(result)
        result.update({"label": label, "side": side, "exposure": exposure,
                       "gain": gain, "path": os.path.relpath(path, root)})
        results.append((result, gray))

    ranked = sorted(results, key=lambda r: r[0]["score"], reverse=True)

    header = ("%-12s %-6s %5s %6s %6s %5s %7s %7s %7s %6s"
              % ("setting", "side", "bg", "peak", "thr", "dots", "spread", "bright%", "sat%", "score"))
    print(header)
    print("-" * len(header))
    lines = [header, "-" * len(header)]
    for result, _ in ranked:
        line = ("%-12s %-6s %5.0f %6d %6d %5d %7.1f %7.3f %7.3f %6.1f"
                % (result["label"], result["side"], result["background_p50"], result["peak"],
                   result["threshold"], result["cluster_dots"], result["cluster_spread_px"],
                   result["bright_pct"], result["saturated_pct"], result["score"]))
        print(line)
        lines.append(line)

    # Render the top-ranked frames, plus contact sheets for whole-sweep review.
    tiles = []
    for result, gray in ranked[:args.top]:
        caption = "%s %s score %.0f dots %d" % (result["label"], result["side"],
                                                result["score"], result["cluster_dots"])
        canvas = annotate(gray, result, caption)
        name = "top_%s_%s.png" % (result["label"].replace("/", "_").replace(" ", ""), result["side"])
        cv2.imwrite(os.path.join(out_dir, name), canvas)
        tiles.append((canvas, caption))

    sheet = contact_sheet(tiles)
    if sheet is not None:
        cv2.imwrite(os.path.join(out_dir, "top_contact_sheet.png"), sheet)

    for side in ("left", "right"):
        side_tiles = []
        for result, gray in sorted([r for r in results if r[0]["side"] == side],
                                   key=lambda r: (r[0]["exposure"] or 0, r[0]["gain"] or 0)):
            label = "%s d%d s%.0f" % (result["label"], result["cluster_dots"], result["score"])
            side_tiles.append((cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR), label))
        sheet = contact_sheet(side_tiles, columns=5, tile_width=256)
        if sheet is not None:
            cv2.imwrite(os.path.join(out_dir, "sweep_%s.png" % side), sheet)

    # Stereo pairing per setting: when both cameras found a cluster, report the
    # disparity (xL - xR) and vertical offset (yL - yR) the trigger will see.
    # The disparity sign says whether the registered camera roles match the
    # physical layout; the vertical offset sizes the epipolar tolerance.
    by_label = {}
    for result, _ in results:
        by_label.setdefault(result["label"], {})[result["side"]] = result
    stereo_lines = []
    stereo_report = []
    for label, sides in by_label.items():
        left, right = sides.get("left"), sides.get("right")
        if not left or not right or left["cluster_dots"] < 2 or right["cluster_dots"] < 2:
            continue
        dx = left["cluster_centre"][0] - right["cluster_centre"][0]
        dy = left["cluster_centre"][1] - right["cluster_centre"][1]
        stereo_report.append({"label": label, "disparity_px": round(dx, 1), "vertical_offset_px": round(dy, 1)})
        stereo_lines.append("%-12s xL-xR %+7.1f px   yL-yR %+7.1f px%s"
                            % (label, dx, dy, "   <-- negative: cameras registered swapped?" if dx < 0 else ""))
    if stereo_lines:
        print("")
        print("Stereo pairs (settings where both cameras found a cluster):")
        for line in stereo_lines:
            print("  " + line)
        lines.append("")
        lines.append("Stereo pairs:")
        lines.extend("  " + line for line in stereo_lines)

    report = [r for r, _ in ranked]
    for entry in report:
        entry.pop("dots", None)  # centroid list is noise in the JSON report
    with open(os.path.join(out_dir, "report.json"), "w", encoding="utf-8") as fh:
        json.dump({"source": root, "results": report, "stereo": stereo_report}, fh, indent=2)
    with open(os.path.join(out_dir, "report.txt"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")

    print("\n[Analyse] Report and contact sheets written to " + out_dir)
    if ranked and ranked[0][0]["score"] > 0:
        best = ranked[0][0]
        print("[Analyse] Best setting: %s (%s) - %d dots, spread %.1f px, score %.1f"
              % (best["label"], best["side"], best["cluster_dots"],
                 best["cluster_spread_px"], best["score"]))
    else:
        print("[Analyse] No frame showed a usable dot cluster. "
              "Check that the IR emitter is on and the ball is in frame.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
