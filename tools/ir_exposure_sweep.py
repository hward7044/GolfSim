"""
GolfSim IR Exposure / Gain Sweep Recorder
-----------------------------------------
Records the stereo pair across a matrix of exposure and gain settings so the
resulting frames can be analysed offline to find the settings where ONLY the
retroreflective dots on the ball are visible.

The camera controls driven here are the same UVC controls the C++
MediaFoundationDriver drives, so values found with this tool transfer directly
to the production pipeline.

Measured control ranges on the Arducam OV9281 (1280x800 YUY2):
    EXPOSURE    -13 .. -5   (UVC log2 seconds; above -5 the frame period clamps it)
    GAIN          0 .. 100
    BRIGHTNESS    0 .. 64   (additive black level -- keep at 0, it lifts the floor)

Usage:
  python tools/ir_exposure_sweep.py                      # full default sweep
  python tools/ir_exposure_sweep.py --strobe continuous  # drive the IR emitter DC-on first
  python tools/ir_exposure_sweep.py --exposures -11 -9 -7 --gains 0 50 100
"""

import argparse
import datetime
import json
import os
import subprocess
import sys

import cv2
import numpy as np

# Defaults chosen from the measured usable control range (see module docstring).
DEFAULT_EXPOSURES = [-13, -12, -11, -10, -9, -8, -7, -6, -5]
DEFAULT_GAINS = [0, 25, 50, 75, 100]

STROBE_CHARS = {
    "continuous": "1",   # IR permanently on -- easiest for static exposure tuning
    "strobe": "H",       # 300 Hz strobe ready mode
    "standby": "L",      # 10 Hz eye-safe standby
    "off": "0",
    "none": None,        # leave the controller alone
}


def send_strobe_command(com_port, char):
    """Send a single char to the Arduino strobe controller via PowerShell.

    Avoids a pyserial dependency; System.IO.Ports ships with .NET Framework.
    Returns True on success. Failure is never fatal -- the sweep still runs.
    """
    if char is None:
        return True
    ps = (
        "$p = New-Object System.IO.Ports.SerialPort "
        + '"' + com_port + '",115200,None,8,one; '
        + "$p.Open(); "
        + "$p.Write(" + '"' + char + '"' + "); "
        + "Start-Sleep -Milliseconds 150; "
        + "$p.Close()"
    )
    try:
        subprocess.run(
            ["powershell", "-NoProfile", "-NonInteractive", "-Command", ps],
            check=True, capture_output=True, timeout=15,
        )
        print("[Sweep] Strobe controller on " + com_port + ": sent '" + char + "'")
        return True
    except Exception as exc:  # noqa: BLE001 - diagnostic tool: report and continue
        print("[Sweep] WARNING: could not drive strobe on " + com_port + " (" + str(exc) + "). "
              "Set the emitter manually before recording.")
        return False


def open_camera(index, width, height):
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUY2"))
    # Manual exposure, and a zero black-level offset so the background floor
    # stays down where the dots can stand out against it.
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    cap.set(cv2.CAP_PROP_BRIGHTNESS, 0)
    ok, _ = cap.read()
    if not ok:
        cap.release()
        return None
    return cap


def autodetect_cameras(width, height):
    """Find the two OV9281s by the resolution they accept.

    The integrated laptop webcam reports 1280x720 and is skipped.
    """
    found = []
    for idx in range(6):
        cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap.release()
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        ok, _ = cap.read()
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        cap.release()
        if ok and w == width and h == height:
            found.append(idx)
        if len(found) == 2:
            break
    return found


def to_gray(frame):
    if frame is None:
        return None
    if frame.ndim == 3:
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return frame


def capture_setting(caps, exposure, gain, settle, frames):
    """Apply one (exposure, gain) pair to every camera and grab `frames` frames."""
    for cap in caps.values():
        cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
        cap.set(cv2.CAP_PROP_GAIN, gain)

    # Burn off in-flight frames still carrying the previous setting.
    for _ in range(settle):
        for cap in caps.values():
            cap.read()

    grabbed = {side: [] for side in caps}
    for _ in range(frames):
        for side, cap in caps.items():
            ok, frame = cap.read()
            if ok:
                grabbed[side].append(to_gray(frame))
    return grabbed


def main():
    parser = argparse.ArgumentParser(
        description="Record an exposure/gain sweep of the stereo IR pair for offline analysis.")
    parser.add_argument("--left-cam", type=int, default=None,
                        help="DSHOW device index for the Left camera (default: autodetect)")
    parser.add_argument("--right-cam", type=int, default=None,
                        help="DSHOW device index for the Right camera (default: autodetect)")
    parser.add_argument("--exposures", type=int, nargs="+", default=DEFAULT_EXPOSURES,
                        help="UVC log2-second exposure values (default: -13..-5)")
    parser.add_argument("--gains", type=int, nargs="+", default=DEFAULT_GAINS,
                        help="Gain values 0..100 (default: 0 25 50 75 100)")
    parser.add_argument("--frames", type=int, default=3,
                        help="Frames captured per setting (default: 3)")
    parser.add_argument("--settle", type=int, default=6,
                        help="Frames discarded after changing a setting (default: 6)")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=800)
    parser.add_argument("--strobe", choices=sorted(STROBE_CHARS), default="none",
                        help="Emitter mode to command before recording (default: none)")
    parser.add_argument("--com", default="COM3",
                        help="Strobe controller serial port (default: COM3)")
    parser.add_argument("--out", default=None,
                        help="Output directory (default: build/replays/sweep_<timestamp>)")
    parser.add_argument("--label", default="", help="Free-text note stored in the manifest")
    args = parser.parse_args()

    if args.strobe != "none":
        send_strobe_command(args.com, STROBE_CHARS[args.strobe])

    left_idx, right_idx = args.left_cam, args.right_cam
    if left_idx is None or right_idx is None:
        print("[Sweep] Autodetecting OV9281 cameras at %dx%d..." % (args.width, args.height))
        found = autodetect_cameras(args.width, args.height)
        if len(found) < 2:
            print("[Sweep] ERROR: found %d camera(s) at %dx%d: %s. "
                  "Pass --left-cam/--right-cam explicitly."
                  % (len(found), args.width, args.height, found))
            return 1
        left_idx = found[0] if left_idx is None else left_idx
        right_idx = found[1] if right_idx is None else right_idx
    print("[Sweep] Left = device %d, Right = device %d" % (left_idx, right_idx))

    caps = {}
    for side, idx in (("left", left_idx), ("right", right_idx)):
        cap = open_camera(idx, args.width, args.height)
        if cap is None:
            print("[Sweep] WARNING: %s camera (device %d) would not open/read; "
                  "continuing without it." % (side, idx))
        else:
            caps[side] = cap
    if not caps:
        print("[Sweep] ERROR: no cameras available.")
        return 1

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = args.out or os.path.join("build", "replays", "sweep_" + stamp)
    raw_dir = os.path.join(out_dir, "raw")
    os.makedirs(raw_dir, exist_ok=True)

    total = len(args.exposures) * len(args.gains)
    print("[Sweep] Recording %d settings x %d frames -> %s" % (total, args.frames, out_dir))
    print("[Sweep] Keep the ball stationary on the tee for the whole sweep.\n")

    settings = []
    step = 0
    for exposure in args.exposures:
        for gain in args.gains:
            step += 1
            grabbed = capture_setting(caps, exposure, gain, args.settle, args.frames)

            entry = {
                "index": step,
                "exposure_requested": exposure,
                "gain_requested": gain,
                "exposure_applied": {s: c.get(cv2.CAP_PROP_EXPOSURE) for s, c in caps.items()},
                "gain_applied": {s: c.get(cv2.CAP_PROP_GAIN) for s, c in caps.items()},
                "frames": {},
                "stats": {},
            }
            for side, images in grabbed.items():
                names = []
                for n, img in enumerate(images):
                    name = "%s_e%d_g%d_%02d.png" % (side, exposure, gain, n)
                    cv2.imwrite(os.path.join(raw_dir, name), img)
                    names.append(name)
                entry["frames"][side] = names
                if images:
                    ref = images[-1]
                    entry["stats"][side] = {
                        "mean": float(ref.mean()),
                        "p50": float(np.percentile(ref, 50)),
                        "p99": float(np.percentile(ref, 99)),
                        "max": int(ref.max()),
                        "saturated_pct": float(100.0 * np.count_nonzero(ref >= 250) / ref.size),
                    }
            settings.append(entry)

            summary = "  ".join(
                "%s: mean %5.1f p99 %5.0f max %3d"
                % (s, entry["stats"][s]["mean"], entry["stats"][s]["p99"], entry["stats"][s]["max"])
                for s in sorted(entry["stats"])
            )
            print("[%3d/%d] exp %3d  gain %3d   %s" % (step, total, exposure, gain, summary))

    manifest = {
        "recorded_at": datetime.datetime.now().isoformat(timespec="seconds"),
        "label": args.label,
        "strobe_mode": args.strobe,
        "resolution": [args.width, args.height],
        "devices": {"left": left_idx, "right": right_idx},
        "frames_per_setting": args.frames,
        "settle_frames": args.settle,
        "settings": settings,
    }
    with open(os.path.join(out_dir, "manifest.json"), "w", encoding="utf-8") as fh:
        json.dump(manifest, fh, indent=2)

    for cap in caps.values():
        cap.release()

    print("\n[Sweep] Done. %d settings written to %s" % (len(settings), out_dir))
    print("[Sweep] Analyse with:\n  python tools/analysis/analyze_ir_dots.py --sweep " + out_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
