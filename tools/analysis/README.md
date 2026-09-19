# GolfSim Diagnostic & Analysis Tools

This folder contains Python diagnostic tools for analyzing camera replays/streams, tuning ball detection parameters, and visualizing stereo triangulation.

## Quick Start

### Run Diagnostic Suite on a Replay / Stream
To analyze a stream folder and generate diagnostic images (Thresholding, DoG, TopHat, Hough, Stereo Pair):

```bash
python tools/analysis/analyze_replay_stream.py --replay build/replays/stream_20260811_201332_702
```

### Options
- `--replay <path>`: Path to replay directory (required).
- `--frame <N>`: Frame index to analyze (default: `25`).
- `--thresh <N>`: Intensity threshold value (default: `100`).
- `--epipolar <N>`: Maximum allowed vertical disparity in pixels (default: `50.0`).

---

## Output Files

The tool populates the `<replay_dir>/annotated/` folder with:
- `left_thresh_100_debug.png` / `right_thresh_100_debug.png`: ROI contour overlays (Green = Passed, Orange = Rejected).
- `left_thresh_100_bin.png` / `right_thresh_100_bin.png`: Binary threshold mask.
- `left_dog_debug.png` / `right_dog_debug.png`: Difference of Gaussians filter output.
- `left_tophat_debug.png` / `right_tophat_debug.png`: Morphological Top-Hat filter output.
- `left_hough_debug.png` / `right_hough_debug.png`: Hough Circles output.
- `stereo_match_visual.png`: Stereo pair visualization connecting left/right centroids with disparity metrics.

---

# IR Exposure / Gain Tuning Workflow

The launch monitor is designed to run in a **dots-only** imaging regime: exposure
and gain are set low enough that ambient light falls away to black, and the only
thing the sensor sees is the retroreflective dots on the ball lit by the IR
emitter. Use this two-step workflow to find those settings.

## Step 1 — Record a sweep

```bash
python tools/ir_exposure_sweep.py --strobe continuous
```

Put the ball on the tee and leave it stationary. The tool walks a matrix of
exposure and gain values, capturing frames from both cameras at each one, and
writes them to `build/replays/sweep_<timestamp>/`.

### Options
- `--strobe <mode>`: command the emitter first — `continuous` (DC on, best for
  static tuning), `strobe` (300 Hz), `standby` (10 Hz), `off`, `none` (default).
- `--com <port>`: strobe controller serial port (default: `COM3`).
- `--exposures <values>`: UVC log2-second exposures (default: `-13 … -5`).
- `--gains <values>`: gain values 0–100 (default: `0 25 50 75 100`).
- `--frames <N>` / `--settle <N>`: frames kept per setting / frames discarded
  after a setting change (defaults: 3 / 6).
- `--left-cam` / `--right-cam`: DSHOW device indices. Autodetected by default.

## Step 2 — Analyse the recording

```bash
python tools/analysis/analyze_ir_dots.py --sweep build/replays/sweep_<timestamp>
```

Scores every frame on how cleanly it shows a dots-only ball — dark background,
several separated dot blobs, and a cluster roughly one ball diameter across —
and ranks the settings. Also runs on frames recorded by the C++ pipeline:

```bash
python tools/analysis/analyze_ir_dots.py --replay build/replays/stream_<timestamp>
```

### Output (`<input>/analysis/`)
- `report.txt` / `report.json`: ranked table of every setting with background
  level, peak, threshold, dot count, cluster spread, bright-pixel %, and score.
  Where both cameras found a cluster, a `Stereo pairs` section gives `xL-xR`
  (disparity — negative means the registered camera roles are swapped; set
  `stereo.swapCameras` in `config/golfsim.json`) and `yL-yR` (vertical offset —
  must stay inside `stereo.epipolarTolerancePx`).
- `sweep_left.png` / `sweep_right.png`: contact sheet of the whole sweep.
- `top_contact_sheet.png` and `top_*.png`: the best-scoring frames, annotated
  with detected dots (green) and the fitted ball cluster (yellow).

## Measured camera control ranges

Arducam OV9281 at 1280x800 YUY2, over UVC. These are the same controls the C++
`MediaFoundationDriver` drives, so values found here transfer to production.

| Control | Usable range | Notes |
| --- | --- | --- |
| `EXPOSURE` | `-13` … `-5` | UVC log2 seconds. Above `-5` the frame period clamps it — setting `-4`…`-1` changes nothing. |
| `GAIN` | `0` … `100` | Values above 100 are rejected. |
| `BRIGHTNESS` | `0` … `64` | Additive black level. **Keep at 0** — it lifts the background floor, which is the opposite of what dots-only imaging needs. |

Under DSHOW the two OV9281s enumerate as device **1** and **2**; device 0 is the
integrated laptop webcam (1280x720). The C++ driver enumerates differently — it
filters to Arducam VID/PID matches only, so its indices 0 and 1 are the two
OV9281s.

## Step 3 — Put the values into the pipeline

The C++ application reads `config/golfsim.json` at startup (override the path
with `--config`). Copy the winning exposure, gain and threshold in:

```json
{
  "camera":   { "exposureUs": 7812, "gain": 0, "brightness": 0, "targetFps": 100 },
  "detector": { "intensityThreshold": 135, "clusterRadiusPx": 28, "minDotsPerCluster": 3 },
  "stereo":   { "epipolarTolerancePx": 150, "swapCameras": false }
}
```

Command-line overrides for quick experiments: `--exposure-us N`, `--gain N`,
`--fps N`, `--intensity N`, `--swap-cameras`. The startup log prints the values
the hardware actually applied (exposure snaps to the nearest UVC step) and the
negotiated frame rate; every replay's `metadata.json` records them too.

The live viewer (`GolfSim.exe --live`) runs the production detector on the
video: `v` cycles to the *Dot Clusters* view, `e`/`E` step exposure one UVC
stop, `g`/`G` step gain by 5, `+`/`-` move the intensity threshold, and `p`
prints the current values as config JSON to paste into the file.

Note that the sweep tool records with `DSHOW` and the application captures with
Media Foundation; the underlying UVC controls are the same, so exposure/gain
values transfer, but the sweep's YUY2 stream runs at 30 fps while the
application negotiates the camera's 100 fps NV12 mode.
