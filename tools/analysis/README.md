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
