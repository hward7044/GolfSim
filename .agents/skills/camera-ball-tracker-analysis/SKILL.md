---
name: camera-ball-tracker-analysis
description: Instructions and workflow for analyzing raw stereo camera frames, tuning ball detector thresholds, generating diagnostic images (DoG, TopHat, Hough, Stereo Triangulation), and fine-tuning C++ tracker parameters.
---

# Camera Ball Tracker Analysis & Tuning Workflow

This skill documents the complete workflow for inspecting camera frame replays/streams, diagnosing detector issues, and tuning the stereoscopic ball detector.

## 1. Overview of the Pipeline
- **Raw Input**: Stereo camera frame pairs stored in `build/replays/stream_.../raw/` or `shot_.../raw/`.
- **ROI**: Tee Floor Zone cropped at `x=350..950, y=440..750` (`cv::Rect(350, 440, 600, 310)`).
- **Candidate Filtering**:
  - `minBallRadiusPx`: `5.0 px`
  - `maxBallRadiusPx`: `50.0 px`
  - `minCircularity`: `0.45`
  - `ballThreshold`: `100` (for dark mat + side illumination)
  - `epipolarTolerancePx`: `50.0 px`

## 2. Automated Diagnostic Analysis Command
Run the Python diagnostic tool against any stream or replay folder:

```bash
python tools/analysis/analyze_replay_stream.py --replay build/replays/stream_20260811_201332_702 --frame 25 --thresh 100 --epipolar 50.0
```

### Outputs Generated in `<replay_dir>/annotated/`:
- `left_thresh_100_debug.png` / `right_thresh_100_debug.png`: Shows passing candidates in green and rejected candidates in orange.
- `left_thresh_100_bin.png` / `right_thresh_100_bin.png`: Binary threshold mask.
- `left_dog_debug.png` / `right_dog_debug.png`: Difference of Gaussians filter output.
- `left_tophat_debug.png` / `right_tophat_debug.png`: Morphological Top-Hat filter output.
- `left_hough_debug.png` / `right_hough_debug.png`: Hough Circles detector output.
- `stereo_match_visual.png`: Visualizes stereo pair lock line and displays disparity and epipolar error.

## 3. How to Interpret Results & Diagnostic Steps

1. **If Candidate Count is Too High (>20 candidates)**:
   - *Cause*: Background lighting is too bright or mat texture is reflective.
   - *Fix*: Raise intensity threshold (`thresh = 100..120`) or darken room/ambient light.

2. **If Golf Ball is Missed (0 candidates)**:
   - *Cause*: Threshold is too high, or ball dome merges with background.
   - *Fix*: Inspect `left_thresh_X_bin.png` to check pixel intensity of the ball. Adjust `thresh` or ensure lighting shines across the ball dome.

3. **If Stereo Match Fails (`No stereo pair match found`)**:
   - *Cause*: Epipolar vertical error between left and right camera views exceeds `epipolarTol`.
   - *Fix*: Check `epiErr` in console output and adjust `epipolarTol` (default: `50.0 px`).

## 4. Updating C++ Pipeline
When optimal parameters are determined:
1. Update `include/Math/StereoBallTrackerTrigger.hpp` default constructor parameters.
2. Update instantiation in `src/main.cpp`.
3. Rebuild with MSVC:
   ```cmd
   cmd.exe /c 'call "C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvars64.bat" && "C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" --build C:\Github\GolfSim\build --config Release'
   ```
