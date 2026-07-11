# Shot Replay & Diagnostics Feature

The Shot Replay and Diagnostics feature enables offline, frame-by-frame analysis of golf shots. It captures the raw stereo camera frames during a swing, automatically draws visual overlays mapping what the algorithms "saw," and writes detailed metadata detailing why the pipeline triggered, tracked, or rejected candidates.

---

## 1. How It Works (Architecture)

To maintain high performance in a high-speed (1000 FPS) launch monitor environment, the feature uses a decoupled, zero-allocation, and asynchronous architecture:

1. **Decoupled Diagnostics (`IDiagnosticProvider`)**:
   Components (like `OpticalGateTrigger` and `OpenCVMomentsTracker`) implement the `IDiagnosticProvider` interface. On frame execution, they store internal state inside a JSON object queryable via `getLatestDiagnostics()`. This keeps the math pipeline signatures standard and decoupled from OpenCV types.
2. **Zero Dynamic Allocation**:
   The `SessionStateMachine` maintains a pre-allocated pool of 40 frame buffers. During a shot, incoming frames are copied using `cv::Mat::copyTo()`, preventing dynamic heap allocation in the critical camera consumer loop.
3. **Thread-Safe Asynchronous I/O**:
   The `FlightRecorder` runs a dedicated background worker thread. When a shot completes, the frames are cloned and enqueued. The worker thread handles image writing, dynamic drawing, and folder limit capping in the background.
4. **Storage Capping**:
   To prevent false triggers from filling up disk space, the recorder automatically limits stored replays to **10**. When a new replay is saved, the oldest folder is deleted.

---

## 2. Replay Directory Layout

Replays are saved under `build/replays/shot_YYYYMMDD_HHMMSS_mmm/`:

```
shot_YYYYMMDD_HHMMSS_mmm/
├── metadata.json           # Kinematics results & frame-by-frame diagnostic JSON
├── raw/
│   ├── left_000.png        # Raw captured left grayscale camera frame
│   └── right_000.png       # Raw captured right grayscale camera frame
└── annotated/
    ├── left_000.png        # Left camera frame with diagnostic overlays
    └── right_000.png       # Right camera frame with diagnostic overlays
```

### Metadata JSON Schema
The `metadata.json` lists solved speed, launch angles, spin, and spin axis, along with a `frames` array detailing:
- Trigger statistics (`nonZeroCount` and threshold value).
- All detected ball candidates (centroid, bounding box, area, circularity, overlapping status, accepted flag, and reasons like `Noise: Area too small`).
- Marker coordinates detected within each candidate.
- Reconstructed 3D world coordinates.

---

## 3. Visual Overlay Annotations

The images saved in `annotated/` automatically render telemetry overlays:

- **Trigger Box (Orange)**: Drawn around the optical gate ROI on the left camera frames, labeled with the motion pixel count vs trigger threshold (e.g. `Gate Pixels: 184 / 150`).
- **Accepted Balls (Green)**: Drawn around the bounding box of a tracked ball, marked with a crosshair at its centroid, and labeled with its area (e.g. `Ball (A:456)`).
- **Sub-pixel Markers (Blue)**: Small solid circles marking the location of high-brightness glints on the ball surface used for spin calculation.
- **Noise / Rejected Candidates (Red)**: Thin red boxes outlining contours that failed tracking constraints, labeled with the specific rejection reason (e.g. `Noise: Area too small`).
- **3D World Coordinates (Yellow)**: Placed in the top-left corner showing the reconstructed world coordinate: `3D: (X, Y, Z)` in meters.

---

## 4. Interactive Replay Viewer

You can play back and step through any saved shot using the C++ interactive viewer:

### Usage Command
```powershell
./build/GolfSim.exe --replay build/replays/shot_YYYYMMDD_HHMMSS_mmm
```

### Controls
| Key | Action |
| --- | --- |
| `SPACE` | Play / Pause auto-playback (100ms interval) |
| `d` / `Arrow Right` | Step forward 1 frame |
| `a` / `Arrow Left` | Step backward 1 frame |
| `o` | Toggle diagnostic overlays ON / OFF |
| `ESC` | Exit replay viewer |
