# Shot Replay & Diagnostics Feature

The Shot Replay and Diagnostics feature enables offline, frame-by-frame analysis of golf shots. It captures the raw stereo camera frames during a swing, automatically draws visual overlays mapping what the algorithms "saw," and writes detailed metadata detailing why the pipeline triggered, tracked, or rejected candidates.

---

## 1. How It Works (Architecture)

To maintain high performance in a high-speed (1000 FPS) launch monitor environment, the feature uses a decoupled, zero-allocation, and asynchronous architecture:

1. **Decoupled Diagnostics (`IDiagnosticProvider`)**:
   Components (like `StereoBallTrackerTrigger` and `DotClusterTracker`) implement the `IDiagnosticProvider` interface. On frame execution, they store internal state inside a JSON object queryable via `getLatestDiagnostics()`. This keeps the math pipeline signatures standard and decoupled from OpenCV types.
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
- Trigger state and, while searching, the stereo geometry of the best pair (`disparityPx`, `verticalOffsetPx`) plus the dot-cluster finder's view of each camera (`leftDots` / `rightDots`).
- All detected ball candidates (centroid, nominal bounding box, dot count, cluster spread, accepted flag, and reasons like `Cluster: too few dots`).
- Every dot: accepted ones as the candidate's `markers`, rejected ones with a reason (`Dot: elongated (aspect)` for specular bars, `Dot: area too small` for sensor sparkle).
- Reconstructed 3D world coordinates.

It also carries a `session` block: the `AppConfig` in force, its source file, and per camera the **applied** exposure (after UVC quantisation), gain and negotiated frame rate, plus the strobe timing the pipeline validated against. A replay therefore says exactly what the hardware was doing.

Stream sessions (`--stream`) record the same trigger and vision diagnostics and write `annotated/` frames too, so a recording made with a stationary ball shows whether the detector sees it.

---

## 3. Visual Overlay Annotations

The images saved in `annotated/` automatically render telemetry overlays:

- **Trigger State (Orange, bottom-left of the left frame)**: `Trigger: SEARCHING (stable 3) | dx 167 dy 0 px` — state, stability counter and the measured stereo disparity / vertical offset of the best pair. A locked ball is boxed in cyan.
- **Accepted Balls (Green)**: The nominal ball box around the dot cluster's centroid, a crosshair at the centroid, and the label `Ball (9 dots, 21 px)` — dot count and cluster spread.
- **Dots (Blue)**: Small circles on every dot that belongs to an accepted cluster; these are the markers the spin solver uses.
- **Rejected (Red)**: Thin boxes around dots that failed the area or aspect gates (specular bars, sensor sparkle) and around clusters that failed the count/spread gates, labeled with the reason (e.g. `Noise: Cluster: too few dots`).
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
