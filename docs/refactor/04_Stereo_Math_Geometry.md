# Refactor 04: Stereo Mathematics & Coordinate Transformations

This document details the mathematical proofs, coordinate frame derivations, and implementation fixes for stereoscopic 3D reconstruction and ray-sphere fallback solving.

---

## 1. Stereo Coordinate Frame Conventions

In stereoscopic computer vision (specifically OpenCV's `stereoCalibrate` and `stereoRectify`), the coordinate system of **Camera 1 (Left Camera)** is established as the **World Coordinate Frame**:
- Origin $(0, 0, 0)$: Optical center of Left Camera.
- $+X$ axis: Points horizontally to the right (towards the Right Camera).
- $+Y$ axis: Points vertically downwards (OpenCV image coordinate convention).
- $+Z$ axis: Points forward along the optical axis (depth into the scene).

```
         World Origin (0,0,0)               Offset Origin (B, 0, 0)
         Left Camera Center                   Right Camera Center
                 │                                     │
                 ▼                                     ▼
                [CL] ───────────── Baseline B ───────── [CR]
                 \                                     /
                  \                                   /
                   \             * (X, Y, Z)         /
                    \           Ball Marker         /
                     \                             /
```

The transformation from Left Camera space ($\mathbf{X}_L$) to Right Camera space ($\mathbf{X}_R$) is defined by rotation matrix $R$ and translation vector $T$:
$$\mathbf{X}_R = R \cdot \mathbf{X}_L + T$$

---

## 2. Derivation of Right Camera Ray-Sphere Fallback

### 2.1 The Ray-Sphere Geometry
When a retroreflective marker dot is visible in the Right Camera but occluded in the Left Camera:
- We know the ball centroid $\mathbf{C} = (X_c, Y_c, Z_c)$ in world coordinates (from stereo triangulation of the ball silhouette).
- We know the ball radius $r = 0.021335\text{ m}$.
- The marker lies on the spherical surface:
$$\|\mathbf{P} - \mathbf{C}\|^2 = r^2$$

A detected pixel in the Right Camera produces a normalized camera ray $\mathbf{d}_R = (x_{\text{norm}}, y_{\text{norm}}, 1.0)^T$, with $\|\mathbf{d}_R\| = 1$.

### 2.2 Right Camera Optical Center in World Coordinates
In the Right Camera's own coordinate frame, its optical center is at the origin $\mathbf{X}_R = (0, 0, 0)^T$.
Substituting into the transformation equation to find its location in world space ($\mathbf{X}_L = \mathbf{O}_R$):
$$0 = R \cdot \mathbf{O}_R + T \implies R \cdot \mathbf{O}_R = -T \implies \mathbf{O}_R = -R^{-1} T$$
Because $R$ is an orthonormal rotation matrix, $R^{-1} = R^T$:
$$\mathbf{O}_R = -R^T \cdot T$$

For a horizontal baseline $B = 100\text{ mm}$ along $X$, OpenCV calibration yields $T = (-0.1, 0, 0)^T$.
If cameras are parallel ($R \approx I$):
$$\mathbf{O}_R = -I \cdot \begin{pmatrix} -0.1 \\ 0 \\ 0 \end{pmatrix} = \begin{pmatrix} \mathbf{+0.1} \\ 0 \\ 0 \end{pmatrix} \quad (+100\text{ mm to the right})$$

### 2.3 Ray Direction in World Coordinates
A direction vector $\mathbf{d}_R$ in Right Camera space transforms to world space ($\mathbf{d}_L$) as:
$$\mathbf{d}_R = R \cdot \mathbf{d}_L \implies \mathbf{d}_L = R^{-1} \cdot \mathbf{d}_R = \mathbf{R^T \cdot d_R}$$

### 2.4 The Existing Bug in `StereoTriangulator.cpp`
Lines 258–259 in [StereoTriangulator.cpp](file:///home/hward/Projects/GolfSim/src/Math/StereoTriangulator.cpp#L258-L259):
```cpp
// INCORRECT CODE IN REPOSITORY:
Eigen::Vector3d rayDir = R_eigen * rayCamR; // Bug: Multiplied by R instead of R^T
Eigen::Vector3d O = T_eigen;                 // Bug: Used T (-0.1) instead of -R^T * T (+0.1)
```
- The code placed the Right Camera at $X = -100\text{ mm}$ (100mm to the *left* of the left camera, 200mm away from where it physically sits!).
- The code projected rays along $R \cdot \mathbf{d}_R$ instead of $R^T \cdot \mathbf{d}_R$.

### 2.5 Implemented Formulation & Extrinsics Cache
Implemented in [src/Math/StereoTriangulator.cpp](file:///home/hward/Projects/GolfSim/src/Math/StereoTriangulator.cpp):

Extrinsics are precomputed once during calibration update rather than per-marker in the hot loop:
```cpp
void StereoTriangulator::updateExtrinsics() {
    if (!calib_.R.empty() && !calib_.T.empty()) {
        // ... matrix conversion ...
        R_inv_ = R_eigen.transpose();
        rightOpticalCenter_ = -R_inv_ * T_eigen; // Exact optical center (+100mm on X)
    }
}
```
During ray-sphere solving for unmatched right markers:
```cpp
Eigen::Vector3d rayDir = (R_inv_ * rayCamR).normalized(); // Exact ray in world frame
Eigen::Vector3d O = rightOpticalCenter_;                  // Exact right camera position (+0.1m)
```

### 2.6 Quadratic Intersection Solution
With $\mathbf{P}(t) = \mathbf{O} + t \mathbf{d}$:
$$\|\mathbf{O} + t \mathbf{d} - \mathbf{C}\|^2 = r^2$$
Let $\mathbf{w} = \mathbf{O} - \mathbf{C}$:
$$t^2 + 2 (\mathbf{w} \cdot \mathbf{d}) t + (\|\mathbf{w}\|^2 - r^2) = 0$$
Discriminant:
$$\Delta = (\mathbf{w} \cdot \mathbf{d})^2 - (\|\mathbf{w}\|^2 - r^2)$$
If $\Delta \ge 0$, the nearest intersection point facing the camera is:
$$t = -(\mathbf{w} \cdot \mathbf{d}) - \sqrt{\Delta}$$
$$\mathbf{P}_{\text{marker}} = \mathbf{O} + t \mathbf{d}$$

---

## 3. Mathematical & Interface Implementations

### 3.1 Mutual Recursion Elimination in `ITriggerDetector.hpp`
Implemented in [include/Math/ITriggerDetector.hpp](file:///home/hward/Projects/GolfSim/include/Math/ITriggerDetector.hpp):
```cpp
// Pure virtual stereoscopic trigger evaluator
virtual bool checkTrigger(const cv::Mat& leftFrame, const cv::Mat& rightFrame) = 0;

// Legacy single-frame optical gate helper delegating strictly in one direction
bool checkOpticalGate(const cv::Mat& currentFrame) {
    return checkTrigger(currentFrame, currentFrame);
}
```
- `BallPresenceTrigger` overrides `checkTrigger()` and evaluates the optical gate on the incoming frame.
- `StereoBallTrackerTrigger` overrides `checkTrigger()` and inherits the clean helper.
- Circular delegation is completely eliminated with zero risk of stack overflow.

### 3.2 Principal Motion Vector Trajectory Sorting
Implemented via `sortAlongPrincipalAxis()` in [src/Math/StereoTriangulator.cpp](file:///home/hward/Projects/GolfSim/src/Math/StereoTriangulator.cpp):

Rather than assuming horizontal image ordering ($x_a < x_b$), which fails on steep vertical launches (e.g., $70^\circ$ lob wedges) or opposite-handed bays:
1. Computes mean 2D centroid $(\bar{x}, \bar{y})$ and $2 \times 2$ covariance matrix:
   $$C = \begin{pmatrix} c_{xx} & c_{xy} \\ c_{xy} & c_{yy} \end{pmatrix}$$
2. Analytically determines the principal eigenvector $\mathbf{v} = (v_x, v_y)$ representing the dominant flight path axis.
3. Orients $\mathbf{v}$ along the flight direction:
   - If $|v_x| > 0.1$: Aligns with `flightDirectionX` ($+1.0$ default for left-to-right, $-1.0$ for right-to-left).
   - If $|v_x| \le 0.1$ (steep vertical lob wedge): Orients along $-Y$ (upward motion toward decreasing image row index).
4. Projects 2D pulse centroids onto $\mathbf{v}$:
   $$s_i = (\mathbf{p}_i - \mathbf{\bar{p}}) \cdot \mathbf{v}$$
5. Orders observations and stereo matches chronologically based on $s_i < s_j$, guaranteeing monotonic time progression for the kinematics engine.


