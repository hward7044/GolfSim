#!/usr/bin/env python3
"""
Camera Ball Tracker Diagnostic Analysis Tool
=============================================
Analyzes raw stereo camera replays/streams and generates a full suite of
diagnostic images (Thresholding, DoG, TopHat, Hough Circles, and Stereo Triangulation).

Usage:
    python tools/analysis/analyze_replay_stream.py --replay build/replays/stream_YYYYMMDD_HHMMSS_mmm
    python tools/analysis/analyze_replay_stream.py --replay build/replays/shot_YYYYMMDD_HHMMSS_mmm --frame 25 --thresh 100
"""

import os
import argparse
import cv2
import numpy as np

def run_analysis(replay_dir, frame_idx=25, thresh_val=120, epipolar_tol=65.0, min_radius=15.0, min_area=150.0):
    raw_dir = os.path.join(replay_dir, "raw")
    annotated_dir = os.path.join(replay_dir, "annotated")
    os.makedirs(annotated_dir, exist_ok=True)

    frame_str = f"{frame_idx:03d}"
    left_path = os.path.join(raw_dir, f"left_{frame_str}.png")
    right_path = os.path.join(raw_dir, f"right_{frame_str}.png")

    if not os.path.exists(left_path) or not os.path.exists(right_path):
        print(f"Error: Frame files not found: {left_path} or {right_path}")
        return

    imgL = cv2.imread(left_path, cv2.IMREAD_GRAYSCALE)
    imgR = cv2.imread(right_path, cv2.IMREAD_GRAYSCALE)

    print(f"=== ANALYZING REPLAY: {os.path.basename(replay_dir)} (Frame {frame_str}) ===")
    print(f"Left Image  Stats: Min={np.min(imgL)}, Max={np.max(imgL)}, Mean={np.mean(imgL):.1f}")
    print(f"Right Image Stats: Min={np.min(imgR)}, Max={np.max(imgR)}, Mean={np.mean(imgR):.1f}")

    # Fine-tuned Tee Floor Zone ROI (x=350..950, y=440..750)
    roi = (350, 440, 600, 310)
    x, y, w, h = roi

    cropL = imgL[y:y+h, x:x+w]
    cropR = imgR[y:y+h, x:x+w]

    print(f"Left ROI  Stats: Min={np.min(cropL)}, Max={np.max(cropL)}, Mean={np.mean(cropL):.1f}")
    print(f"Right ROI Stats: Min={np.min(cropR)}, Max={np.max(cropR)}, Mean={np.mean(cropR):.1f}")

    # 1. THRESHOLDING DIAGNOSTICS
    def extract_candidates(frame, thresh):
        safeRoi = frame[y:y+h, x:x+w]
        blurred = cv2.GaussianBlur(safeRoi, (3, 3), 0)
        _, bin_mat = cv2.threshold(blurred, thresh, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(bin_mat, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        candidates = []
        annotated = cv2.cvtColor(safeRoi, cv2.COLOR_GRAY2BGR)
        
        for c in contours:
            area = cv2.contourArea(c)
            if area < min_area: continue
            (cx, cy), radius = cv2.minEnclosingCircle(c)
            perimeter = cv2.arcLength(c, True)
            circ = (4.0 * np.pi * area) / (perimeter * perimeter) if perimeter > 0 else 0
            passed = (radius >= min_radius and radius <= 50.0 and circ >= 0.45)
            
            m = cv2.moments(c)
            if m['m00'] > 0:
                cen_x = x + m['m10'] / m['m00']
                cen_y = y + m['m01'] / m['m00']
                if passed:
                    cv2.drawContours(annotated, [c], -1, (0, 255, 0), 2)
                    cv2.circle(annotated, (int(cx), int(cy)), int(radius), (255, 0, 0), 1)
                    candidates.append({'pos': (cen_x, cen_y), 'radius': radius, 'area': area, 'circ': circ})
                else:
                    cv2.drawContours(annotated, [c], -1, (0, 165, 255), 1)

        return candidates, annotated, bin_mat

    candsL, annL, binL = extract_candidates(imgL, thresh_val)
    candsR, annR, binR = extract_candidates(imgR, thresh_val)

    print(f"\n--- Threshold {thresh_val} Candidates ---")
    print(f"Left Candidates: {len(candsL)}")
    for i, c in enumerate(candsL):
        print(f"  Left [{i}]: Pos=({c['pos'][0]:.1f}, {c['pos'][1]:.1f}), Radius={c['radius']:.1f}px, Circ={c['circ']:.2f}")

    print(f"Right Candidates: {len(candsR)}")
    for i, c in enumerate(candsR):
        print(f"  Right [{i}]: Pos=({c['pos'][0]:.1f}, {c['pos'][1]:.1f}), Radius={c['radius']:.1f}px, Circ={c['circ']:.2f}")

    cv2.imwrite(os.path.join(annotated_dir, f"left_thresh_{thresh_val}_debug.png"), annL)
    cv2.imwrite(os.path.join(annotated_dir, f"right_thresh_{thresh_val}_debug.png"), annR)
    cv2.imwrite(os.path.join(annotated_dir, f"left_thresh_{thresh_val}_bin.png"), binL)
    cv2.imwrite(os.path.join(annotated_dir, f"right_thresh_{thresh_val}_bin.png"), binR)

    # 2. DIFFERENCE OF GAUSSIANS (DoG)
    def generate_dog_debug(img, name):
        safeRoi = img[y:y+h, x:x+w]
        g1 = cv2.GaussianBlur(safeRoi, (5, 5), 1.0)
        g2 = cv2.GaussianBlur(safeRoi, (21, 21), 5.0)
        dog = cv2.subtract(g1, g2)
        _, bin_mat = cv2.threshold(dog, 15, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(bin_mat, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        annotated = cv2.cvtColor(safeRoi, cv2.COLOR_GRAY2BGR)
        for c in contours:
            area = cv2.contourArea(c)
            if area < 5: continue
            (cx, cy), radius = cv2.minEnclosingCircle(c)
            perimeter = cv2.arcLength(c, True)
            circ = (4.0 * np.pi * area) / (perimeter * perimeter) if perimeter > 0 else 0
            passed = (radius >= 5.0 and radius <= 50.0 and circ >= 0.40)
            if passed: cv2.drawContours(annotated, [c], -1, (0, 255, 0), 2)
            else: cv2.drawContours(annotated, [c], -1, (0, 165, 255), 1)
        cv2.imwrite(os.path.join(annotated_dir, f"{name}_dog_debug.png"), annotated)
        cv2.imwrite(os.path.join(annotated_dir, f"{name}_dog_bin.png"), bin_mat)

    generate_dog_debug(imgL, "left")
    generate_dog_debug(imgR, "right")

    # 3. TOP-HAT TRANSFORM
    def generate_tophat_debug(img, name):
        safeRoi = img[y:y+h, x:x+w]
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (25, 25))
        tophat = cv2.morphologyEx(safeRoi, cv2.MORPH_TOPHAT, kernel)
        _, bin_mat = cv2.threshold(tophat, 15, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(bin_mat, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        annotated = cv2.cvtColor(safeRoi, cv2.COLOR_GRAY2BGR)
        for c in contours:
            area = cv2.contourArea(c)
            if area < 5: continue
            (cx, cy), radius = cv2.minEnclosingCircle(c)
            perimeter = cv2.arcLength(c, True)
            circ = (4.0 * np.pi * area) / (perimeter * perimeter) if perimeter > 0 else 0
            passed = (radius >= 5.0 and radius <= 50.0 and circ >= 0.40)
            if passed: cv2.drawContours(annotated, [c], -1, (0, 255, 0), 2)
            else: cv2.drawContours(annotated, [c], -1, (0, 165, 255), 1)
        cv2.imwrite(os.path.join(annotated_dir, f"{name}_tophat_debug.png"), annotated)
        cv2.imwrite(os.path.join(annotated_dir, f"{name}_tophat_bin.png"), bin_mat)

    generate_tophat_debug(imgL, "left")
    generate_tophat_debug(imgR, "right")

    # 4. HOUGH CIRCLES
    def generate_hough_debug(img, name):
        safeRoi = img[y:y+h, x:x+w]
        blurred = cv2.GaussianBlur(safeRoi, (5, 5), 1.5)
        circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1.0, minDist=30.0,
                                   param1=50.0, param2=15.0, minRadius=5, maxRadius=20)
        annotated = cv2.cvtColor(safeRoi, cv2.COLOR_GRAY2BGR)
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (cx, cy, r) in circles:
                cv2.circle(annotated, (cx, cy), r, (0, 255, 0), 2)
                cv2.circle(annotated, (cx, cy), 2, (0, 0, 255), 3)
        cv2.imwrite(os.path.join(annotated_dir, f"{name}_hough_debug.png"), annotated)

    generate_hough_debug(imgL, "left")
    generate_hough_debug(imgR, "right")

    # 5. STEREO MATCHING
    best_pair = None
    min_score = 1e9

    for cL in candsL:
        for cR in candsR:
            epiErr = abs(cL['pos'][1] - cR['pos'][1])
            disp = cR['pos'][0] - cL['pos'][0] if args.swap else (cL['pos'][0] - cR['pos'][0])
            if epiErr <= epipolar_tol and 10.0 <= disp <= 300.0:
                radDiff = abs(cL['radius'] - cR['radius']) / max(cL['radius'], cR['radius'])
                circPenalty = 1.0 - (cL['circ'] * cR['circ'])
                score = epiErr + (10.0 * radDiff) + (50.0 * circPenalty)
                if score < min_score:
                    min_score = score
                    best_pair = (cL, cR, epiErr, disp, score)

    stereo_vis = np.hstack((cv2.cvtColor(imgL, cv2.COLOR_GRAY2BGR), cv2.cvtColor(imgR, cv2.COLOR_GRAY2BGR)))

    if best_pair:
        cL, cR, epi, disp, score = best_pair
        ptL = (int(cL['pos'][0]), int(cL['pos'][1]))
        ptR = (int(cR['pos'][0]) + imgL.shape[1], int(cR['pos'][1]))
        cv2.circle(stereo_vis, ptL, int(cL['radius']), (0, 255, 0), 3)
        cv2.circle(stereo_vis, ptR, int(cR['radius']), (0, 255, 0), 3)
        cv2.line(stereo_vis, ptL, ptR, (0, 255, 255), 2)
        text = f"STEREO LOCK: Disparity={disp:.1f}px | EpiErr={epi:.1f}px"
        cv2.putText(stereo_vis, text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        print("\nSTEREO PAIR MATCH FOUND!")
        print(f"  Left Ball:  Pos=({cL['pos'][0]:.1f}, {cL['pos'][1]:.1f}), Radius={cL['radius']:.1f}px, Circ={cL['circ']:.2f}")
        print(f"  Right Ball: Pos=({cR['pos'][0]:.1f}, {cR['pos'][1]:.1f}), Radius={cR['radius']:.1f}px, Circ={cR['circ']:.2f}")
        print(f"  Epipolar Error: {epi:.1f} px, Disparity: {disp:.1f} px")
    else:
        print("\nNo stereo pair match found under current epipolar tolerance.")

    cv2.imwrite(os.path.join(annotated_dir, "stereo_match_visual.png"), stereo_vis)
    print(f"\nAll diagnostic images successfully generated in: {annotated_dir}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Analyze stereo replay/stream session and generate diagnostic images.")
    parser.add_argument("--replay", required=True, help="Path to replay or stream folder (e.g. build/replays/stream_20260811_201332_702)")
    parser.add_argument("--frame", type=int, default=25, help="Frame index to analyze (default: 25)")
    parser.add_argument("--thresh", type=int, default=120, help="Intensity threshold value (default: 120)")
    parser.add_argument("--epipolar", type=float, default=65.0, help="Epipolar tolerance in pixels (default: 65.0)")
    parser.add_argument("--min-radius", type=float, default=15.0, help="Minimum ball radius in pixels (default: 15.0)")
    parser.add_argument("--min-area", type=float, default=150.0, help="Minimum contour area in pixels^2 (default: 150.0)")
    parser.add_argument("--swap", action="store_true", help="Swap Left and Right streams for disparity calculation")
    args = parser.parse_args()

    run_analysis(args.replay, args.frame, args.thresh, args.epipolar, args.min_radius, args.min_area)
