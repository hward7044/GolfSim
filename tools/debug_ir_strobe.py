"""
GolfSim IR Strobe & Retroreflective Dot Visualizer Tool
-------------------------------------------------------
Runs a live webcam feed from cameras 0 and 1, with real-time thresholding
and glint detection to verify IR emitter circuit output and retroreflective ball dots.

Usage:
  python tools/debug_ir_strobe.py [--left-cam 1] [--right-cam 0] [--thresh 200]
"""

import sys
import cv2
import numpy as np
import argparse

def main():
    parser = argparse.ArgumentParser(description="GolfSim IR Strobe & Retroreflective Dot Visualizer")
    parser.add_argument("--left-cam", type=int, default=1, help="Device index for Left camera (default: 1)")
    parser.add_argument("--right-cam", type=int, default=0, help="Device index for Right camera (default: 0)")
    parser.add_argument("--thresh", type=int, default=200, help="Initial threshold level (default: 200)")
    args = parser.parse_args()

    print(f"[IR Debugger] Opening Camera Left (Index {args.left_cam}) and Camera Right (Index {args.right_cam})...")
    capL = cv2.VideoCapture(args.left_cam, cv2.CAP_DSHOW)
    capR = cv2.VideoCapture(args.right_cam, cv2.CAP_DSHOW)

    for cap in [capL, capR]:
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUY2'))

    thresh = args.thresh
    view_mode = 0  # 0: Side-by-side with Glint circles, 1: Threshold Mask, 2: Left, 3: Right

    print("\n=======================================================")
    print("GOLFSIM IR STROBE DEBUGGER CONTROLS:")
    print("  'v' or TAB : Toggle view modes (Annotated Live -> Threshold Mask -> Single View)")
    print("  '+' / '-'  : Increase / Decrease Threshold (Current: 200)")
    print("  's'        : Print intensity stats & retroreflective glint counts")
    print("  ESC / 'q'  : Exit")
    print("=======================================================\n")

    win_name = "GolfSim IR Strobe Debugger"
    cv.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv.resizeWindow(win_name, 1280, 800)

    while True:
        retL, frameL = capL.read() if capL.isOpened() else (False, None)
        retR, frameR = capR.read() if capR.isOpened() else (False, None)

        if not retL and not retR:
            print("[IR Debugger] Error: Unable to read frames from video devices.")
            break

        def process_side(img, title):
            if img is None:
                blank = np.zeros((400, 640, 3), dtype=np.uint8)
                cv2.putText(blank, f"{title} OFFLINE", (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                return blank, 0, 0

            if len(img.shape) == 3:
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            else:
                gray = img

            # Threshold for retroreflective dots
            _, mask = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if view_mode == 1: # Threshold mask mode
                annotated = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            else:
                annotated = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                sat_count = np.sum(gray >= 240)
                # Highlight retroreflective dots (> thresh) in yellow/red
                for c in contours:
                    area = cv2.contourArea(c)
                    if area > 1 and area < 10000:
                        (cx, cy), r = cv2.minEnclosingCircle(c)
                        cv2.circle(annotated, (int(cx), int(cy)), int(r) + 5, (0, 0, 255), 2)
                        cv2.circle(annotated, (int(cx), int(cy)), 2, (0, 255, 255), -1)

            mean_val = np.mean(gray)
            sat_pixels = np.sum(gray >= 240)

            cv2.putText(annotated, f"{title} | Mean: {mean_val:.1f} | Sat(>=240): {sat_pixels}px", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(annotated, f"Glints (Thresh >= {thresh}): {len(contours)}", (20, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            return annotated, mean_val, len(contours)

        dispL, meanL, countL = process_side(frameL, "LEFT")
        dispR, meanR, countR = process_side(frameR, "RIGHT")

        resL = cv2.resize(dispL, (640, 400))
        resR = cv2.resize(dispR, (640, 400))
        combined = np.hstack((resL, resR))

        cv2.imshow(win_name, combined)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):
            break
        elif key == ord('v') or key == 9:
            view_mode = (view_mode + 1) % 2
        elif key == ord('+') or key == ord('='):
            thresh = min(254, thresh + 5)
            print(f"[IR Debugger] Threshold set to {thresh}")
        elif key == ord('-') or key == ord('_'):
            thresh = max(10, thresh - 5)
            print(f"[IR Debugger] Threshold set to {thresh}")
        elif key == ord('s'):
            print(f"[IR Debugger Snapshot] Thresh={thresh} | Left: mean={meanL:.1f}, glints={countL} | Right: mean={meanR:.1f}, glints={countR}")

    capL.release()
    capR.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
