#!/usr/bin/env python3
"""
nav_cam_tuner_node.py — Interactive tuning node for nav cam cube detection.

Subscribes to the navigation camera and opens an OpenCV window with sliders
to tune both the high-contrast checkerboard detector AND an optional HSV
color filter. Use this to find the right parameters before integrating
into HeadPoseTracker.

Usage:
    # First launch the nav cam:
    ros2 launch stretch_core navigation_camera.launch.py

    # Then run this node:
    ros2 run vtam_core nav_cam_tuner_node

    # Or with a different topic:
    ros2 run vtam_core nav_cam_tuner_node --ros-args -p image_topic:=/nav_cam/image_raw

Sliders:
    [CONTRAST DETECTOR]
    Window Size   — patch size to score for checkerboard transitions
    Step Size     — sliding window step (smaller = more precise, slower)
    Min Score     — minimum transition count to be considered a valid detection
    Score Dilate  — how many top-scoring patches to merge into one centroid

    [HSV FILTER] (optional layer on top)
    H Min / H Max — hue range
    S Min / S Max — saturation range  
    V Min / V Max — value (brightness) range
    Use HSV       — toggle HSV mask on/off

    [DISPLAY]
    Show Thresh   — toggle adaptive threshold view
    Show Scores   — toggle heatmap of transition scores
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


WINDOW_MAIN   = "Nav Cam Tuner — Detection"
WINDOW_MASK   = "Nav Cam Tuner — Threshold / Mask"
WINDOW_SCORES = "Nav Cam Tuner — Score Heatmap"


class NavCamTunerNode(Node):
    def __init__(self):
        super().__init__('nav_cam_tuner_node')
        self.declare_parameter('image_topic', '/navigation_camera/image_raw')
        topic = self.get_parameter('image_topic').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.latest_frame = None

        self.create_subscription(Image, topic, self._image_cb, 10)
        self.create_timer(0.033, self._display_tick)  # ~30Hz display loop

        self._build_windows()
        self.get_logger().info(f"Tuner ready. Subscribed to: {topic}")
        self.get_logger().info("OpenCV windows should appear shortly.")

    # ── Window / Slider Setup ─────────────────────────────────────────────────

    def _build_windows(self):
        cv2.namedWindow(WINDOW_MAIN,   cv2.WINDOW_NORMAL)
        cv2.namedWindow(WINDOW_MASK,   cv2.WINDOW_NORMAL)
        cv2.namedWindow(WINDOW_SCORES, cv2.WINDOW_NORMAL)

        # ── ADD THESE THREE LINES ─────────────────────────────────────────────
        blank = np.zeros((100, 400, 3), dtype=np.uint8)
        cv2.imshow(WINDOW_MAIN, blank)
        cv2.imshow(WINDOW_MASK, blank)
        cv2.imshow(WINDOW_SCORES, blank)
        cv2.waitKey(1)  # flush the window creation
        # ─────────────────────────────────────────────────────────────────────

        cv2.resizeWindow(WINDOW_MAIN,   800, 600)

        cv2.resizeWindow(WINDOW_MAIN,   800, 600)
        cv2.resizeWindow(WINDOW_MASK,   800, 600)
        cv2.resizeWindow(WINDOW_SCORES, 800, 600)

        # ── Contrast detector sliders ─────────────────────────────────────────
        cv2.createTrackbar("Win Size",    WINDOW_MAIN, 60,  200, lambda v: None)
        cv2.createTrackbar("Step Size",   WINDOW_MAIN, 15,  60,  lambda v: None)
        cv2.createTrackbar("Min Score",   WINDOW_MAIN, 250, 1000, lambda v: None)
        cv2.createTrackbar("Top-N Merge", WINDOW_MAIN, 5,   20,  lambda v: None)

        # ── HSV filter sliders ────────────────────────────────────────────────
        cv2.createTrackbar("H Min",   WINDOW_MASK, 0,   179, lambda v: None)
        cv2.createTrackbar("H Max",   WINDOW_MASK, 179, 179, lambda v: None)
        cv2.createTrackbar("S Min",   WINDOW_MASK, 0,   255, lambda v: None)
        cv2.createTrackbar("S Max",   WINDOW_MASK, 60,  255, lambda v: None)
        cv2.createTrackbar("V Min",   WINDOW_MASK, 170, 255, lambda v: None)
        cv2.createTrackbar("V Max",   WINDOW_MASK, 255, 255, lambda v: None)
        cv2.createTrackbar("Use HSV", WINDOW_MASK, 0,   1,   lambda v: None)

        # ── Display toggles ───────────────────────────────────────────────────
        cv2.createTrackbar("Show Thresh",  WINDOW_MASK,   1, 1, lambda v: None)
        cv2.createTrackbar("Show Heatmap", WINDOW_SCORES, 1, 1, lambda v: None)

    def _read_sliders(self):
        return {
            'win':      max(20, cv2.getTrackbarPos("Win Size",    WINDOW_MAIN)),
            'step':     max(5,  cv2.getTrackbarPos("Step Size",   WINDOW_MAIN)),
            'min_score':     cv2.getTrackbarPos("Min Score",   WINDOW_MAIN),
            'top_n':    max(1,  cv2.getTrackbarPos("Top-N Merge", WINDOW_MAIN)),
            'h_min':         cv2.getTrackbarPos("H Min",   WINDOW_MASK),
            'h_max':         cv2.getTrackbarPos("H Max",   WINDOW_MASK),
            's_min':         cv2.getTrackbarPos("S Min",   WINDOW_MASK),
            's_max':         cv2.getTrackbarPos("S Max",   WINDOW_MASK),
            'v_min':         cv2.getTrackbarPos("V Min",   WINDOW_MASK),
            'v_max':         cv2.getTrackbarPos("V Max",   WINDOW_MASK),
            'use_hsv':       cv2.getTrackbarPos("Use HSV", WINDOW_MASK),
            'show_thresh':   cv2.getTrackbarPos("Show Thresh",  WINDOW_MASK),
            'show_heatmap':  cv2.getTrackbarPos("Show Heatmap", WINDOW_SCORES),
        }

    # ── ROS callback ──────────────────────────────────────────────────────────

    def _image_cb(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f"Image convert error: {e}")

    # ── Main display / detection loop ─────────────────────────────────────────

    def _display_tick(self):
        if self.latest_frame is None:
            return

        frame = self.latest_frame.copy()
        p = self._read_sliders()
        h_frame, w_frame = frame.shape[:2]

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # ── Adaptive threshold ────────────────────────────────────────────────
        thresh = cv2.adaptiveThreshold(
            gray, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY, 11, 2
        )

        # ── Optional HSV mask ─────────────────────────────────────────────────
        hsv_mask = cv2.inRange(
            hsv,
            (p['h_min'], p['s_min'], p['v_min']),
            (p['h_max'], p['s_max'], p['v_max'])
        )
        if p['use_hsv']:
            thresh = cv2.bitwise_and(thresh, hsv_mask)

        # ── Sliding window transition scorer ─────────────────────────────────
        win  = p['win']
        step = p['step']

        cols = list(range(0, w_frame - win, step))
        rows = list(range(0, h_frame - win, step))

        score_map = np.zeros((len(rows), len(cols)), dtype=np.float32)

        for ri, y in enumerate(rows):
            for ci, x in enumerate(cols):
                patch = thresh[y:y+win, x:x+win].astype(np.int16)
                h_trans = np.sum(np.abs(np.diff(patch, axis=1)) > 128)
                v_trans = np.sum(np.abs(np.diff(patch, axis=0)) > 128)
                score_map[ri, ci] = h_trans + v_trans

        # ── Find top-N windows and merge into centroid ────────────────────────
        detection_centroid = None
        flat_scores = score_map.flatten()
        top_n = min(p['top_n'], len(flat_scores))
        top_indices = np.argpartition(flat_scores, -top_n)[-top_n:]

        valid_patches = []
        for idx in top_indices:
            ri = idx // len(cols)
            ci = idx % len(cols)
            score = score_map[ri, ci]
            if score >= p['min_score']:
                px = cols[ci] + win // 2
                py = rows[ri] + win // 2
                valid_patches.append((px, py, score))

        if valid_patches:
            total_w = sum(s for _, _, s in valid_patches)
            cx = sum(x * s for x, _, s in valid_patches) / total_w
            cy = sum(y * s for _, y, s in valid_patches) / total_w
            detection_centroid = (int(cx), int(cy))

        # ── Build heatmap display ─────────────────────────────────────────────
        if p['show_heatmap']:
            norm_scores = cv2.normalize(score_map, None, 0, 255, cv2.NORM_MINMAX)
            heatmap_small = norm_scores.astype(np.uint8)
            heatmap_color = cv2.applyColorMap(heatmap_small, cv2.COLORMAP_JET)
            heatmap_full  = cv2.resize(heatmap_color, (w_frame, h_frame),
                                       interpolation=cv2.INTER_NEAREST)
            cv2.imshow(WINDOW_SCORES, heatmap_full)
        else:
            blank = np.zeros((100, 400, 3), dtype=np.uint8)
            cv2.putText(blank, "Heatmap OFF", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 100, 100), 2)
            cv2.imshow(WINDOW_SCORES, blank)

        # ── Build mask display ────────────────────────────────────────────────
        if p['show_thresh']:
            mask_display = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
        else:
            mask_display = np.zeros_like(frame)
            cv2.putText(mask_display, "Thresh/Mask OFF", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 100, 100), 2)

        if p['use_hsv']:
            # Tint HSV mask region green on mask display
            hsv_overlay = cv2.cvtColor(hsv_mask, cv2.COLOR_GRAY2BGR)
            hsv_overlay[:, :, 0] = 0  # zero out blue and red channels
            hsv_overlay[:, :, 2] = 0
            mask_display = cv2.addWeighted(mask_display, 0.7, hsv_overlay, 0.3, 0)

        cv2.imshow(WINDOW_MASK, mask_display)

        # ── Draw detection on main frame ──────────────────────────────────────
        # Draw all valid scored windows
        for px, py, score in valid_patches:
            half = win // 2
            alpha_val = min(1.0, score / (p['min_score'] * 2))
            color = (0, int(255 * alpha_val), int(255 * (1 - alpha_val)))
            cv2.rectangle(frame,
                          (px - half, py - half),
                          (px + half, py + half),
                          color, 1)
            cv2.putText(frame, f"{int(score)}", (px - half, py - half - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)

        # Draw final merged centroid
        if detection_centroid:
            cx, cy = detection_centroid
            # Crosshair
            cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (0, 255, 0), 2)
            cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 30, (0, 255, 0), 2)
            cv2.putText(frame, f"CUBE ({cx}, {cy})",
                        (cx + 35, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Pixel error from image center (useful for head control tuning)
            err_x = cx - w_frame // 2
            err_y = cy - h_frame // 2
            cv2.putText(frame, f"err x:{err_x:+d} y:{err_y:+d}",
                        (cx + 35, cy + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 255), 1)
        else:
            cv2.putText(frame, "NO DETECTION", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        # Draw image center crosshair
        cv2.line(frame, (w_frame//2 - 10, h_frame//2),
                         (w_frame//2 + 10, h_frame//2), (255, 255, 0), 1)
        cv2.line(frame, (w_frame//2, h_frame//2 - 10),
                         (w_frame//2, h_frame//2 + 10), (255, 255, 0), 1)

        # Stats overlay
        n_valid = len(valid_patches)
        top_score = max((s for _, _, s in valid_patches), default=0)
        cv2.putText(frame, f"Valid patches: {n_valid}  Top score: {int(top_score)}",
                    (10, h_frame - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        cv2.imshow(WINDOW_MAIN, frame)
        cv2.waitKey(1)

    # ── ROS callback ──────────────────────────────────────────────────────────

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = NavCamTunerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()