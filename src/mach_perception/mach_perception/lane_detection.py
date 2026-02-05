import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge


class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__("lane_detection_node")

        # Parameters
        self.declare_parameter("image_topic", "/camera/left/image_raw")
        self.declare_parameter("lane_topic", "/lane_center")
        self.declare_parameter("conf_topic", "/lane_center/confidence")

        self.declare_parameter("roi_start_pct", 0.55)
        self.declare_parameter("row_step", 5)
        self.declare_parameter("min_pixels_row", 20)
        self.declare_parameter("smoothing", 0.7)

        self.declare_parameter("real_lane_width_cm", 32.0)
        self.declare_parameter("width_tol_frac", 0.35)

        # Expected distance between left and right lane markings in pixels
        # To find the distance: get right_lane_x and left_lane_x
        # Calculate lane_width = left_lane_x - right_lane_x 
        # lane_w_min_px = 0.6 * lane_width
        # lane_w_max_px = 1.4 * lane_width
        self.declare_parameter("lane_w_min_px", 240)
        self.declare_parameter("lane_w_max_px", 560)
        self.declare_parameter("lane_center_bias", 0.0)

        # Search tolerance around expected left/right in locked mode
        self.declare_parameter("edge_tol_frac", 0.4)

        # Output format for controller
        self.declare_parameter("publish_normalized", True)

        # Debug
        self.declare_parameter("show_debug", True)

        # Morphology to reduce double edges
        self.declare_parameter("enable_close", True)
        self.declare_parameter("close_kernel", 7)

        # Relock behavior if confidence stays low
        self.declare_parameter("relock_conf_thresh", 0.2)
        self.declare_parameter("relock_frames", 15)  # consecutive bad frames before reset

        # ROS setup
        self.image_topic = self.get_parameter("image_topic").value
        self.lane_topic = self.get_parameter("lane_topic").value
        self.conf_topic = self.get_parameter("conf_topic").value

        self.bridge = CvBridge()

        self.lane_pub = self.create_publisher(Float32, self.lane_topic, 10)
        self.conf_pub = self.create_publisher(Float32, self.conf_topic, 10)

        self.sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)

        # State variables
        self.prev_center = None
        self.base_center_px = None
        self.base_half_width_px = None
        self.px_per_cm = None

        self.bad_lock_ctr = 0

        self.get_logger().info(f"Subscribed: {self.image_topic}")
        self.get_logger().info(f"Publishing: {self.lane_topic} (Float32), {self.conf_topic} (Float32)")


    # Algorithm functions
    def preprocess(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        th = cv2.adaptiveThreshold(
            gray, 255,
            cv2.ADAPTIVE_THRESH_MEAN_C,
            cv2.THRESH_BINARY_INV,
            11, 5
        )

        # Fill edges to avoid double-edge stripes
        if bool(self.get_parameter("enable_close").value):
            k = int(self.get_parameter("close_kernel").value)
            k = max(3, k)
            if k % 2 == 0:
                k += 1
            kernel_close = np.ones((k, k), np.uint8)
            th = cv2.morphologyEx(th, cv2.MORPH_CLOSE, kernel_close)

        # Clean small noise
        kernel_open = np.ones((3, 3), np.uint8)
        th = cv2.morphologyEx(th, cv2.MORPH_OPEN, kernel_open)

        return th

    def find_center(self, binary, row_step, min_pixels_row, lane_w_min_px, lane_w_max_px):
        """
        Unlocked mode: choose best pair of segments per row based on lane width range.
        """
        h, w = binary.shape
        centers = []
        centers_with_rows = []

        expected_sep = 0.5 * (lane_w_min_px + lane_w_max_px)

        for y in range(h - 1, int(h * 0.6), -row_step):
            row = binary[y]
            xs = np.where(row > 0)[0]
            if xs.size < min_pixels_row:
                continue

            segs = self._row_segments(xs, gap_thresh=3)
            segs = [s for s in segs if s["width"] >= 6]
            if len(segs) < 2:
                continue

            best_pair = None
            best_score = 1e9
            for i in range(len(segs)):
                for j in range(i + 1, len(segs)):
                    left_mid = segs[i]["mid"]
                    right_mid = segs[j]["mid"]
                    sep = right_mid - left_mid
                    if not (lane_w_min_px <= sep <= lane_w_max_px):
                        continue
                    score = abs(sep - expected_sep)
                    if score < best_score:
                        best_score = score
                        best_pair = (left_mid, right_mid)

            if best_pair is None:
                continue

            left_mid, right_mid = best_pair
            c = int((left_mid + right_mid) // 2)
            centers.append(c)
            centers_with_rows.append((c, y))

        if len(centers) == 0:
            return None, []
        return int(np.mean(centers)), centers_with_rows

    def _row_segments(self, xs: np.ndarray, gap_thresh: int = 3):
        """
        Convert white pixel indices into contiguous segments (runs).
        Returns list of dicts: {"start","end","width","mid"}
        """
        if xs.size == 0:
            return []

        cuts = np.where(np.diff(xs) > gap_thresh)[0]
        starts = np.r_[xs[0], xs[cuts + 1]]
        ends = np.r_[xs[cuts], xs[-1]]

        segs = []
        for s, e in zip(starts, ends):
            width = int(e - s + 1)
            mid = int((s + e) // 2)
            segs.append({"start": int(s), "end": int(e), "width": width, "mid": mid})
        return segs

    def find_center_locked(self, binary, base_center, base_half_width, row_step, min_pixels_row,
                           edge_tol_frac, real_lane_width_cm, width_tol_frac, px_per_cm,
                           lane_w_min_px, lane_w_max_px,
                           gap_thresh: int = 3, min_seg_width: int = 6):
        """
        Locked mode: find segments near expected left/right, then enforce lane width separation.
        """
        h, w = binary.shape
        centers = []
        all_centers = []

        exp_left = int(base_center - base_half_width)
        exp_right = int(base_center + base_half_width)
        tol = int(base_half_width * edge_tol_frac)

        y_bottom = h - 1
        y_top = int(h * 0.6)

        for y in range(y_bottom, y_top, -row_step):
            row = binary[y]
            xs = np.where(row > 0)[0]
            if xs.size < min_pixels_row:
                continue

            segs = self._row_segments(xs, gap_thresh=gap_thresh)
            segs = [s for s in segs if s["width"] >= min_seg_width]
            if not segs:
                continue

            left_segs = [s for s in segs if (exp_left - tol) <= s["mid"] <= (exp_left + tol)]
            right_segs = [s for s in segs if (exp_right - tol) <= s["mid"] <= (exp_right + tol)]
            if not left_segs or not right_segs:
                continue

            left_seg = min(left_segs, key=lambda s: abs(s["mid"] - exp_left))
            right_seg = min(right_segs, key=lambda s: abs(s["mid"] - exp_right))

            left = int(left_seg["mid"])
            right = int(right_seg["mid"])
            if right <= left:
                continue

            # Enforce plausible lane width in pixels
            sep = right - left
            if not (lane_w_min_px <= sep <= lane_w_max_px):
                continue

            # Real-width check (only if px_per_cm is valid)
            if px_per_cm is not None and px_per_cm > 1e-6:
                width_cm = sep / px_per_cm
                if not (real_lane_width_cm * (1 - width_tol_frac) <= width_cm <= real_lane_width_cm * (1 + width_tol_frac)):
                    continue

            c = int((left + right) // 2)
            centers.append(c)
            all_centers.append((c, y))

        if len(centers) == 0:
            return None, []
        return int(np.mean(centers)), all_centers


    # ROS callback
    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        if frame is None or frame.size == 0:
            return

        roi_start_pct = float(self.get_parameter("roi_start_pct").value)
        row_step = int(self.get_parameter("row_step").value)
        min_pixels_row = int(self.get_parameter("min_pixels_row").value)
        smoothing = float(self.get_parameter("smoothing").value)

        real_lane_width_cm = float(self.get_parameter("real_lane_width_cm").value)
        width_tol_frac = float(self.get_parameter("width_tol_frac").value)

        lane_w_min_px = int(self.get_parameter("lane_w_min_px").value)
        lane_w_max_px = int(self.get_parameter("lane_w_max_px").value)
        lane_center_bias = float(self.get_parameter("lane_center_bias").value)
        edge_tol_frac = float(self.get_parameter("edge_tol_frac").value)

        publish_normalized = bool(self.get_parameter("publish_normalized").value)
        show_debug = bool(self.get_parameter("show_debug").value)

        relock_conf_thresh = float(self.get_parameter("relock_conf_thresh").value)
        relock_frames = int(self.get_parameter("relock_frames").value)

        h, w = frame.shape[:2]
        roi_start = int(h * roi_start_pct)
        roi = frame[roi_start:, :]

        binary = self.preprocess(roi)

        # Establish fixed lane using SEGMENTS + PAIR SELECTION across bottom rows
        if self.base_center_px is None:
            h_roi, w_roi = binary.shape
            best_pair = None
            best_score = 1e9
            expected_sep = 0.5 * (lane_w_min_px + lane_w_max_px)

            y_start = h_roi - 1
            y_end = max(int(h_roi * 0.85), 0)

            for y in range(y_start, y_end, -1):
                xs = np.where(binary[y] > 0)[0]
                if xs.size < min_pixels_row:
                    continue

                segs = self._row_segments(xs, gap_thresh=3)
                segs = [s for s in segs if s["width"] >= 6]
                if len(segs) < 2:
                    continue

                for i in range(len(segs)):
                    for j in range(i + 1, len(segs)):
                        left_mid = segs[i]["mid"]
                        right_mid = segs[j]["mid"]
                        sep = right_mid - left_mid
                        if not (lane_w_min_px <= sep <= lane_w_max_px):
                            continue

                        score = abs(sep - expected_sep)
                        if score < best_score:
                            best_score = score
                            best_pair = (left_mid, right_mid)

            if best_pair is not None:
                left_mid, right_mid = best_pair
                self.base_center_px = int((left_mid + right_mid) // 2)
                self.base_half_width_px = float((right_mid - left_mid) / 2.0)
                self.px_per_cm = float((right_mid - left_mid) / real_lane_width_cm)

        # Center detection
        if self.base_center_px is not None and self.base_half_width_px is not None:
            center, centers_with_rows = self.find_center_locked(
                binary,
                self.base_center_px,
                self.base_half_width_px,
                row_step=row_step,
                min_pixels_row=min_pixels_row,
                edge_tol_frac=edge_tol_frac,
                real_lane_width_cm=real_lane_width_cm,
                width_tol_frac=width_tol_frac,
                px_per_cm=self.px_per_cm,
                lane_w_min_px=lane_w_min_px,
                lane_w_max_px=lane_w_max_px,
            )
        else:
            center, centers_with_rows = self.find_center(
                binary,
                row_step=row_step,
                min_pixels_row=min_pixels_row,
                lane_w_min_px=lane_w_min_px,
                lane_w_max_px=lane_w_max_px,
            )

        # Temporal smoothing
        if center is not None:
            if self.prev_center is None:
                smooth_center = center
            else:
                smooth_center = int(smoothing * self.prev_center + (1.0 - smoothing) * center)
            self.prev_center = smooth_center
        else:
            smooth_center = self.prev_center if self.prev_center is not None else (w // 2)

        # Offset relative to image center (px)
        offset_px = float(smooth_center - (w / 2.0))

        # Publish normalized for controller
        if publish_normalized:
            denom = (w / 2.0) if w > 0 else 1.0
            offset_out = float(offset_px / denom)  # ~[-1, 1]
        else:
            offset_out = float(offset_px)

        self.get_logger().info(f"lane_center_bias={lane_center_bias:.6f} raw_norm={offset_out:.6f}")    
        offset_out = offset_out - lane_center_bias

        # Confidence: ratio of valid scan rows
        h_roi = binary.shape[0]
        y_bottom = h_roi - 1
        y_top = int(h_roi * 0.6)
        expected_rows = max(1, int((y_bottom - y_top) / max(1, row_step)))
        valid_rows = len(centers_with_rows)
        confidence = float(np.clip(valid_rows / expected_rows, 0.0, 1.0))

        # Relock if stuck in bad detection
        if confidence < relock_conf_thresh:
            self.bad_lock_ctr += 1
        else:
            self.bad_lock_ctr = 0

        if self.bad_lock_ctr >= relock_frames:
            self.base_center_px = None
            self.base_half_width_px = None
            self.px_per_cm = None
            self.bad_lock_ctr = 0

        # Publish lane_center + confidence
        lane_msg = Float32()
        lane_msg.data = offset_out
        self.lane_pub.publish(lane_msg)

        conf_msg = Float32()
        conf_msg.data = confidence
        self.conf_pub.publish(conf_msg)

        # Debug visualization
        if show_debug:
            visual = roi.copy()

            for (c, y) in centers_with_rows:
                if 0 <= y < visual.shape[0]:
                    cv2.circle(visual, (int(c), int(y)), 2, (0, 255, 0), -1)

            cv2.line(visual, (int(smooth_center), visual.shape[0]),
                     (int(smooth_center), max(0, visual.shape[0] - 80)), (0, 0, 255), 3)
            cv2.circle(visual, (int(smooth_center), max(0, visual.shape[0] - 40)), 8, (0, 0, 255), -1)

            cv2.putText(visual, f"offset: {offset_out:.3f}" + (" (norm)" if publish_normalized else " (px)"),
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            cv2.putText(visual, f"conf: {confidence:.2f} rows: {valid_rows}/{expected_rows}",
                        (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)

            # Draw expected left/right if locked
            if self.base_center_px is not None and self.base_half_width_px is not None:
                exp_left = int(self.base_center_px - self.base_half_width_px)
                exp_right = int(self.base_center_px + self.base_half_width_px)
                cv2.line(visual, (exp_left, visual.shape[0]), (exp_left, max(0, visual.shape[0] - 60)), (255, 0, 0), 2)
                cv2.line(visual, (exp_right, visual.shape[0]), (exp_right, max(0, visual.shape[0] - 60)), (255, 0, 0), 2)

            cv2.imshow("Lane Center", visual)
            cv2.imshow("Threshold", binary)
            cv2.waitKey(1)


def main():
    rclpy.init()
    node = LaneDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
