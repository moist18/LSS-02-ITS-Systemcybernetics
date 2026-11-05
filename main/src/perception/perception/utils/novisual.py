import time
import numpy as np
import cv2

class BBoxVisualization():
    def __init__(self, cls_dict):
        self.cls_dict = cls_dict  # ['blue_box', 'green_box', 'orange_buoy', 'green_dock', 'yellow_buoy', 'red_dock', 'black_buoy']

    # HSV colour-blob fallback — Kamikaze ONLY, and only when YOLO misses the
    # target this frame. See visual.py for the full reasoning.
    HSV_RANGES = {
        'green_buoy': ([35,  60,  40],  [85,  255, 255], None,           None          ),
        'red_buoy':   ([0,   100, 50],  [10,  255, 255], [160, 100, 50], [180, 255, 255]),
        'black_buoy': ([0,   0,   0],   [180, 80,  80],  None,           None          ),
    }

    _HSV_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    _last_hsv_debug = 0.0  # throttle state, shared class-wide — fine, single camera loop

    def _detect_by_hsv(self, frame, color_name, min_area):
        """Largest matching colour blob: (cx, cy, x1, y1, x2, y2), or None if
        nothing big enough. See visual.py for the bbox reasoning."""
        ranges = self.HSV_RANGES.get(color_name)
        if ranges is None:
            return None

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower1, upper1, lower2, upper2 = ranges
        mask = cv2.inRange(hsv, np.array(lower1), np.array(upper1))
        if lower2 is not None:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, np.array(lower2), np.array(upper2)))

        # Open (drop small noise specks) then close (fill small holes) — same as
        # teshsv.py; without it, stray noise pixels can outscore or split the
        # real blob's contour.
        cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._HSV_KERNEL, dst=mask)
        cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._HSV_KERNEL, dst=mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        now = time.time()
        debug_due = now - type(self)._last_hsv_debug >= 1.0
        if debug_due:
            type(self)._last_hsv_debug = now

        if not contours:
            if debug_due:
                print(f"[HSV] {color_name}: NO contours in mask — colour range doesn't match anything in frame", flush=True)
            return None

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < min_area:
            if debug_due:
                print(f"[HSV] {color_name}: largest contour {int(area)}px² < min_area {min_area}px² — found something, too small", flush=True)
            return None

        m = cv2.moments(largest)
        if m['m00'] == 0:
            return None
        cx, cy = int(m['m10'] / m['m00']), int(m['m01'] / m['m00'])
        x, y, w, h = cv2.boundingRect(largest)
        if debug_due:
            print(f"[HSV] {color_name}: HIT area={int(area)}px² at ({cx},{cy})", flush=True)
        return (cx, cy, x, y, x + w, y + h)

    def processor(self, frame, fps, mission, left_buoy, right_buoy, boxes, confs, clss, width, height, batas_bawah, batas_tengah, batas_atas, min_buoy_area, track_ids, velocities, distances_for_vis, orange_prefer_side='left', hsv_fallback_enabled=False, hsv_min_area=300):
        self.target_detected = False
        connector_midpoint = None
        yaw = batas_tengah

        max_area = {
            'blue_box': 0,
            'green_box': 0,
            'orange_buoy': 0,
            'green_dock': 0,
            'yellow_buoy': 0,
            'red_dock': 0,
            'black_buoy': 0
        }
        max_bbox = {
            'blue_box': None,
            'green_box': None,
            'orange_buoy': None,
            'green_dock': None,
            'yellow_buoy': None,
            'red_dock': None,
            'black_buoy': None
        }
        max_conf = {
            'blue_box': 0,
            'green_box': 0,
            'orange_buoy': 0,
            'green_dock': 0,
            'yellow_buoy': 0,
            'red_dock': 0,
            'black_buoy': 0
        }
        midpoints = {
            'blue_box': None,
            'green_box': None,
            'orange_buoy': None,
            'green_dock': None,
            'yellow_buoy': None,
            'red_dock': None,
            'black_buoy': None
        }

        # Detected object
        for bb, cf, cl in zip(boxes, confs, clss):
            cl = int(cl)
            x1, y1, x2, y2 = map(float, bb)
            cls_name = self.cls_dict.get(cl, 'CLS{}'.format(cl))
            midpoint = (int((x1 + x2) / 2), int((y1 + y2) / 2))
            area = (x2 - x1) * (y2 - y1)

            # Filter out objects with area below threshold (too far/small)
            if mission == "Floating Ball" or mission == "Kamikaze":
                if cls_name in ['orange_buoy', 'yellow_buoy', 'black_buoy'] and area < min_buoy_area:
                    continue

            if cls_name == 'orange_buoy':
                # Two orange buoys can be in frame at once (orange-yellow-orange
                # gate) — steer off the LEFT one only, ignore the right one outright.
                # See visual.py for the full reasoning. Which side survives is
                # picked by orange_prefer_side.
                keep_this = (
                    midpoints['orange_buoy'] is None or
                    (midpoint[0] > midpoints['orange_buoy'][0] if orange_prefer_side == 'right'
                     else midpoint[0] < midpoints['orange_buoy'][0])
                )
                if keep_this:
                    max_bbox['orange_buoy'] = (int(x1), int(y1), int(x2), int(y2))
                    max_conf['orange_buoy'] = cf
                    midpoints['orange_buoy'] = midpoint
            elif cls_name in max_area and area > max_area[cls_name]:
                max_area[cls_name] = area
                max_bbox[cls_name] = (int(x1), int(y1), int(x2), int(y2))
                max_conf[cls_name] = cf
                midpoints[cls_name] = midpoint

        # Connection line and midpoint
        if left_buoy in midpoints and right_buoy in midpoints and midpoints[left_buoy] and midpoints[right_buoy]:
            connector_midpoint = (int((midpoints[left_buoy][0] + midpoints[right_buoy][0]) / 2),
                                  int((midpoints[left_buoy][1] + midpoints[right_buoy][1]) / 2))

        # Mission Handler
        if mission == "Floating Ball":
            if connector_midpoint and left_buoy in midpoints and right_buoy in midpoints and left_buoy and right_buoy:
                a, b = connector_midpoint
                yaw = batas_bawah + a * ((batas_atas - batas_bawah) * (1/640))

            elif left_buoy in midpoints and midpoints[left_buoy] is not None:
                a, b = midpoints[left_buoy]
                yaw = batas_tengah + a * ((batas_tengah - batas_bawah) * (1/640))

            elif right_buoy in midpoints and midpoints[right_buoy] is not None:
                a, b = midpoints[right_buoy]
                yaw = batas_bawah + a * ((batas_tengah - batas_bawah) * (1/640))

        elif mission == "Kamikaze":
            target_buoy = left_buoy
            hsv_hit = self._detect_by_hsv(frame, target_buoy, hsv_min_area)
            if hsv_hit is not None:
                self.target_detected = True
                a, b = hsv_hit[0], hsv_hit[1]
                yaw = batas_bawah + a * ((batas_atas - batas_bawah) * (1 / 640))

        return frame, yaw
