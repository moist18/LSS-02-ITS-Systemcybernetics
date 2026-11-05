# import numpy as np
# import cv2

# class BBoxVisualization():
#     def __init__(self, cls_dict):
#         self.cls_dict = cls_dict  # ['blue_box', 'green_box', 'green_buoy', 'green_dock', 'red_buoy', 'red_dock', 'black_buoy', 'yellow_buoy', 'orange_buoy']

#         # ── PGF (Potential Gap Finding) parameters ────────────────────────────
#         self.pgf_r_safe          = 1.0    # safety radius (meters)
#         self.pgf_focal_length_x  = 320.0  # focal length X (pixels)
#         self.pgf_fov_rad         = np.deg2rad(90.0)  # 90 horizontal FoV
#         self.pgf_emergency_turn  = 200    # RC units away from mid
#         self.pgf_buoy_diameter_m = 0.20   # buoy real diameter (meters)

#     # ── PGF helpers ──────────────────────────────────────────────────────────

#     def estimate_distance_buoy(self, bbox_width_px):
#         if bbox_width_px < 2:
#             return float('inf')
#         return (self.pgf_buoy_diameter_m * self.pgf_focal_length_x) / bbox_width_px

#     def _pixel_to_angle(self, cx, image_width):
#         return (cx - image_width / 2.0) * self.pgf_fov_rad / image_width

#     def _merge_intervals(self, intervals):
#         if not intervals:
#             return []
#         merged = [intervals[0]]
#         for lo, hi in intervals[1:]:
#             if lo <= merged[-1][1]:
#                 merged[-1] = (merged[-1][0], max(merged[-1][1], hi))
#             else:
#                 merged.append((lo, hi))
#         return merged

#     def _find_largest_gap(self, blocked, lo_limit, hi_limit):
#         candidates = []
#         prev_end = lo_limit
#         for lo, hi in blocked:
#             lo_clipped = max(lo, lo_limit)
#             hi_clipped = min(hi, hi_limit)
#             if lo_clipped > prev_end:
#                 candidates.append((prev_end, lo_clipped))
#             prev_end = max(prev_end, hi_clipped)
#         if prev_end < hi_limit:
#             candidates.append((prev_end, hi_limit))
#         if not candidates:
#             return (lo_limit, hi_limit)
#         return max(candidates, key=lambda seg: seg[1] - seg[0])

#     def _find_all_free_gaps(self, blocked, lo_limit, hi_limit):
#         candidates = []
#         prev_end = lo_limit
#         for lo, hi in blocked:
#             lo_clipped = max(lo, lo_limit)
#             hi_clipped = min(hi, hi_limit)
#             if lo_clipped > prev_end:
#                 candidates.append((prev_end, lo_clipped))
#             prev_end = max(prev_end, hi_clipped)
#         if prev_end < hi_limit:
#             candidates.append((prev_end, hi_limit))
#         return candidates if candidates else [(lo_limit, hi_limit)]

#     def _score_gap(self, gap, buoy_angles, image_width, velocities=None):
#         gap_lo, gap_hi = gap
#         gap_center_rad = (gap_lo + gap_hi) / 2.0
#         gap_width_rad = gap_hi - gap_lo
#         gap_width_px  = gap_width_rad / self.pgf_fov_rad * image_width

#         flank_dists = []
#         for angle, dist, _, _ in buoy_angles:
#             if dist < float('inf'):
#                 flank_dists.append(dist)
#         avg_dist = np.mean(flank_dists) if flank_dists else 5.0

#         risk = 0.0
#         if buoy_angles:
#             min_angular_dist = min(
#                 abs(angle - gap_center_rad) for angle, _, _, _ in buoy_angles
#             )
#             risk = max(0.0, 100.0 * (1.0 - min_angular_dist / (self.pgf_fov_rad / 2.0)))

#         score = 0.5 * gap_width_px + 0.4 * avg_dist - 0.1 * risk
#         return score, gap_width_px, avg_dist, risk

#     def get_safe_heading(self, detections, image_width, mid_rc, batas_bawah, batas_atas,
#                          velocities=None):
#         buoy_angles = []

#         for det in detections:
#             bbox_w = det['x2'] - det['x1']
#             cx     = (det['x1'] + det['x2']) / 2.0
#             dist   = self.estimate_distance_buoy(bbox_w)

#             if dist == float('inf') or dist > 8.0:
#                 continue

#             angle = self._pixel_to_angle(cx, image_width)
#             buoy_angles.append((angle, dist, bbox_w, cx))

#         if not buoy_angles:
#             return mid_rc, "PGF: NO BUOY (straight)", {}

#         for angle, dist, _, _ in buoy_angles:
#             if dist < self.pgf_r_safe:
#                 emergency_yaw = mid_rc + (self.pgf_emergency_turn if angle < 0 else -self.pgf_emergency_turn)
#                 emergency_yaw = int(np.clip(emergency_yaw, batas_bawah, batas_atas))
#                 return emergency_yaw, "PGF: EMERGENCY TURN", {'emergency': True}

#         intervals = []
#         for angle, dist, _, _ in buoy_angles:
#             half_block = np.arcsin(np.clip(self.pgf_r_safe / dist, -1.0, 1.0))
#             intervals.append((angle - half_block, angle + half_block))

#         intervals.sort()
#         merged = self._merge_intervals(intervals)

#         lo_limit = -self.pgf_fov_rad / 2.0
#         hi_limit =  self.pgf_fov_rad / 2.0

#         all_gaps = self._find_all_free_gaps(merged, lo_limit, hi_limit)

#         scored_gaps = []
#         for gap in all_gaps:
#             score, gw_px, avg_d, risk = self._score_gap(gap, buoy_angles, image_width, velocities)
#             scored_gaps.append({
#                 'gap': gap,
#                 'score': score,
#                 'width_px': gw_px,
#                 'distance': avg_d,
#                 'risk': risk
#             })

#         scored_gaps.sort(key=lambda g: g['score'], reverse=True)
#         best = scored_gaps[0]
#         best_gap   = best['gap']
#         target_ang = (best_gap[0] + best_gap[1]) / 2.0

#         fov_half = self.pgf_fov_rad / 2.0
#         rc_yaw = batas_bawah + (target_ang + fov_half) / self.pgf_fov_rad * (batas_atas - batas_bawah)
#         rc_yaw = int(np.clip(rc_yaw, batas_bawah, batas_atas))

#         gap_width_deg = np.degrees(best_gap[1] - best_gap[0])
#         return rc_yaw, f"PGF: gap {gap_width_deg:.0f} s={best['score']:.0f}", {}

#     # ── Main processor ───────────────────────────────────────────────────────

#     def processor(self, frame, fps, mission, left_buoy, right_buoy,
#                   boxes, confs, clss, width, height,
#                   batas_bawah, batas_tengah, batas_atas,
#                   min_buoy_area):
#         connector_midpoint = None
#         yaw = batas_tengah

#         max_area = {k: 0    for k in ['blue_box','green_box','green_buoy','green_dock','red_buoy','red_dock','black_buoy','yellow_buoy','orange_buoy']}
#         max_bbox = {k: None for k in ['blue_box','green_box','green_buoy','green_dock','red_buoy','red_dock','black_buoy','yellow_buoy','orange_buoy']}
#         max_conf = {k: 0    for k in ['blue_box','green_box','green_buoy','green_dock','red_buoy','red_dock','black_buoy','yellow_buoy','orange_buoy']}
#         midpoints = {k: None for k in ['blue_box','green_box','green_buoy','green_dock','red_buoy','red_dock','black_buoy','yellow_buoy','orange_buoy']}

#         buoy_detections_pgf = []

#         for bb, cf, cl in zip(boxes, confs, clss):
#             cl = int(cl)
#             x1, y1, x2, y2 = map(float, bb)
#             cls_name = self.cls_dict.get(cl, f'CLS{cl}')
#             midpoint = (int((x1 + x2) / 2), int((y1 + y2) / 2))
#             area = (x2 - x1) * (y2 - y1)

#             if mission == "Floating Ball" or mission == "Kamikaze":
#                 if cls_name in ['green_buoy', 'red_buoy', 'black_buoy', 'yellow_buoy', 'orange_buoy']:
#                     buoy_detections_pgf.append({
#                         'cls_name': cls_name,
#                         'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
#                         'conf': cf
#                     })
#                 if cls_name in ['green_buoy', 'red_buoy', 'black_buoy', 'yellow_buoy', 'orange_buoy'] and area < min_buoy_area:
#                     continue

#             if cls_name in max_area and area > max_area[cls_name]:
#                 max_area[cls_name] = area
#                 max_bbox[cls_name] = (int(x1), int(y1), int(x2), int(y2))
#                 max_conf[cls_name] = cf
#                 midpoints[cls_name] = midpoint

#         # Connector midpoint
#         if left_buoy in midpoints and right_buoy in midpoints and midpoints[left_buoy] and midpoints[right_buoy]:
#             connector_midpoint = (
#                 int((midpoints[left_buoy][0] + midpoints[right_buoy][0]) / 2),
#                 int((midpoints[left_buoy][1] + midpoints[right_buoy][1]) / 2)
#             )

#         # Mission handlers
#         if mission == "Floating Ball":
#             if buoy_detections_pgf:
#                 pgf_yaw, pgf_status, pgf_debug = self.get_safe_heading(
#                     buoy_detections_pgf, width, batas_tengah, batas_bawah, batas_atas
#                 )
#                 yaw = pgf_yaw
#             else:
#                 yaw = batas_tengah

#         elif mission == "Kamikaze":
#             target_buoy = left_buoy
#             # Fallback if target is black_buoy but only blue_box is detected
#             if target_buoy == "black_buoy" and (midpoints.get("black_buoy") is None) and (midpoints.get("blue_box") is not None):
#                 target_buoy = "blue_box"

#             if target_buoy in midpoints and midpoints[target_buoy] is not None:
#                 a, b = midpoints[target_buoy]
#                 yaw = batas_bawah + a * ((batas_atas - batas_bawah) * (1 / 640))

#         return frame, yaw


import numpy as np
import cv2


class BBoxVisualization():
    def __init__(self, cls_dict):
        self.cls_dict = cls_dict  # ['blue_box','green_box','green_buoy','green_dock','red_buoy','red_dock','black_buoy','yellow_buoy','orange_buoy']

        # ── PGF (Potential Gap Finding) parameters ────────────────────────────
        self.pgf_r_safe          = 1.0    # safety radius (meters)
        self.pgf_focal_length_x  = 320.0  # focal length X (px) -- fallback only
        self.pgf_fov_rad         = np.deg2rad(87.0)  # D455 depth H-FoV ~87 deg
        self.pgf_emergency_turn  = 200    # RC units away from mid
        self.pgf_buoy_diameter_m = 0.20   # buoy real diameter (m) -- fallback only
        self.pgf_max_range       = 8.0    # ignore buoys farther than this (m)

        # Score penalty per side of a gap NOT flanked by an actual buoy
        # (i.e. that side is just the outer FOV edge) — keeps PGF from
        # chasing "wide open" unmarked water over a real buoy-to-buoy gate.
        self.pgf_unbounded_gap_penalty = 400.0   # score units per unbounded side

        # Mild score penalty per degree a gap's centre deviates from
        # straight-ahead — tie-breaker toward forward-centred gaps.
        self.pgf_heading_bias_weight   = 2.0     # score units per degree

        # ── Depth camera (RealSense D455) ─────────────────────────────────────
        self.depth_intrinsics    = None   # set via set_depth_intrinsics()
        self.depth_patch_half    = 4      # half-size of median window for depth sampling
        self.depth_default_scale = 0.001  # meters per depth unit (D455 default ~0.001)

        # ── Optional: generic obstacle blocking from raw depth map ────────────
        # WARNING: experimental on water. Water surface / sun glare can produce
        # noisy depth. Keep OFF until validated in your environment.
        self.pgf_use_depth_obstacles = False
        self.pgf_obstacle_range      = 3.0   # (m) columns closer than this get blocked
        self.pgf_obstacle_cols       = 64    # number of angular columns to scan

    # ── Depth camera setup ────────────────────────────────────────────────────

    def set_depth_intrinsics(self, intrinsics):
        """Call once after starting the RealSense pipeline.
        intrinsics: dict {'fx','fy','ppx','ppy'} from the ALIGNED depth stream
        (aligned to color, since YOLO bboxes come from the color frame)."""
        self.depth_intrinsics = intrinsics

    def _sample_depth(self, depth_image, cx, cy, depth_scale):
        """Return median depth (meters) in a small window around (cx, cy),
        ignoring invalid (zero) pixels. None if no valid depth there."""
        if depth_image is None:
            return None
        h, w = depth_image.shape[:2]
        cx = int(round(cx)); cy = int(round(cy))
        half = self.depth_patch_half
        x0 = max(0, cx - half); x1 = min(w, cx + half + 1)
        y0 = max(0, cy - half); y1 = min(h, cy + half + 1)
        if x1 <= x0 or y1 <= y0:
            return None
        patch = depth_image[y0:y1, x0:x1].astype(np.float32)
        valid = patch[patch > 0]
        if valid.size == 0:
            return None
        return float(np.median(valid)) * depth_scale

    # ── PGF helpers ──────────────────────────────────────────────────────────

    def _estimate_distance_from_bbox(self, bbox_width_px):
        """Fallback used ONLY when depth is unavailable at the buoy pixel."""
        if bbox_width_px < 2:
            return float('inf')
        return (self.pgf_buoy_diameter_m * self.pgf_focal_length_x) / bbox_width_px

    def _pixel_to_angle(self, cx, image_width):
        # Prefer true intrinsics (accurate, non-linear). Fall back to FoV model.
        if self.depth_intrinsics is not None:
            fx  = self.depth_intrinsics['fx']
            ppx = self.depth_intrinsics['ppx']
            return float(np.arctan((cx - ppx) / fx))
        return (cx - image_width / 2.0) * self.pgf_fov_rad / image_width

    def _angle_limits(self, image_width):
        if self.depth_intrinsics is not None:
            fx  = self.depth_intrinsics['fx']
            ppx = self.depth_intrinsics['ppx']
            lo = float(np.arctan((0.0 - ppx) / fx))
            hi = float(np.arctan((image_width - ppx) / fx))
            return lo, hi
        return -self.pgf_fov_rad / 2.0, self.pgf_fov_rad / 2.0

    def _merge_intervals(self, intervals):
        if not intervals:
            return []
        intervals = sorted(intervals)
        merged = [intervals[0]]
        for lo, hi in intervals[1:]:
            if lo <= merged[-1][1]:
                merged[-1] = (merged[-1][0], max(merged[-1][1], hi))
            else:
                merged.append((lo, hi))
        return merged

    def _find_largest_gap(self, blocked, lo_limit, hi_limit):
        candidates = []
        prev_end = lo_limit
        for lo, hi in blocked:
            lo_clipped = max(lo, lo_limit)
            hi_clipped = min(hi, hi_limit)
            if lo_clipped > prev_end:
                candidates.append((prev_end, lo_clipped))
            prev_end = max(prev_end, hi_clipped)
        if prev_end < hi_limit:
            candidates.append((prev_end, hi_limit))
        if not candidates:
            return (lo_limit, hi_limit)
        return max(candidates, key=lambda seg: seg[1] - seg[0])

    def _find_all_free_gaps(self, blocked, lo_limit, hi_limit):
        """Returns list of (gap_lo, gap_hi, bounded_lo, bounded_hi). bounded_*
        is False when that side is just the outer FOV edge, not an actual
        buoy — i.e. the gap is only "open" because nothing was detected
        there (could be open water / outside the arena)."""
        candidates = []
        prev_end = lo_limit
        for lo, hi in blocked:
            lo_clipped = max(lo, lo_limit)
            hi_clipped = min(hi, hi_limit)
            if lo_clipped > prev_end:
                bounded_lo = prev_end > lo_limit
                candidates.append((prev_end, lo_clipped, bounded_lo, True))
            prev_end = max(prev_end, hi_clipped)
        if prev_end < hi_limit:
            bounded_lo = prev_end > lo_limit
            candidates.append((prev_end, hi_limit, bounded_lo, False))
        return candidates if candidates else [(lo_limit, hi_limit, False, False)]

    def _score_gap(self, gap, buoy_angles, image_width, fov_span, velocities=None):
        gap_lo, gap_hi, bounded_lo, bounded_hi = gap
        gap_center_rad = (gap_lo + gap_hi) / 2.0
        gap_width_rad  = gap_hi - gap_lo
        gap_width_px   = gap_width_rad / fov_span * image_width

        flank_dists = [dist for _, dist, _, _ in buoy_angles if dist < float('inf')]
        avg_dist = np.mean(flank_dists) if flank_dists else 5.0

        risk = 0.0
        if buoy_angles:
            min_angular_dist = min(abs(a - gap_center_rad) for a, _, _, _ in buoy_angles)
            risk = max(0.0, 100.0 * (1.0 - min_angular_dist / (fov_span / 2.0)))

        unbounded_penalty = 0.0
        if not bounded_lo:
            unbounded_penalty += self.pgf_unbounded_gap_penalty
        if not bounded_hi:
            unbounded_penalty += self.pgf_unbounded_gap_penalty

        heading_bias = self.pgf_heading_bias_weight * abs(np.degrees(gap_center_rad))

        score = 0.5 * gap_width_px + 0.4 * avg_dist - 0.1 * risk - unbounded_penalty - heading_bias
        return score, gap_width_px, avg_dist, risk

    def _depth_obstacle_intervals(self, depth_image, depth_scale, image_width):
        """OPTIONAL: turn the raw depth map into blocked angular intervals so the
        boat also avoids obstacles the YOLO model does NOT recognise.
        Scans a horizontal band into angular columns; a column closer than
        pgf_obstacle_range becomes a blocked interval."""
        if depth_image is None:
            return []
        h, w = depth_image.shape[:2]
        band = depth_image[int(h * 0.35):int(h * 0.75), :].astype(np.float32) * depth_scale
        band[band <= 0] = np.nan
        n = self.pgf_obstacle_cols
        col_w = max(1, w // n)
        intervals = []
        for i in range(n):
            col = band[:, i * col_w:(i + 1) * col_w]
            valid = col[~np.isnan(col)]
            if valid.size < 10:
                continue
            d = float(np.percentile(valid, 10))  # near portion of the column
            if d < self.pgf_obstacle_range:
                cx = (i + 0.5) * col_w
                ang = self._pixel_to_angle(cx, image_width)
                half = np.arcsin(np.clip(self.pgf_r_safe / max(d, 0.3), -1.0, 1.0))
                intervals.append((ang - half, ang + half))
        return intervals

    # ── Safe heading (PGF core) ───────────────────────────────────────────────

    def get_safe_heading(self, detections, image_width, mid_rc,
                         batas_bawah, batas_atas,
                         depth_image=None, depth_scale=None, velocities=None):
        if depth_scale is None:
            depth_scale = self.depth_default_scale

        buoy_angles = []
        for det in detections:
            cx     = (det['x1'] + det['x2']) / 2.0
            cy     = (det['y1'] + det['y2']) / 2.0
            bbox_w =  det['x2'] - det['x1']

            # 1) measured depth first, 2) bbox-size estimate only as fallback
            dist = self._sample_depth(depth_image, cx, cy, depth_scale)
            if dist is None or dist <= 0.0:
                dist = self._estimate_distance_from_bbox(bbox_w)

            if dist == float('inf') or dist > self.pgf_max_range:
                continue

            angle = self._pixel_to_angle(cx, image_width)
            buoy_angles.append((angle, dist, bbox_w, cx))

        if not buoy_angles:
            return mid_rc, "PGF: NO BUOY (straight)", {}

        # Emergency: something inside the safety radius -> hard turn away from it
        for angle, dist, _, _ in buoy_angles:
            if dist < self.pgf_r_safe:
                emergency_yaw = mid_rc + (self.pgf_emergency_turn if angle < 0 else -self.pgf_emergency_turn)
                emergency_yaw = int(np.clip(emergency_yaw, batas_bawah, batas_atas))
                return emergency_yaw, "PGF: EMERGENCY TURN", {'emergency': True}

        # Blocked intervals from buoys (wider block when closer)
        intervals = []
        for angle, dist, _, _ in buoy_angles:
            half_block = np.arcsin(np.clip(self.pgf_r_safe / dist, -1.0, 1.0))
            intervals.append((angle - half_block, angle + half_block))

        # Optional: add generic obstacles straight from the depth map
        if self.pgf_use_depth_obstacles:
            intervals += self._depth_obstacle_intervals(depth_image, depth_scale, image_width)

        merged = self._merge_intervals(intervals)

        lo_limit, hi_limit = self._angle_limits(image_width)
        fov_span = hi_limit - lo_limit

        all_gaps = self._find_all_free_gaps(merged, lo_limit, hi_limit)

        scored_gaps = []
        for gap in all_gaps:
            score, gw_px, avg_d, risk = self._score_gap(gap, buoy_angles, image_width, fov_span, velocities)
            scored_gaps.append({'gap': gap, 'score': score,
                                'width_px': gw_px, 'distance': avg_d, 'risk': risk})

        scored_gaps.sort(key=lambda g: g['score'], reverse=True)
        best       = scored_gaps[0]
        best_gap   = best['gap']
        target_ang = (best_gap[0] + best_gap[1]) / 2.0

        rc_yaw = batas_bawah + (target_ang - lo_limit) / fov_span * (batas_atas - batas_bawah)
        rc_yaw = int(np.clip(rc_yaw, batas_bawah, batas_atas))

        gap_width_deg = np.degrees(best_gap[1] - best_gap[0])
        return rc_yaw, f"PGF: gap {gap_width_deg:.0f} s={best['score']:.0f}", {}

    # ── Main processor ───────────────────────────────────────────────────────

    def processor(self, frame, fps, mission, left_buoy, right_buoy,
                  boxes, confs, clss, width, height,
                  batas_bawah, batas_tengah, batas_atas,
                  min_buoy_area,
                  track_ids=None, velocities=None, distances=None, course=2,
                  depth_image=None, depth_scale=None):
        connector_midpoint = None
        yaw = batas_tengah

        keys = ['blue_box','green_box','green_buoy','green_dock','red_buoy',
                'red_dock','black_buoy','yellow_buoy','orange_buoy']
        max_area  = {k: 0    for k in keys}
        max_bbox  = {k: None for k in keys}
        max_conf  = {k: 0    for k in keys}
        midpoints = {k: None for k in keys}

        buoy_detections_pgf = []

        for bb, cf, cl in zip(boxes, confs, clss):
            cl = int(cl)
            x1, y1, x2, y2 = map(float, bb)
            cls_name = self.cls_dict.get(cl, f'CLS{cl}')
            midpoint = (int((x1 + x2) / 2), int((y1 + y2) / 2))
            area = (x2 - x1) * (y2 - y1)

            if mission == "Floating Ball" or mission == "Kamikaze":
                if cls_name in ['green_buoy','red_buoy','black_buoy','yellow_buoy','orange_buoy']:
                    buoy_detections_pgf.append({'cls_name': cls_name,
                                                'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                                                'conf': cf})
                if cls_name in ['green_buoy','red_buoy','black_buoy','yellow_buoy','orange_buoy'] and area < min_buoy_area:
                    continue

            if cls_name in max_area and area > max_area[cls_name]:
                max_area[cls_name] = area
                max_bbox[cls_name] = (int(x1), int(y1), int(x2), int(y2))
                max_conf[cls_name] = cf
                midpoints[cls_name] = midpoint

        # Connector midpoint (kept for visualization)
        if left_buoy in midpoints and right_buoy in midpoints and midpoints[left_buoy] and midpoints[right_buoy]:
            connector_midpoint = (int((midpoints[left_buoy][0] + midpoints[right_buoy][0]) / 2),
                                  int((midpoints[left_buoy][1] + midpoints[right_buoy][1]) / 2))

        # Mission handlers
        if mission == "Floating Ball":
            if buoy_detections_pgf:
                pgf_yaw, pgf_status, pgf_debug = self.get_safe_heading(
                    buoy_detections_pgf, width, batas_tengah, batas_bawah, batas_atas,
                    depth_image=depth_image, depth_scale=depth_scale, velocities=velocities
                )
                yaw = pgf_yaw
            else:
                yaw = batas_tengah

        elif mission == "Kamikaze":
            target_buoy = left_buoy
            if target_buoy == "black_buoy" and (midpoints.get("black_buoy") is None) and (midpoints.get("blue_box") is not None):
                target_buoy = "blue_box"

            if target_buoy in midpoints and midpoints[target_buoy] is not None:
                a, b = midpoints[target_buoy]
                # Optional: use measured depth here too if you want distance-aware Kamikaze
                yaw = batas_bawah + a * ((batas_atas - batas_bawah) * (1 / 640))

        left_score  = float(max_conf.get(left_buoy, 0.0))
        right_score = float(max_conf.get(right_buoy, 0.0))

        return frame, yaw, left_score, right_score


# ─────────────────────────────────────────────────────────────────────────────
# EXAMPLE: how to feed the D455 into processor()
# ─────────────────────────────────────────────────────────────────────────────
# import pyrealsense2 as rs
#
# pipeline = rs.pipeline()
# config = rs.config()
# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
# config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
# profile = pipeline.start(config)
#
# depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()  # ~0.001
# align = rs.align(rs.stream.color)   # IMPORTANT: align depth to color (bbox space)
#
# viz = BBoxVisualization(cls_dict)
#
# # set intrinsics ONCE (from the aligned depth stream)
# frames  = align.process(pipeline.wait_for_frames())
# intr    = frames.get_depth_frame().profile.as_video_stream_profile().intrinsics
# viz.set_depth_intrinsics({'fx': intr.fx, 'fy': intr.fy, 'ppx': intr.ppx, 'ppy': intr.ppy})
#
# while True:
#     frames  = align.process(pipeline.wait_for_frames())
#     color_frame = frames.get_color_frame()
#     depth_frame = frames.get_depth_frame()
#     if not color_frame or not depth_frame:
#         continue
#     color_image = np.asanyarray(color_frame.get_data())
#     depth_image = np.asanyarray(depth_frame.get_data())   # uint16, aligned to color
#
#     # boxes, confs, clss = your_yolo(color_image)   # run detection on COLOR
#
#     frame_out, yaw = viz.processor(
#         color_image, fps, mission, left_buoy, right_buoy,
#         boxes, confs, clss, width=640, height=480,
#         batas_bawah=batas_bawah, batas_tengah=batas_tengah, batas_atas=batas_atas,
#         min_buoy_area=min_buoy_area,
#         depth_image=depth_image, depth_scale=depth_scale
#     )