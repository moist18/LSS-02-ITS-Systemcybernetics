import numpy as np
import cv2


class BBoxVisualization():
    def __init__(self, cls_dict):
        self.cls_dict = cls_dict  # ['blue_box', 'green_box', 'green_buoy', 'green_dock', 'red_buoy', 'red_dock', 'black_buoy', 'yellow_buoy', 'orange_buoy']

        # Constants
        self.FONT = cv2.FONT_HERSHEY_SIMPLEX
        self.FONT_SIZE = 0.75
        self.THICKNESS = 2
        self.CIRCLE_SIZE = 7
        self.BLUE_COLOR   = (255, 155, 0)
        self.GREEN_COLOR  = (0, 255, 0)
        self.RED_COLOR    = (0, 0, 255)
        self.PURPLE_COLOR = (255, 55, 125)
        self.ORANGE_COLOR = (0, 125, 255)
        self.CYAN_COLOR   = (255, 255, 0)
        self.BLACK_COLOR  = (0, 0, 0)
        self.WHITE_COLOR  = (255, 255, 255)

        # Deadzone config
        self.midpoint_deadzone = 50
        self.single_deadzone   = int(self.midpoint_deadzone * 4.5)

    # ── HSV colour-blob fallback ───────────────────────────────────────────────
    # Kamikaze ONLY. Kicks in when YOLO returns nothing for the target class.
    # Cheaper and more lighting-sensitive than the model — never overrides a
    # real YOLO detection, only fills the gap when midpoints[target] is None.
    # Tune ranges using teshsv.py on the actual field lighting.
    #
    # Format per entry: [H_lo, S_lo, V_lo, H_hi, S_hi, V_hi]
    # Red wraps around H=0/180 so it needs two entries (red_buoy → red1 + red2).
    HSV_RANGES = {
        'green_buoy': ([35,  60,  40],  [85,  255, 255], None,          None         ),
        'red_buoy':   ([0,   100, 50],  [10,  255, 255], [160, 100, 50],[180, 255, 255]),
        'black_buoy': ([0,   0,   0],   [180, 80,  80],  None,          None         ),
    }

    def _detect_by_hsv(self, frame, color_name, min_area):
        """Return centroid (cx, cy) of the largest matching HSV blob, or None."""
        entry = self.HSV_RANGES.get(color_name)
        if entry is None or frame is None:
            return None

        lower1, upper1, lower2, upper2 = entry
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(lower1), np.array(upper1))
        if lower2 is not None:
            mask = cv2.bitwise_or(
                mask, cv2.inRange(hsv, np.array(lower2), np.array(upper2)))

        # Morphology: remove small noise
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        largest = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(largest) < min_area:
            return None
        m = cv2.moments(largest)
        if m['m00'] == 0:
            return None
        return (int(m['m10'] / m['m00']), int(m['m01'] / m['m00']))

        # ── PGF (Potential Gap Finding) parameters ────────────────────────────
        # R_safe: safety radius around each buoy in meters.
        # Any heading within R_safe of a buoy is blocked.
        self.pgf_r_safe          = 1.0    # meters  (safety clearance)

        # Focal length X (pixels) — used to convert pixel offset → angle.
        # Fallback only; real intrinsics (set_depth_intrinsics) take priority.
        self.pgf_focal_length_x  = 320.0  # pixels

        # Field of view in radians (horizontal). Fallback reference range.
        self.pgf_fov_rad         = np.deg2rad(87.0)  # D455 depth H-FoV ~87°

        # Emergency turn RC value difference from centre — applied when a buoy
        # is closer than R_safe (collision imminent).
        self.pgf_emergency_turn  = 200    # RC units away from mid

        # Real buoy diameter in metres (used for monocular distance estimate).
        self.pgf_buoy_diameter_m = 0.20   # meters -- fallback only

        # Ignore buoys farther than this (meters).
        self.pgf_max_range       = 8.0

        # Score penalty applied per side of a gap that is NOT flanked by an
        # actual detected buoy (i.e. that side is just the outer FOV edge).
        # Keeps PGF from chasing "wide open" unmarked water (which may be
        # outside the arena) over a real, narrower buoy-to-buoy channel.
        self.pgf_unbounded_gap_penalty = 400.0   # score units per unbounded side

        # Mild score penalty per degree a gap's centre deviates from
        # straight-ahead (0°) — tie-breaker so PGF prefers forward-centred
        # gaps over off-angle ones of similar width.
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
        """Convert horizontal pixel position to angle relative to camera axis (radians).
        Prefer true intrinsics (accurate, non-linear). Fall back to FoV model."""
        if self.depth_intrinsics is not None:
            fx  = self.depth_intrinsics['fx']
            ppx = self.depth_intrinsics['ppx']
            return float(np.arctan((cx - ppx) / fx))
        return (cx - image_width / 2.0) * self.pgf_fov_rad / image_width

    def _angle_limits(self, image_width):
        """Angular search range [lo, hi] — asymmetric when intrinsics are set."""
        if self.depth_intrinsics is not None:
            fx  = self.depth_intrinsics['fx']
            ppx = self.depth_intrinsics['ppx']
            lo = float(np.arctan((0.0 - ppx) / fx))
            hi = float(np.arctan((image_width - ppx) / fx))
            return lo, hi
        return -self.pgf_fov_rad / 2.0, self.pgf_fov_rad / 2.0

    def _merge_intervals(self, intervals):
        """Merge a sorted list of (lo, hi) angle intervals."""
        if not intervals:
            return []
        merged = [intervals[0]]
        for lo, hi in intervals[1:]:
            if lo <= merged[-1][1]:
                merged[-1] = (merged[-1][0], max(merged[-1][1], hi))
            else:
                merged.append((lo, hi))
        return merged

    def _find_largest_gap(self, blocked, lo_limit, hi_limit):
        """
        Find the widest free angular gap within [lo_limit, hi_limit]
        given a list of merged blocked intervals.
        Returns (gap_lo, gap_hi) of the best free segment.
        """
        # Build candidate free segments between blocked intervals
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
            # No free gap — return straight ahead
            return (lo_limit, hi_limit)

        return max(candidates, key=lambda seg: seg[1] - seg[0])

    def _find_all_free_gaps(self, blocked, lo_limit, hi_limit):
        """
        Find ALL free angular gaps within [lo_limit, hi_limit].
        Returns list of (gap_lo, gap_hi, bounded_lo, bounded_hi) tuples.

        bounded_lo/bounded_hi are False when that side of the gap coincides
        with the outer search-range edge instead of an actual buoy's blocked
        interval — i.e. the gap is only "open" because nothing was detected
        there (could be open water / outside the arena), not because a real
        buoy marks that side of a channel.
        """
        candidates = []
        prev_end = lo_limit
        for lo, hi in blocked:
            lo_clipped = max(lo, lo_limit)
            hi_clipped = min(hi, hi_limit)
            if lo_clipped > prev_end:
                bounded_lo = prev_end > lo_limit
                bounded_hi = True
                candidates.append((prev_end, lo_clipped, bounded_lo, bounded_hi))
            prev_end = max(prev_end, hi_clipped)
        if prev_end < hi_limit:
            bounded_lo = prev_end > lo_limit
            candidates.append((prev_end, hi_limit, bounded_lo, False))
        return candidates if candidates else [(lo_limit, hi_limit, False, False)]

    def _score_gap(self, gap, buoy_angles, image_width, fov_span, velocities=None):
        """
        Score a gap using the formula:
          score = 0.5 * gap_width_px + 0.4 * distance_m - 0.1 * risk
                  - unbounded_penalty - heading_bias

        Args:
            gap:          (lo_rad, hi_rad, bounded_lo, bounded_hi) angular gap bounds
            buoy_angles:  list of (angle, dist, bbox_w, cx) for all buoys
            image_width:  frame width in pixels
            fov_span:     angular search range span (hi_limit - lo_limit), radians
            velocities:   dict {track_id: (vx, vy)} or None

        Returns:
            (score, gap_width_px, avg_dist, risk)
        """
        gap_lo, gap_hi, bounded_lo, bounded_hi = gap
        gap_center_rad = (gap_lo + gap_hi) / 2.0

        # Convert angular width to pixel width
        gap_width_rad = gap_hi - gap_lo
        gap_width_px  = gap_width_rad / fov_span * image_width

        # Distance: average distance of the two closest buoys flanking this gap
        flank_dists = []
        for angle, dist, _, _ in buoy_angles:
            if dist < float('inf'):
                flank_dists.append(dist)
        avg_dist = np.mean(flank_dists) if flank_dists else 5.0  # default 5m

        # Risk: proximity of nearest buoy to gap center (0–100 scale)
        risk = 0.0
        if buoy_angles:
            min_angular_dist = min(
                abs(angle - gap_center_rad) for angle, _, _, _ in buoy_angles
            )
            # Closer buoy to gap center → higher risk
            # risk 100 when buoy is at gap center, 0 when far away
            risk = max(0.0, 100.0 * (1.0 - min_angular_dist / (fov_span / 2.0)))

        # Unbounded-side penalty: a gap not flanked by a real buoy on BOTH
        # sides is only "open" because nothing was detected there (could be
        # open water outside the arena) — heavily de-prioritise it vs a real
        # buoy-to-buoy channel. Only side actually missing a buoy is penalised,
        # so a gap missing both sides (e.g. no buoys at all) is penalised twice.
        unbounded_penalty = 0.0
        if not bounded_lo:
            unbounded_penalty += self.pgf_unbounded_gap_penalty
        if not bounded_hi:
            unbounded_penalty += self.pgf_unbounded_gap_penalty

        # Heading bias: mild preference for gaps closer to straight-ahead (0 rad),
        # so the boat doesn't chase a wide-but-off-angle opening when a
        # comparable centred gap exists.
        heading_bias = self.pgf_heading_bias_weight * abs(np.degrees(gap_center_rad))

        # Apply scoring formula
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

    def get_safe_heading(self, detections, image_width, mid_rc, batas_bawah, batas_atas,
                         depth_image=None, depth_scale=None, velocities=None):
        """
        PGF: Potential Gap Finding for buoy navigation.

        Converts all visible buoys to angular blocked intervals, finds the
        widest free gap, and returns the target RC yaw value.

        Distance is measured directly from the D455 depth map at each buoy's
        pixel (accurate at any range); the monocular bbox-width formula is
        used ONLY as a fallback when depth is missing/invalid there.

        Args:
            detections:  list of dict with keys 'cls_name', 'x1','y1','x2','y2'
                         (pre-filtered to only buoys in the Floating Ball mission)
            image_width: frame width in pixels
            mid_rc:      RC mid-value (neutral steering)
            batas_bawah: minimum RC yaw value
            batas_atas:  maximum RC yaw value
            depth_image: aligned depth frame (numpy, uint16) or None
            depth_scale: meters per depth unit; defaults to depth_default_scale

        Returns:
            (rc_yaw, status_str, debug_info)
            rc_yaw:      int — RC override yaw value
            status_str:  str — human-readable status for OSD
            debug_info:  dict — angles & intervals for visualization
        """
        if depth_scale is None:
            depth_scale = self.depth_default_scale

        buoy_angles = []  # list of (angle_rad, distance_m, bbox_width_px, cx)

        for det in detections:
            bbox_w = det['x2'] - det['x1']
            cx     = (det['x1'] + det['x2']) / 2.0
            cy     = (det['y1'] + det['y2']) / 2.0

            # 1) measured depth first, 2) bbox-size estimate only as fallback
            dist = self._sample_depth(depth_image, cx, cy, depth_scale)
            if dist is None or dist <= 0.0:
                dist = self._estimate_distance_from_bbox(bbox_w)

            if dist == float('inf') or dist > self.pgf_max_range:
                # Too far / too small — skip
                continue

            angle = self._pixel_to_angle(cx, image_width)
            buoy_angles.append((angle, dist, bbox_w, cx))

        if not buoy_angles:
            # No relevant buoys detected — go straight
            return mid_rc, "PGF: NO BUOY (straight)", {}

        # Emergency: any buoy closer than R_safe
        for angle, dist, _, _ in buoy_angles:
            if dist < self.pgf_r_safe:
                # Steer away from the nearest threat
                emergency_yaw = mid_rc + (self.pgf_emergency_turn if angle < 0 else -self.pgf_emergency_turn)
                emergency_yaw = int(np.clip(emergency_yaw, batas_bawah, batas_atas))
                return emergency_yaw, "PGF: EMERGENCY TURN", {'emergency': True}

        # Build blocked angular intervals
        intervals = []
        for angle, dist, _, _ in buoy_angles:
            half_block = np.arcsin(np.clip(self.pgf_r_safe / dist, -1.0, 1.0))
            intervals.append((angle - half_block, angle + half_block))

        # Optional: add generic obstacles straight from the depth map
        if self.pgf_use_depth_obstacles:
            intervals += self._depth_obstacle_intervals(depth_image, depth_scale, image_width)

        merged = self._merge_intervals(intervals)

        # Search range: real intrinsics FoV when available, else fallback FoV
        lo_limit, hi_limit = self._angle_limits(image_width)
        fov_span = hi_limit - lo_limit

        # Find ALL free gaps and score each one
        all_gaps = self._find_all_free_gaps(merged, lo_limit, hi_limit)

        scored_gaps = []
        for gap in all_gaps:
            score, gw_px, avg_d, risk = self._score_gap(
                gap, buoy_angles, image_width, fov_span, velocities
            )
            scored_gaps.append({
                'gap': gap,
                'score': score,
                'width_px': gw_px,
                'distance': avg_d,
                'risk': risk
            })

        # Select gap with highest score
        scored_gaps.sort(key=lambda g: g['score'], reverse=True)
        best = scored_gaps[0]
        best_gap   = best['gap']
        target_ang = (best_gap[0] + best_gap[1]) / 2.0  # centre of gap

        # Convert angle → RC yaw (linear mapping over the actual search range)
        rc_yaw = batas_bawah + (target_ang - lo_limit) / fov_span * (batas_atas - batas_bawah)
        rc_yaw = int(np.clip(rc_yaw, batas_bawah, batas_atas))

        debug_info = {
            'buoy_angles': buoy_angles,
            'merged_blocked': merged,
            'best_gap': best_gap,
            'target_ang_deg': np.degrees(target_ang),
            'scored_gaps': scored_gaps,
            'lo_limit': lo_limit,
            'hi_limit': hi_limit,
        }

        gap_width_deg = np.degrees(best_gap[1] - best_gap[0])
        return rc_yaw, f"PGF: gap {gap_width_deg:.0f}° s={best['score']:.0f}", debug_info

    # ── Draw PGF overlay ────────────────────────────────────────────────────

    def draw_pgf_overlay(self, frame, debug_info, width, height, batas_bawah, batas_atas):
        """Draw PGF angular gap visualization with score info."""
        if not debug_info or debug_info.get('emergency'):
            return

        cx_frame = width  // 2
        cy_frame = height - 20  # arc origin at bottom-centre
        radius   = min(width, height) // 3

        # Use the actual search range from get_safe_heading (asymmetric when
        # real depth intrinsics are set) instead of assuming symmetric FoV.
        lo_limit = debug_info.get('lo_limit', -self.pgf_fov_rad / 2.0)
        hi_limit = debug_info.get('hi_limit',  self.pgf_fov_rad / 2.0)

        def angle_to_xy(ang_rad):
            a = -np.pi / 2 + ang_rad
            return (int(cx_frame + radius * np.cos(a)),
                    int(cy_frame + radius * np.sin(a)))

        # Draw blocked intervals in red
        for lo, hi in debug_info.get('merged_blocked', []):
            lo_c = max(lo, lo_limit)
            hi_c = min(hi, hi_limit)
            if lo_c < hi_c:
                p1 = angle_to_xy(lo_c)
                p2 = angle_to_xy(hi_c)
                cv2.line(frame, (cx_frame, cy_frame), p1, self.RED_COLOR, 2)
                cv2.line(frame, (cx_frame, cy_frame), p2, self.RED_COLOR, 2)

        # Draw ALL scored gaps with score labels
        scored_gaps = debug_info.get('scored_gaps', [])
        for i, sg in enumerate(scored_gaps):
            gap = sg['gap']
            score = sg['score']
            is_best = (i == 0)  # first is best (sorted by score desc)
            color = self.GREEN_COLOR if is_best else (100, 200, 100)
            thickness = 2 if is_best else 1

            p_lo = angle_to_xy(max(gap[0], lo_limit))
            p_hi = angle_to_xy(min(gap[1], hi_limit))
            cv2.line(frame, (cx_frame, cy_frame), p_lo, color, thickness)
            cv2.line(frame, (cx_frame, cy_frame), p_hi, color, thickness)

            # Score label at gap midpoint on arc
            mid_ang = (gap[0] + gap[1]) / 2.0
            p_mid = angle_to_xy(mid_ang)
            label = f"S:{score:.0f}"
            cv2.putText(frame, label, (p_mid[0]-15, p_mid[1]-5),
                        self.FONT, 0.4, color, 1)

        # Draw target heading arrow in cyan
        target_ang = np.deg2rad(debug_info.get('target_ang_deg', 0.0))
        p_target   = angle_to_xy(target_ang)
        cv2.arrowedLine(frame, (cx_frame, cy_frame), p_target, self.CYAN_COLOR, 3, tipLength=0.2)

        # Best gap info HUD
        if scored_gaps:
            best = scored_gaps[0]
            info = (f"PGF: {debug_info['target_ang_deg']:.1f}°  "
                    f"W:{best['width_px']:.0f}px  "
                    f"D:{best['distance']:.1f}m  "
                    f"R:{best['risk']:.0f}  "
                    f"S:{best['score']:.0f}")
            cv2.putText(frame, info, (10, height - 10),
                        self.FONT, 0.45, self.CYAN_COLOR, self.THICKNESS)

    # ── Course 1: orange-buoy gate centerline ──────────────────────────────

    def _course1_orange_gate(self, frame, boxes, confs, clss, width, height,
                              batas_bawah, batas_tengah, batas_atas, min_buoy_area):
        """
        Course 1 Floating Ball: steer through a gate formed by two orange buoys.
        Picks the two largest orange_buoy detections as the gate posts and aims
        at their midpoint. Falls back to steering straight at a single buoy if
        only one is visible.
        """
        orange_dets = []
        for bb, cf, cl in zip(boxes, confs, clss):
            cls_name = self.cls_dict.get(int(cl), '')
            if cls_name != 'orange_buoy':
                continue
            x1, y1, x2, y2 = map(float, bb)
            area = (x2 - x1) * (y2 - y1)
            if area < min_buoy_area:
                continue
            orange_dets.append({
                'cx': (x1 + x2) / 2.0, 'cy': (y1 + y2) / 2.0,
                'bbox': (int(x1), int(y1), int(x2), int(y2))
            })

        yaw = batas_tengah
        status = "C1: NO ORANGE GATE (straight)"

        if len(orange_dets) >= 2:
            orange_dets.sort(key=lambda d: (d['bbox'][2] - d['bbox'][0]) * (d['bbox'][3] - d['bbox'][1]),
                              reverse=True)
            left, right = sorted(orange_dets[:2], key=lambda d: d['cx'])
            mid_x = (left['cx'] + right['cx']) / 2.0
            mid_y = (left['cy'] + right['cy']) / 2.0
            midpoint = (int(mid_x), int(mid_y))

            for d in (left, right):
                cv2.rectangle(frame, d['bbox'][:2], d['bbox'][2:], self.ORANGE_COLOR, self.THICKNESS)
            left_pt  = (int(left['cx']),  int(left['cy']))
            right_pt = (int(right['cx']), int(right['cy']))
            cv2.line(frame, left_pt, right_pt, self.WHITE_COLOR, self.THICKNESS)
            cv2.circle(frame, midpoint, self.CIRCLE_SIZE, self.RED_COLOR, -1)
            cv2.line(frame, (width // 2, height // 2), midpoint, self.WHITE_COLOR, self.THICKNESS)

            yaw = batas_bawah + mid_x * ((batas_atas - batas_bawah) / width)
            status = f"C1: GATE CENTER x={mid_x:.0f}"

            if mid_x < width / 2 - self.midpoint_deadzone:
                cv2.putText(frame, "TURN", (560, 30), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
                cv2.putText(frame, "LEFT", (560, 60), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
            elif mid_x > width / 2 + self.midpoint_deadzone:
                cv2.putText(frame, "TURN",  (560, 30), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
                cv2.putText(frame, "RIGHT", (560, 60), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)

        elif len(orange_dets) == 1:
            d = orange_dets[0]
            cv2.rectangle(frame, d['bbox'][:2], d['bbox'][2:], self.ORANGE_COLOR, self.THICKNESS)
            pt = (int(d['cx']), int(d['cy']))
            cv2.line(frame, (width // 2, height // 2), pt, self.WHITE_COLOR, self.THICKNESS)
            cv2.circle(frame, pt, self.CIRCLE_SIZE, self.RED_COLOR, -1)

            yaw = batas_bawah + d['cx'] * ((batas_atas - batas_bawah) / width)
            status = f"C1: SINGLE ORANGE x={d['cx']:.0f}"

        cv2.putText(frame, status, (10, height - 30), self.FONT, 0.6, self.CYAN_COLOR, self.THICKNESS)
        return yaw

    # ── Main processor ───────────────────────────────────────────────────────

    def processor(self, frame, fps, mission, left_buoy, right_buoy,
                  boxes, confs, clss, width, height,
                  batas_bawah, batas_tengah, batas_atas,
                  min_buoy_area,
                  track_ids=None, velocities=None, distances=None,
                  course=2, depth_image=None, depth_scale=None):

        connector_midpoint = None
        yaw = batas_tengah

        max_area = {k: 0    for k in ['blue_box','green_box','green_buoy','green_dock','red_buoy','red_dock','black_buoy','yellow_buoy','orange_buoy']}
        max_bbox = {k: None for k in ['blue_box','green_box','green_buoy','green_dock','red_buoy','red_dock','black_buoy','yellow_buoy','orange_buoy']}
        max_conf = {k: 0    for k in ['blue_box','green_box','green_buoy','green_dock','red_buoy','red_dock','black_buoy','yellow_buoy','orange_buoy']}
        midpoints= {k: None for k in ['blue_box','green_box','green_buoy','green_dock','red_buoy','red_dock','black_buoy','yellow_buoy','orange_buoy']}

        # ── Gambar bbox + midpoint line untuk semua deteksi (sebelum filter area) ──
        # Mengikuti pola visual.py: semua visualisasi ada di dalam class utility.
        # Drawing dilakukan di sini agar SEMUA bbox selalu tampil terlepas dari
        # filter area yang dipakai untuk logika navigasi di bawah.
        CLASS_COLORS = {
            'green_buoy':  (0, 255, 0),
            'red_buoy':    (0, 0, 255),
            'green_dock':  (0, 200, 100),
            'red_dock':    (0, 80, 255),
            'green_box':   (100, 255, 100),
            'blue_box':    (255, 100, 0),
            'black_buoy':  (0, 0, 0),
            'yellow_buoy': (0, 255, 255),
            'orange_buoy': (0, 125, 255),
        }
        frame_cx = width  // 2
        frame_cy = height // 2
        if track_ids is None:
            track_ids = []
        if velocities is None:
            velocities = {}

        for idx, (bb, cf, cl) in enumerate(zip(boxes, confs, clss)):
            cl_int = int(cl)
            bx1, by1, bx2, by2 = [int(float(v)) for v in bb]
            cls_name_raw = self.cls_dict.get(cl_int, f'CLS{cl_int}')
            box_color = CLASS_COLORS.get(cls_name_raw, (0, 255, 255))

            # Bounding box
            cv2.rectangle(frame, (bx1, by1), (bx2, by2), box_color, self.THICKNESS)

            # Stacked vertical labels (Track ID, Distance, Class & Conf)
            label_lines = []
            tid = track_ids[idx] if idx < len(track_ids) else None
            if tid is not None:
                label_lines.append(f"#{tid}")

            # Include distance if available
            dist = distances[idx] if (distances is not None and idx < len(distances)) else None
            if dist is not None and dist > 0:
                label_lines.append(f"{dist:.2f}m")

            label_lines.append(f"{cls_name_raw} {float(cf):.2f}")

            # Draw labels stacked vertically upwards from the top edge of bbox
            y = by1 - 8
            for line in reversed(label_lines):
                draw_y = max(y, 15)
                cv2.putText(frame, line, (bx1, draw_y),
                            self.FONT, 0.50, box_color, self.THICKNESS)
                y -= 15

            # Midpoint objek + line dari pusat frame (seperti visual.py)
            obj_cx = (bx1 + bx2) // 2
            obj_cy = (by1 + by2) // 2
            cv2.circle(frame, (obj_cx, obj_cy), self.CIRCLE_SIZE, self.RED_COLOR, -1)
            cv2.line(frame, (frame_cx, frame_cy), (obj_cx, obj_cy),
                     self.WHITE_COLOR, self.THICKNESS)

            # Velocity arrow (direction of movement)
            if tid is not None and tid in velocities:
                vx, vy = velocities[tid]
                # Scale velocity for visualization (arrow length)
                arrow_scale = 5.0
                end_x = int(obj_cx + vx * arrow_scale)
                end_y = int(obj_cy + vy * arrow_scale)
                cv2.arrowedLine(frame, (obj_cx, obj_cy), (end_x, end_y),
                                self.ORANGE_COLOR, 2, tipLength=0.3)

        # ── Collect detections ─────────────────────────────────────────────
        buoy_detections_pgf = []   # raw dicts for PGF (Floating Ball only, no area filter yet)

        for bb, cf, cl in zip(boxes, confs, clss):
            cl = int(cl)
            x1, y1, x2, y2 = map(float, bb)
            cls_name = self.cls_dict.get(cl, f'CLS{cl}')
            midpoint = (int((x1 + x2) / 2), int((y1 + y2) / 2))
            area     = (x2 - x1) * (y2 - y1)

            if mission == "Floating Ball":
                # Collect all buoys for PGF (no area filter — PGF uses distance instead)
                if cls_name in ['green_buoy', 'red_buoy', 'black_buoy', 'yellow_buoy', 'orange_buoy']:
                    buoy_detections_pgf.append({
                        'cls_name': cls_name,
                        'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                        'conf': cf
                    })

            if mission in ("Floating Ball", "Kamikaze"):
                # Area filter for fallback steering display / Kamikaze target lock
                if cls_name in ['green_buoy', 'red_buoy', 'black_buoy', 'yellow_buoy', 'orange_buoy'] and area < min_buoy_area:
                    continue

            if cls_name in max_area and area > max_area[cls_name]:
                max_area[cls_name] = area
                max_bbox[cls_name] = (int(x1), int(y1), int(x2), int(y2))
                max_conf[cls_name] = cf
                midpoints[cls_name] = midpoint

        # ── Draw bounding boxes ───────────────────────────────────────────
        for cls_name, bbox in max_bbox.items():
            if bbox is not None:
                x1, y1, x2, y2 = bbox
                conf  = max_conf[cls_name]
                color = (self.GREEN_COLOR if cls_name == 'green_buoy'
                         else self.RED_COLOR if cls_name == 'red_buoy'
                         else self.BLACK_COLOR if cls_name == 'black_buoy'
                         else (0, 255, 255) if cls_name == 'yellow_buoy'
                         else self.ORANGE_COLOR if cls_name == 'orange_buoy'
                         else self.BLUE_COLOR)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, self.THICKNESS)
                cv2.circle(frame, midpoints[cls_name], self.CIRCLE_SIZE, self.RED_COLOR, -1)
                cv2.putText(frame, f"{cls_name} {conf:.2f}", (x1, y1 - 10),
                            self.FONT, self.FONT_SIZE, color, self.THICKNESS)

        # ── Connector midpoint (both buoys visible) ───────────────────────
        if left_buoy in midpoints and right_buoy in midpoints and midpoints[left_buoy] and midpoints[right_buoy]:
            connector_midpoint = (
                int((midpoints[left_buoy][0] + midpoints[right_buoy][0]) / 2),
                int((midpoints[left_buoy][1] + midpoints[right_buoy][1]) / 2)
            )

        # ══════════════════════════════════════════════════════════════════
        #  MISSION HANDLERS
        #  "Floating Ball" = PGF (potential gap finder, multi-centerline)
        #  "Kamikaze"       = target one of green/red/black buoy to ram
        # ══════════════════════════════════════════════════════════════════

        if mission == "Floating Ball":
            # ── PGF: primary steering when any buoy is visible ────────────
            if buoy_detections_pgf:
                pgf_yaw, pgf_status, pgf_debug = self.get_safe_heading(
                    buoy_detections_pgf, width, batas_tengah, batas_bawah, batas_atas,
                    depth_image=depth_image, depth_scale=depth_scale,
                    velocities=velocities
                )
                yaw = pgf_yaw

                # Draw PGF arc overlay
                self.draw_pgf_overlay(frame, pgf_debug, width, height, batas_bawah, batas_atas)

                # Draw connector line if both buoys visible
                if connector_midpoint:
                    cv2.line(frame, midpoints[left_buoy], midpoints[right_buoy],
                             self.WHITE_COLOR, self.THICKNESS)
                    cv2.line(frame, (int(width / 2), int(height / 2)), connector_midpoint,
                             self.WHITE_COLOR, self.THICKNESS)
                    cv2.circle(frame, connector_midpoint, self.CIRCLE_SIZE, self.RED_COLOR, -1)

                    a = connector_midpoint[0]
                    if a < int(width / 2) - self.midpoint_deadzone:
                        cv2.putText(frame, "TURN", (560, 30), self.FONT, self.FONT_SIZE,
                                    self.ORANGE_COLOR, self.THICKNESS)
                        cv2.putText(frame, "LEFT", (560, 60), self.FONT, self.FONT_SIZE,
                                    self.ORANGE_COLOR, self.THICKNESS)
                    elif a > int(width / 2) + self.midpoint_deadzone:
                        cv2.putText(frame, "TURN", (560, 30), self.FONT, self.FONT_SIZE,
                                    self.ORANGE_COLOR, self.THICKNESS)
                        cv2.putText(frame, "RIGHT", (560, 60), self.FONT, self.FONT_SIZE,
                                    self.ORANGE_COLOR, self.THICKNESS)
                else:
                    # Single buoy visible — draw line to it
                    visible_buoy = (midpoints[left_buoy] if left_buoy in midpoints else None) or (midpoints[right_buoy] if right_buoy in midpoints else None)
                    if visible_buoy:
                        cv2.line(frame, (int(width / 2), int(height / 2)), visible_buoy,
                                 self.WHITE_COLOR, self.THICKNESS)
                        cv2.circle(frame, visible_buoy, self.CIRCLE_SIZE, self.RED_COLOR, -1)

                # PGF status HUD (bottom-left)
                cv2.putText(frame, pgf_status, (10, height - 30),
                            self.FONT, 0.6, self.CYAN_COLOR, self.THICKNESS)

            else:
                # No buoys — straight ahead (yaw stays at batas_tengah)
                cv2.putText(frame, "PGF: NO BUOY (straight)", (10, height - 30),
                            self.FONT, 0.6, self.WHITE_COLOR, self.THICKNESS)

        elif mission == "Kamikaze":
            target_buoy = left_buoy
            # Fallback if target is black_buoy but only blue_box is detected
            if target_buoy == "black_buoy" and (midpoints.get("black_buoy") is None) and (midpoints.get("blue_box") is not None):
                target_buoy = "blue_box"

            if target_buoy in midpoints and midpoints[target_buoy] is not None:
                a, b = midpoints[target_buoy]
                yaw  = batas_bawah + a * ((batas_atas - batas_bawah) * (1 / 640))

                cv2.line(frame, (int(width / 2), int(height / 2)), midpoints[target_buoy],
                         self.WHITE_COLOR, self.THICKNESS)
                cv2.circle(frame, midpoints[target_buoy], self.CIRCLE_SIZE, self.RED_COLOR, -1)

                if a < int(width / 2) - self.midpoint_deadzone:
                    cv2.putText(frame, "TURN", (560, 30), self.FONT, self.FONT_SIZE,
                                self.ORANGE_COLOR, self.THICKNESS)
                    cv2.putText(frame, "LEFT",  (560, 60), self.FONT, self.FONT_SIZE,
                                self.ORANGE_COLOR, self.THICKNESS)
                elif a > int(width / 2) + self.midpoint_deadzone:
                    cv2.putText(frame, "TURN",  (560, 30), self.FONT, self.FONT_SIZE,
                                self.ORANGE_COLOR, self.THICKNESS)
                    cv2.putText(frame, "RIGHT", (560, 60), self.FONT, self.FONT_SIZE,
                                self.ORANGE_COLOR, self.THICKNESS)

        # ── Deadzone grid ─────────────────────────────────────────────────
        cv2.circle(frame, (int(width / 2), int(height / 2)), self.CIRCLE_SIZE, self.RED_COLOR, -1)
        cv2.line(frame, (int(width/2) - self.single_deadzone, 0),
                        (int(width/2) - self.single_deadzone, height), self.GREEN_COLOR, self.THICKNESS)
        cv2.line(frame, (int(width/2) + self.single_deadzone, 0),
                        (int(width/2) + self.single_deadzone, height), self.GREEN_COLOR, self.THICKNESS)
        cv2.line(frame, (int(width/2) - self.midpoint_deadzone, 0),
                        (int(width/2) - self.midpoint_deadzone, height), self.BLUE_COLOR, self.THICKNESS)
        cv2.line(frame, (int(width/2) + self.midpoint_deadzone, 0),
                        (int(width/2) + self.midpoint_deadzone, height), self.BLUE_COLOR, self.THICKNESS)
        cv2.line(frame, (0, int(height/2) - self.midpoint_deadzone),
                        (width, int(height/2) - self.midpoint_deadzone), self.RED_COLOR, self.THICKNESS)
        cv2.line(frame, (0, int(height/2) + self.midpoint_deadzone),
                        (width, int(height/2) + self.midpoint_deadzone), self.RED_COLOR, self.THICKNESS)

        # ── HUD ───────────────────────────────────────────────────────────
        cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), self.FONT, self.FONT_SIZE,
                    self.ORANGE_COLOR, self.THICKNESS)
        cv2.putText(frame, f"YAW: {int(yaw)}", (10, 60), self.FONT, self.FONT_SIZE,
                    self.ORANGE_COLOR, self.THICKNESS)

        # Detection confidence of whatever class is currently designated as
        # left_buoy/right_buoy (0.0 if that class wasn't seen this frame) —
        # feeds the mission planner's Course 2 fusion-confidence calculation.
        left_score  = float(max_conf.get(left_buoy, 0.0))
        right_score = float(max_conf.get(right_buoy, 0.0))

        return frame, yaw, left_score, right_score
