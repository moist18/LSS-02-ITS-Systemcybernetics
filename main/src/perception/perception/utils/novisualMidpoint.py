import time

import numpy as np

class BBoxVisualization():
    """Midpoint steering without drawing overlay, for the D455 depth-camera node.

    Non-visual twin of visualMidpoint.py: identical steering decisions, no OpenCV
    drawing, to save CPU during competition runs. Keep the two in step — every
    steering change here must be mirrored there and vice versa.

    Steering is GEOMETRIC, not colour-slot based. The arena has three buoys —
    orange left, yellow centre, orange right — which a colour-slot model cannot
    represent: it keeps one representative per colour, so the two oranges collapse
    into one and the gate pair jumps between the left and right gap as the boat
    moves. Worse, an orange appearing on the right still used the left-slot formula
    and steered *into* it (measured in the field: raw_yaw pinned at 1700).

    Here every buoy is a candidate. Adjacent buoys (sorted left to right) form
    candidate gates, and the gate with the longest connector line wins. Colour is
    still reported back on the score topics, but it no longer decides where to steer.
    """

    # Buoy classes that can act as a gate post.
    BUOY_CLASSES = ['green_buoy', 'red_buoy', 'black_buoy', 'orange_buoy', 'yellow_buoy']

    # Everything the Kamikaze branch and the score topics can address by name.
    TRACKED_CLASSES = BUOY_CLASSES + ['blue_box', 'green_box', 'green_dock', 'red_dock']

    def __init__(self, cls_dict):
        self.cls_dict = cls_dict

        # Gate search tuning — after a gate is passed, the next one is usually
        # outside the field of view, because the boat leaves the gate pointing
        # wherever the gate pointed it. These control the sweep that goes looking
        # for it. Keep in step with visualMidpoint.
        self.pass_line_ratio = 0.60        # gate counts as "close" once its connector spans this much of the frame
        self.pass_depth_m = 2.0            # ...or once either post is measured this near
        self.search_timeout_sec = 6.0      # how long to sweep before giving up
        self.search_yaw_offset = 120       # RC units from mid while sweeping
        self.gate_grace_sec = 0.5          # a gate lost for less than this may still come back

        # Gate search state
        self._last_gate_time = None
        self._gate_close = False
        self._search_active = False        # sweeping for the next gate right now
        self._search_start = 0.0
        self._search_until = 0.0
        self._gates_passed = 0

    # Gate search — remembering the gate just passed

    def _note_gate(self, a, b, line_len, width, now):
        """Record the gate currently in view, and whether it has come close.

        A gate seen briefly at a distance and then lost was never passed — the
        detection just blinked. Only a gate whose connector line grew wide (or whose
        posts measured near) and *then* vanished means the boat went through it.
        """
        if self._last_gate_time is None or (now - self._last_gate_time) > self.gate_grace_sec:
            self._gate_close = False

        self._last_gate_time = now

        near = min([d for d in (a['dist'], b['dist']) if d is not None], default=None)
        if line_len >= self.pass_line_ratio * width or (near is not None and near <= self.pass_depth_m):
            self._gate_close = True

        # A gate is in view, so any sweep has done its job. Gates on this course are
        # independent props — which colour sat on which side of THIS gate says
        # nothing about where the next one is, so no direction is inferred here.
        self._search_active = False

    def _update_search(self, now):
        """Return the sweep direction to steer: -1 left, +1 right, 0 not searching.

        Gates on this course are independent props, placed with no relation to one
        another — which colour sat on which side of the gate just passed says
        nothing about which side the next one is on. So this does not guess: it
        sweeps right for the first half of the search window, then left for the
        second half, and gives up if neither turns up a gate.
        """
        if self._search_active:
            if now >= self._search_until:
                self._search_active = False
                return 0
            leg = self.search_timeout_sec / 2.0
            return 1 if (now - self._search_start) < leg else -1

        if self._last_gate_time is None:
            return 0
        if (now - self._last_gate_time) < self.gate_grace_sec:
            return 0
        if not self._gate_close:
            return 0

        self._search_active = True
        self._search_start = now
        self._search_until = now + self.search_timeout_sec
        self._gate_close = False
        self._gates_passed += 1           # WE PASSED A GATE!
        return 1

    def _reset_search(self):
        self._last_gate_time = None
        self._gate_close = False
        self._search_active = False
        self._search_start = 0.0
        self._search_until = 0.0
        self._gates_passed = 0

    # Steering maths

    def _yaw_from_x(self, cx, width, batas_bawah, batas_tengah, batas_atas):
        """Map a target x pixel to an RC yaw, with the neutral point at batas_tengah.
        Handles asymmetric limits (e.g. 1000 min, 1500 mid, 1700 max).
        """
        offset = (float(cx) - width / 2.0) / (width / 2.0)   # -1 (far left) .. +1 (far right)
        if offset >= 0:
            span = float(batas_atas - batas_tengah)
            yaw = batas_tengah + offset * span
        else:
            span = float(batas_tengah - batas_bawah)
            yaw = batas_tengah + offset * span
        return int(np.clip(yaw, batas_bawah, batas_atas))


    def _yaw_away_from(self, cx, width, batas_bawah, batas_tengah, batas_atas):
        """Steer away from a lone buoy: left buoy -> turn right, and vice versa.

        Strength grows as the buoy approaches the centre line — a buoy dead ahead is
        the one about to be hit, a buoy at the frame edge is already being cleared.
        """
        span = float(batas_atas - batas_tengah)
        offset = (float(cx) - width / 2.0) / (width / 2.0)
        strength = 1.0 - min(1.0, abs(offset))
        yaw = batas_tengah - np.sign(offset) * strength * span
        return int(np.clip(yaw, batas_bawah, batas_atas))

    def _select_gate(self, buoys):
        """Pick the gate to aim at: the adjacent pair with the longest connector line.

        Only ADJACENT buoys pair up. With orange-yellow-orange the candidates are
        (orange, yellow) and (yellow, orange); pairing the two oranges across the
        yellow would describe a "gap" with a buoy sitting in the middle of it.

        Longest line wins, measured between centroids. In pixels a nearer gap is
        also a wider one, so this naturally prefers the gate the boat is closest to.
        """
        if len(buoys) < 2:
            return None

        ordered = sorted(buoys, key=lambda b: b['cx'])

        best = None
        best_len = -1.0
        for a, b in zip(ordered, ordered[1:]):
            line_len = float(np.hypot(b['cx'] - a['cx'], b['cy'] - a['cy']))
            if line_len > best_len:
                best_len = line_len
                best = (a, b)

        return best

    def processor(self, frame, fps, mission, left_buoy, right_buoy, boxes, confs, clss,
                  width, height, batas_bawah, batas_tengah, batas_atas, min_buoy_area,
                  distances=None, max_range=None):
        yaw = batas_tengah

        # Per-class bests, still needed for the score topics and the Kamikaze target.
        max_area  = {k: 0    for k in self.TRACKED_CLASSES}
        max_conf  = {k: 0    for k in self.TRACKED_CLASSES}
        midpoints = {k: None for k in self.TRACKED_CLASSES}

        # Every buoy that survives the filters is a gate-post candidate.
        buoys = []

        for idx, (bb, cf, cl) in enumerate(zip(boxes, confs, clss)):
            cl = int(cl)
            x1, y1, x2, y2 = map(float, bb)
            cls_name = self.cls_dict.get(cl, 'CLS{}'.format(cl))
            cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            area = (x2 - x1) * (y2 - y1)
            dist = distances[idx] if (distances is not None and idx < len(distances)) else None

            # Too small — far away, or a false positive on glare.
            if mission in ("Floating Ball", "Kamikaze"):
                if cls_name in self.BUOY_CLASSES and area < min_buoy_area:
                    continue

            # Measured range beyond the depth camera's usable distance is arena
            # clutter, not this gate.
            if max_range is not None and dist is not None and dist > max_range:
                continue

            if cls_name in max_area and area > max_area[cls_name]:
                max_area[cls_name]  = area
                max_conf[cls_name]  = cf
                midpoints[cls_name] = (int(cx), int(cy))

            if cls_name in self.BUOY_CLASSES:
                buoys.append({'cls_name': cls_name, 'cx': cx, 'cy': cy, 'dist': dist})

        # Mission handlers

        if mission == "Floating Ball":
            now = time.monotonic()
            gate = self._select_gate(buoys)

            if gate is not None:
                a, b = gate
                gate_cx = (a['cx'] + b['cx']) / 2.0
                gate_len = float(np.hypot(b['cx'] - a['cx'], b['cy'] - a['cy']))

                self._note_gate(a, b, gate_len, width, now)

                yaw = self._yaw_from_x(gate_cx, width, batas_bawah, batas_tengah, batas_atas)

            else:
                # No gate in view. If the boat just went through one, the next gap is
                # off to the side the centre buoy was on — sweep that way to bring it
                # into frame.
                search_dir = self._update_search(now)

                if search_dir != 0:
                    # The sweep outranks the lone-buoy rule.
                    yaw = int(np.clip(batas_tengah + search_dir * self.search_yaw_offset,
                                      batas_bawah, batas_atas))

                elif len(buoys) == 1:
                    lone = buoys[0]
                    # Steer AWAY from the lone buoy based purely on its position in
                    # the frame — NOT its colour. _yaw_away_from steers in the
                    # correct direction for any cx: buoy on RIGHT → yaw < 1500
                    # (turn left), buoy on LEFT → yaw > 1500 (turn right).
                    # Strength grows as the buoy approaches dead-centre (most danger).
                    yaw = self._yaw_away_from(lone['cx'], width, batas_bawah, batas_tengah, batas_atas)
                    yaw = int(np.clip(yaw, batas_bawah, batas_atas))


        elif mission == "Kamikaze":
            # Leaving Floating Ball ends any gate hunt.
            self._reset_search()

            # Kamikaze still targets by colour: the planner names the buoy to ram.
            target_buoy = left_buoy
            if target_buoy == "black_buoy" and (midpoints.get("black_buoy") is None) and (midpoints.get("blue_box") is not None):
                target_buoy = "blue_box"

            if target_buoy in midpoints and midpoints[target_buoy] is not None:
                tx, _ = midpoints[target_buoy]
                yaw = self._yaw_from_x(tx, width, batas_bawah, batas_tengah, batas_atas)

        else:
            # Any other mission string (Course 1 legs, empty after a reset).
            self._reset_search()

        # Detection confidence for whatever colour currently sits in each slot. The
        # planner tracks its kamikaze target through the left score, so this is the
        # only feedback path telling it the target is still in frame.
        left_score  = float(max_conf.get(left_buoy, 0.0))
        right_score = float(max_conf.get(right_buoy, 0.0))

        return frame, yaw, left_score, right_score
