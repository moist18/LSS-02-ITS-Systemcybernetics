import time

import numpy as np
import cv2

class BBoxVisualization():
    """Midpoint steering with drawing overlay, for the D455 depth-camera node.

    Split out of visual.py so vision.py (plain USB camera) keeps running as it did.

    Steering is GEOMETRIC, not colour-slot based. The arena has three buoys —
    orange left, yellow centre, orange right — which the old colour-slot model
    cannot represent: it keeps one representative per colour, so the two oranges
    collapse into one and the gate pair jumps between the left and right gap as the
    boat moves. Worse, an orange appearing on the right still used the left-slot
    formula and steered *into* it (measured in the field: raw_yaw pinned at 1700).

    Here every buoy is a candidate. Adjacent buoys (sorted left to right) form
    candidate gates, and the gate with the longest connector line wins. Colour is
    still reported back on the score topics, but it no longer decides where to steer.
    """

    # Buoy classes that can act as a gate post.
    BUOY_CLASSES = ['green_buoy', 'red_buoy', 'black_buoy', 'orange_buoy', 'yellow_buoy']

    # Everything the Kamikaze branch and the score topics can address by name.
    TRACKED_CLASSES = BUOY_CLASSES + ['blue_box', 'green_box', 'green_dock', 'red_dock']

    CLASS_COLORS = {
        'green_buoy':  (0, 255, 0),
        'red_buoy':    (0, 0, 255),
        'black_buoy':  (0, 0, 0),
        'orange_buoy': (0, 125, 255),
        'yellow_buoy': (0, 255, 255),
        'green_dock':  (0, 200, 100),
        'red_dock':    (0, 80, 255),
        'green_box':   (100, 255, 100),
        'blue_box':    (255, 155, 0),
    }

    def __init__(self, cls_dict, logger=None):
        self.cls_dict = cls_dict

        # Diagnostic logging — pass the node's ROS logger in to get a throttled
        # dump of what the detector actually saw each frame (kept vs filtered
        # buoys, and why) plus the steering decision. None disables it.
        self.logger = logger
        self._log_interval = 0.5
        self._last_log = 0.0

        # Constants
        self.FONT = cv2.FONT_HERSHEY_SIMPLEX
        self.FONT_SIZE = 0.75
        self.THICKNESS = 2
        self.CIRCLE_SIZE = 7
        self.BLUE_COLOR = (255, 155, 0)
        self.GREEN_COLOR = (0, 255, 0)
        self.RED_COLOR = (0, 0, 255)
        self.PURPLE_COLOR = (255, 55, 125)
        self.ORANGE_COLOR = (0, 125, 255)
        self.BLACK_COLOR = (0, 0, 0)
        self.WHITE_COLOR = (255, 255, 255)
        self.CYAN_COLOR = (255, 255, 0)

        # Variables
        self.midpoint_deadzone = 50
        self.single_deadzone = int(self.midpoint_deadzone * 4.5)

        # Gate search tuning — after a gate is passed, the next one is usually
        # outside the field of view, because the boat leaves the gate pointing
        # wherever the gate pointed it. These control the sweep that goes looking
        # for it. Tunable live from the node.
        self.pass_line_ratio = 0.60        # gate counts as "close" once its connector spans this much of the frame
        self.pass_depth_m = 2.0            # ...or once either post is measured this near
        self.search_timeout_sec = 6.0      # how long to sweep before giving up
        self.search_yaw_offset = 120       # RC units from mid while sweeping
        self.gate_grace_sec = 0.5          # a gate lost for less than this may still come back

        # Gate search state
        self._last_gate_time = None        # when a gate was last seen
        self._gate_close = False           # did that gate ever get close enough to count as passed
        self._search_active = False        # sweeping for the next gate right now
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

    def _yaw_pass_side(self, cx, pass_side, width, batas_bawah, batas_tengah, batas_atas):
        """Steer to pass on a specific side of a lone buoy.
        pass_side: 'right' or 'left'
        """
        margin = width * 0.25  # 160 pixels for 640 width
        if pass_side == 'right':
            target_x = cx + margin
        else:
            target_x = cx - margin
        return self._yaw_from_x(target_x, width, batas_bawah, batas_tengah, batas_atas)



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

    # Gate search — remembering the gate just passed

    def _note_gate(self, a, b, line_len, width, now):
        """Record the gate currently in view, and whether it has come close.

        A gate seen briefly at a distance and then lost was never passed — the
        detection just blinked. Only a gate whose connector line grew wide (or whose
        posts measured near) and *then* vanished means the boat went through it.
        """
        # Gone for longer than the grace period, so this is a different gate:
        # start judging its closeness from scratch.
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
                self._search_active = False   # gave up; fall back to the normal rules
                return 0
            leg = self.search_timeout_sec / 2.0
            return 1 if (now - self._search_start) < leg else -1

        if self._last_gate_time is None:
            return 0
        if (now - self._last_gate_time) < self.gate_grace_sec:
            return 0                      # might just be a blink
        if not self._gate_close:
            return 0                      # was never close enough to have been passed

        self._search_active = True
        self._search_start = now
        self._search_until = now + self.search_timeout_sec
        self._gate_close = False          # consume, so one passed gate triggers one sweep
        self._gates_passed += 1           # WE PASSED A GATE!
        return 1

    def _reset_search(self):
        self._last_gate_time = None
        self._gate_close = False
        self._search_active = False
        self._search_start = 0.0
        self._search_until = 0.0
        self._gates_passed = 0

    def _log(self, now, msg):
        """Throttled diagnostic log — no-op if no logger was given."""
        if self.logger is None:
            return
        if now - self._last_log < self._log_interval:
            return
        self._last_log = now
        self.logger.info(msg)

    def processor(self, frame, fps, mission, left_buoy, right_buoy, boxes, confs, clss,
                  width, height, batas_bawah, batas_tengah, batas_atas, min_buoy_area,
                  distances=None, max_range=None):
        yaw = batas_tengah
        status = ""
        now = time.monotonic()

        # Per-class bests, still needed for the score topics and the Kamikaze target.
        max_area  = {k: 0    for k in self.TRACKED_CLASSES}
        max_conf  = {k: 0    for k in self.TRACKED_CLASSES}
        midpoints = {k: None for k in self.TRACKED_CLASSES}

        # Every buoy that survives the filters is a gate-post candidate.
        buoys = []

        # What the detector actually produced this frame, kept or dropped and
        # why — this is the part worth logging when steering looks wrong,
        # since a bad decision downstream is usually a missing/filtered buoy
        # upstream, not a bug in the gate/search maths.
        raw_dets = []

        for idx, (bb, cf, cl) in enumerate(zip(boxes, confs, clss)):
            cl = int(cl)
            x1, y1, x2, y2 = map(float, bb)
            cls_name = self.cls_dict.get(cl, 'CLS{}'.format(cl))
            cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            area = (x2 - x1) * (y2 - y1)
            dist = distances[idx] if (distances is not None and idx < len(distances)) else None

            drop_reason = None

            # Too small — far away, or a false positive on glare.
            if mission in ("Floating Ball", "Kamikaze"):
                if cls_name in self.BUOY_CLASSES and area < min_buoy_area:
                    drop_reason = f"area {int(area)}<{int(min_buoy_area)}"

            # Measured range beyond the depth camera's usable distance is arena
            # clutter, not this gate.
            if drop_reason is None and max_range is not None and dist is not None and dist > max_range:
                drop_reason = f"range {dist:.1f}m>{max_range:.1f}m"

            side = "L" if cx < width / 2 else "R"
            det_str = f"{cls_name}({side},{int(cx)},{cf:.2f}"
            det_str += f",{dist:.1f}m)" if dist is not None else ")"
            if drop_reason:
                det_str += f"[DROP:{drop_reason}]"
            raw_dets.append(det_str)

            if drop_reason is not None:
                continue

            if cls_name in max_area and area > max_area[cls_name]:
                max_area[cls_name]  = area
                max_conf[cls_name]  = cf
                midpoints[cls_name] = (int(cx), int(cy))

            if cls_name in self.BUOY_CLASSES:
                buoys.append({
                    'cls_name': cls_name, 'conf': float(cf), 'dist': dist,
                    'cx': cx, 'cy': cy,
                    'x1': int(x1), 'y1': int(y1), 'x2': int(x2), 'y2': int(y2),
                })

        # Draw every candidate, not just one per colour — with three buoys and two
        # of them the same colour, a per-colour overlay hides the one that lost.
        for b in buoys:
            color = self.CLASS_COLORS.get(b['cls_name'], self.BLUE_COLOR)
            cv2.rectangle(frame, (b['x1'], b['y1']), (b['x2'], b['y2']), color, self.THICKNESS)
            cv2.circle(frame, (int(b['cx']), int(b['cy'])), self.CIRCLE_SIZE, self.RED_COLOR, -1)

            label = f"{b['cls_name']} {b['conf']:.2f}"
            if b['dist'] is not None and b['dist'] > 0:
                label += f" {b['dist']:.2f}m"
            cv2.putText(frame, label, (b['x1'], max(15, b['y1'] - 10)),
                        self.FONT, 0.5, color, self.THICKNESS)

        # Mission handlers

        if mission == "Floating Ball":
            now = time.monotonic()
            gate = self._select_gate(buoys)

            if gate is not None:
                a, b = gate
                gate_cx = (a['cx'] + b['cx']) / 2.0
                gate_cy = (a['cy'] + b['cy']) / 2.0
                gate_len = float(np.hypot(b['cx'] - a['cx'], b['cy'] - a['cy']))

                self._note_gate(a, b, gate_len, width, now)

                yaw = self._yaw_from_x(gate_cx, width, batas_bawah, batas_tengah, batas_atas)
                status = f"GATE {a['cls_name'][:3]}-{b['cls_name'][:3]} w={int(gate_len)}px"
                if self._gate_close:
                    status += " CLOSE"

                cv2.line(frame, (int(a['cx']), int(a['cy'])), (int(b['cx']), int(b['cy'])),
                         self.WHITE_COLOR, self.THICKNESS)
                cv2.line(frame, (int(width / 2), int(height / 2)), (int(gate_cx), int(gate_cy)),
                         self.WHITE_COLOR, self.THICKNESS)
                cv2.circle(frame, (int(gate_cx), int(gate_cy)), self.CIRCLE_SIZE, self.RED_COLOR, -1)

            else:
                # No gate in view. If the boat just went through one, the next gap is
                # off to the side the centre buoy was on — sweep that way to bring it
                # into frame, because the boat leaves a gate pointing wherever the
                # gate pointed it, not at the next one.
                search_dir = self._update_search(now)

                if search_dir != 0:
                    # The sweep outranks the lone-buoy rule: a consistent heading
                    # search beats reacting to whichever post is still in frame —
                    # UNLESS the sweep is heading straight at the one buoy still in
                    # view (same side), which would drive the boat into it and out
                    # of the course. Flip the sweep away from that buoy instead;
                    # if the lone buoy is on the opposite side, the sweep is
                    # already moving away from it, so leave it alone.
                    if len(buoys) == 1:
                        lone_side = 1 if buoys[0]['cx'] >= width / 2 else -1
                        if search_dir == lone_side:
                            search_dir = -search_dir

                    yaw = int(np.clip(batas_tengah + search_dir * self.search_yaw_offset,
                                      batas_bawah, batas_atas))
                    remaining = max(0.0, self._search_until - now)
                    status = f"SEARCH {'RIGHT' if search_dir > 0 else 'LEFT'} ({remaining:.1f}s)"

                    # Arrow along the sweep direction, drawn from the frame centre.
                    tip = (int(width / 2 + search_dir * width / 4), int(height / 2))
                    cv2.arrowedLine(frame, (int(width / 2), int(height / 2)), tip,
                                    self.PURPLE_COLOR, 3, tipLength=0.3)

                elif len(buoys) == 1:
                    lone = buoys[0]
                    # Steer AWAY from the lone buoy based purely on its position in
                    # the frame — NOT its colour. _yaw_away_from steers in the
                    # correct direction for any cx: buoy on RIGHT → yaw < 1500
                    # (turn left), buoy on LEFT → yaw > 1500 (turn right).
                    # Strength grows as the buoy approaches dead-centre (most danger).
                    lone_on_right = lone['cx'] >= width / 2
                    yaw = self._yaw_away_from(lone['cx'], width, batas_bawah, batas_tengah, batas_atas)
                    yaw = int(np.clip(yaw, batas_bawah, batas_atas))
                    side = "RIGHT" if lone_on_right else "LEFT"
                    status = f"SINGLE {lone['cls_name'][:3]} {side} -> {'LEFT' if lone_on_right else 'RIGHT'}"

                    cv2.line(frame, (int(width / 2), int(height / 2)),
                             (int(lone['cx']), int(lone['cy'])), self.CYAN_COLOR, self.THICKNESS)


                else:
                    status = "NO BUOY (straight)"

        elif mission == "Kamikaze":
            # Leaving Floating Ball ends any gate hunt.
            self._reset_search()

            # Kamikaze still targets by colour: the planner names the buoy to ram.
            target_buoy = left_buoy
            if target_buoy == "black_buoy" and (midpoints.get("black_buoy") is None) and (midpoints.get("blue_box") is not None):
                target_buoy = "blue_box"

            if target_buoy in midpoints and midpoints[target_buoy] is not None:
                tx, ty = midpoints[target_buoy]
                yaw = self._yaw_from_x(tx, width, batas_bawah, batas_tengah, batas_atas)
                status = f"KAMIKAZE {target_buoy}"

                cv2.line(frame, (int(width / 2), int(height / 2)), (tx, ty),
                         self.WHITE_COLOR, self.THICKNESS)
                cv2.circle(frame, (tx, ty), self.CIRCLE_SIZE, self.RED_COLOR, -1)
            else:
                status = "KAMIKAZE no target"

        else:
            # Any other mission string (Course 1 legs, empty after a reset).
            self._reset_search()

        # Turn hint, derived from the yaw actually produced so the overlay can never
        # disagree with the command.
        if yaw < batas_tengah - self.midpoint_deadzone:
            turn = "LEFT"
        elif yaw > batas_tengah + self.midpoint_deadzone:
            turn = "RIGHT"
        else:
            turn = None

        if turn:
            cv2.putText(frame, "TURN", (560, 30), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
            cv2.putText(frame, turn,   (560, 60), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)

        # Deadzone grid area
        cv2.circle(frame, (int(width / 2), int(height / 2)), self.CIRCLE_SIZE, self.RED_COLOR, -1)
        cv2.line(frame, (int(width/2)-self.single_deadzone, 0), (int(width/2)-self.single_deadzone, height), self.GREEN_COLOR, self.THICKNESS)
        cv2.line(frame, (int(width/2)+self.single_deadzone, 0), (int(width/2)+self.single_deadzone, height), self.GREEN_COLOR, self.THICKNESS)
        cv2.line(frame, (int(width/2)-self.midpoint_deadzone, 0), (int(width/2)-self.midpoint_deadzone, height), self.BLUE_COLOR, self.THICKNESS)
        cv2.line(frame, (int(width/2)+self.midpoint_deadzone, 0), (int(width/2)+self.midpoint_deadzone, height), self.BLUE_COLOR, self.THICKNESS)
        cv2.line(frame, (0,int(height/2)-self.midpoint_deadzone), (width, int(height/2)-self.midpoint_deadzone), self.RED_COLOR, self.THICKNESS)
        cv2.line(frame, (0,int(height/2)+self.midpoint_deadzone), (width, int(height/2)+self.midpoint_deadzone), self.RED_COLOR, self.THICKNESS)

        # HUD
        cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
        cv2.putText(frame, f"YAW: {int(yaw)}", (10, 60), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
        if status:
            cv2.putText(frame, status, (10, height - 20), self.FONT, 0.6, self.CYAN_COLOR, self.THICKNESS)

        # Detection confidence for whatever colour currently sits in each slot. The
        # planner tracks its kamikaze target through the left score, so this is the
        # only feedback path telling it the target is still in frame.
        left_score  = float(max_conf.get(left_buoy, 0.0))
        right_score = float(max_conf.get(right_buoy, 0.0))

        # raw_dets shows every box YOLO/NMS actually handed back this frame — class,
        # side, conf, and why it was dropped if it was — vs. len(buoys), what survived
        # into gate/single-buoy logic. Two same-coloured boxes where one should be
        # yellow points at a class-index mismatch; a post that vanishes only when the
        # gate gets close points at NMS collapsing overlapping boxes of the same class.
        self._log(now, f"DETS[{len(raw_dets)}→{len(buoys)}] " + " ".join(raw_dets) +
                        f" | {status or 'no status'} yaw={int(yaw)}")

        return frame, yaw, left_score, right_score
