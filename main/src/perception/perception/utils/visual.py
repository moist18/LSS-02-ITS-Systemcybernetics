import time
import numpy as np
import cv2

class BBoxVisualization():
    def __init__(self, cls_dict):
        self.cls_dict = cls_dict  # ['blue_box', 'green_box', 'orange_buoy', 'green_dock', 'yellow_buoy', 'red_dock', 'black_buoy']

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
        self.YELLOW_COLOR = (0, 255, 255)
        self.BLACK_COLOR = (0, 0, 0)
        self.WHITE_COLOR = (255, 255, 255)
        self.CYAN_COLOR = (255, 255, 0)

        # Variables
        self.midpoint_deadzone = 50
        self.single_deadzone = int(self.midpoint_deadzone * 4.5)

        # Warna kotak per class. Class tanpa entri di sini digambar biru.
        self.CLASS_COLORS = {
            'orange_buoy': self.ORANGE_COLOR,
            'yellow_buoy': self.YELLOW_COLOR,
            'black_buoy':  self.BLACK_COLOR,
            'red_buoy':    self.RED_COLOR,
            'green_buoy':  self.GREEN_COLOR,
            'red_dock':    self.RED_COLOR,
            'green_dock':  self.GREEN_COLOR,
        }

    # Semua nama class yang boleh masuk bucket processor(). Gabungan set model
    # lama (blue_box/…/red_dock) dan set engine sekarang (red_buoy/green_buoy)
    # supaya satu file visual.py tetap jalan untuk dua-duanya.
    TRACKED_CLASSES = ('blue_box', 'green_box', 'orange_buoy', 'green_dock',
                       'yellow_buoy', 'red_dock', 'black_buoy',
                       'red_buoy', 'green_buoy')

    # HSV colour-blob fallback — Kamikaze ONLY, dipakai hanya kalau YOLO tidak
    # menemukan target di frame ini DAN hsv_fallback_enabled=true. Murah, tidak
    # butuh training, tapi jauh lebih sensitif terhadap cahaya dan tidak bisa
    # membedakan warna bertetangga, jadi tidak pernah menimpa deteksi YOLO.
    # (lower1, upper1, lower2, upper2) — red wraps hue 0/180 so it needs two
    # ranges; green/black only need one, so lower2/upper2 stay None.
    # Format: (lower1, upper1, lower2, upper2)
    # lower2/upper2 = None untuk warna yang tidak wrap H=0/180
    HSV_RANGES = {
        'green_buoy':  ([35,  60,  40],  [85,  255, 255], None,            None          ),
        # H atas dipersempit 10 -> 6 dan H bawah 160 -> 172. Buoy oranye jatuh di
        # hue ~8-22, jadi rentang lama menelan oranye bulat-bulat: blob oranye
        # lolos sebagai 'red_buoy' dan kamikaze mengejar buoy yang salah.
        # S minimum dinaikkan 100 -> 120 supaya pantulan air kemerahan tidak ikut.
        'red_buoy':    ([0,   120, 70],  [6,   255, 255], [172, 120, 70],  [180, 255, 255]),
        'orange_buoy': ([8,   120, 90],  [22,  255, 255], None,            None          ),
        'yellow_buoy': ([23,  100, 100], [34,  255, 255], None,            None          ),
        'black_buoy':  ([0,   0,   0],   [180, 80,  80],  None,            None          ),
    }

    # Warna yang gampang tertukar di HSV. Kalau target merah tapi blob oranye
    # jauh lebih besar di frame yang sama, blob "merah" itu hampir pasti sisi
    # gelap buoy oranye — tolak daripada mengejar target salah.
    HSV_CONFUSABLE = {
        'red_buoy':    'orange_buoy',
        'orange_buoy': 'red_buoy',
    }

    _HSV_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    _last_hsv_debug = 0.0  # throttle state, shared class-wide — fine, single camera loop

    def _mask_for(self, hsv, color_name):
        """Binary mask untuk satu warna, sudah di-open+close.

        Open (buang bintik noise) lalu close (tutup lubang kecil) — sama seperti
        teshsv.py; tanpa itu pixel noise bisa mengalahkan atau memecah kontur
        blob aslinya."""
        lower1, upper1, lower2, upper2 = self.HSV_RANGES[color_name]
        mask = cv2.inRange(hsv, np.array(lower1), np.array(upper1))
        if lower2 is not None:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, np.array(lower2), np.array(upper2)))
        cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._HSV_KERNEL, dst=mask)
        cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._HSV_KERNEL, dst=mask)
        return mask

    def _largest_area(self, hsv, color_name):
        """Luas kontur terbesar untuk satu warna, 0.0 kalau tidak ada."""
        if color_name not in self.HSV_RANGES:
            return 0.0
        contours, _ = cv2.findContours(self._mask_for(hsv, color_name),
                                       cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return max((cv2.contourArea(c) for c in contours), default=0.0)

    def _detect_by_hsv(self, frame, color_name, min_area):
        """Largest matching colour blob: (cx, cy, x1, y1, x2, y2), or None if
        nothing big enough. Bbox included so the caller can actually draw a
        rectangle — a bare centroid dot is easy to miss and gives no sense of
        the blob's size/confidence the way a YOLO box does."""
        if color_name not in self.HSV_RANGES:
            return None

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = self._mask_for(hsv, color_name)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Throttled 1/sec diagnostic — prints to the node's terminal so "why isn't
        # it detecting" has numbers instead of guesses: no contours at all (range/
        # lighting/camera problem) vs a contour that's just under min_area
        # (threshold too high / object too small or far) vs a real hit.
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

        # Tolak blob yang kemungkinan besar milik warna tetangga (merah vs
        # oranye). Rentang hue keduanya bersebelahan, jadi sisi gelap buoy
        # oranye tetap bisa lolos mask merah walau sudah dipersempit.
        rival = self.HSV_CONFUSABLE.get(color_name)
        if rival is not None:
            rival_area = self._largest_area(hsv, rival)
            if rival_area > area * 1.5:
                if debug_due:
                    print(f"[HSV] {color_name}: REJECTED — blob {rival} {int(rival_area)}px² "
                          f"jauh lebih besar dari kandidat {int(area)}px², "
                          f"kemungkinan buoy {rival}", flush=True)
                return None

        m = cv2.moments(largest)
        if m['m00'] == 0:
            return None
        cx, cy = int(m['m10'] / m['m00']), int(m['m01'] / m['m00'])
        x, y, w, h = cv2.boundingRect(largest)
        if debug_due:
            print(f"[HSV] {color_name}: HIT area={int(area)}px² at ({cx},{cy})", flush=True)
        return (cx, cy, x, y, x + w, y + h)

    # Detektor utama Kamikaze. Nama class model bisa beda antar engine: engine
    # baru punya green_buoy/red_buoy, engine lama tidak. Kalau nama persis target
    # tidak ada di cls_dict, jatuh ke alias di bawah supaya target tetap ketemu.
    KAMIKAZE_YOLO_ALIASES = {
        'green_buoy': ('green_buoy', 'green_dock', 'green_box', 'yellow_buoy'),
        'red_buoy':   ('red_buoy',   'red_dock', 'orange_buoy'),
        'black_buoy': ('black_buoy',),
    }

    _last_yolo_debug = 0.0  # throttle state, sama pola dengan _last_hsv_debug

    def _detect_by_yolo(self, target_buoy, boxes, confs, clss, min_area):
        """Box YOLO terbesar yang cocok dengan target Kamikaze:
        (cx, cy, x1, y1, x2, y2, conf, cls_name), atau None.

        Dibaca langsung dari hasil YOLO frame ini — sengaja TIDAK lewat
        midpoints/max_bbox milik processor(), karena dict itu dipakai bersama
        mission lain (Floating Ball) dan sudah kena filter khusus gate (mis.
        orange_prefer_side). Jalur terpisah = nol efek samping ke mission lain."""
        names = set(self.cls_dict.values()) if hasattr(self.cls_dict, 'values') else set()
        if target_buoy in names:
            wanted = (target_buoy,)
        else:
            wanted = self.KAMIKAZE_YOLO_ALIASES.get(target_buoy, (target_buoy,))

        best = None
        best_area = 0.0
        for bb, cf, cl in zip(boxes, confs, clss):
            cls_name = self.cls_dict.get(int(cl), 'CLS{}'.format(int(cl)))
            if cls_name not in wanted:
                continue
            x1, y1, x2, y2 = map(float, bb)
            area = (x2 - x1) * (y2 - y1)
            if area < min_area or area <= best_area:
                continue
            best_area = area
            best = (int((x1 + x2) / 2), int((y1 + y2) / 2),
                    int(x1), int(y1), int(x2), int(y2), float(cf), cls_name)

        # Diagnostik throttle 1 detik — biar terminal menunjukkan detektor mana
        # yang menyala, dan kalau dua-duanya gagal, gagalnya di sisi mana.
        now = time.time()
        if now - type(self)._last_yolo_debug >= 1.0:
            type(self)._last_yolo_debug = now
            if best is None:
                print(f"[YOLO] {target_buoy}: no box matching {wanted} "
                      f"(area >= {min_area}px²) — lanjut ke HSV backup kalau aktif", flush=True)
            else:
                print(f"[YOLO] {target_buoy}: HIT as '{best[7]}' "
                      f"conf={best[6]:.2f} area={int(best_area)}px² at ({best[0]},{best[1]})", flush=True)
        return best

    def processor(self, frame, fps, mission, left_buoy, right_buoy, boxes, confs, clss, width, height, batas_bawah, batas_tengah, batas_atas, min_buoy_area, orange_prefer_side='left', hsv_fallback_enabled=False, hsv_min_area=300):
        self.target_detected = False
        # Detection confidence for the Kamikaze target this frame. Published on
        # /detect/left_buoy/score so a planner can tell "target visible" from
        # "target lost" — without it the score topic has no publisher at all
        # when this module is the active visualiser.
        self.target_score = 0.0
        connector_midpoint = None
        yaw = batas_tengah

        # Bucket per class. Nama yang tidak ada di sini di-drop diam-diam oleh
        # loop di bawah — red_buoy dan green_buoy hilang total karena set ini
        # masih set 7-class model lama, padahal engine sekarang mengeluarkannya.
        max_area = {k: 0    for k in self.TRACKED_CLASSES}
        max_bbox = {k: None for k in self.TRACKED_CLASSES}
        max_conf = {k: 0    for k in self.TRACKED_CLASSES}
        midpoints = {k: None for k in self.TRACKED_CLASSES}

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
                # gate) — steer off ONE side only, picked by orange_prefer_side.
                # The other orange belongs to the far post / next gate; picking by
                # area let it hijack the slot whenever it was bigger/closer,
                # snapping the gate line to the wrong pair. Ignore it outright
                # instead of tracking it at all.
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

        # Bounding boxes with the largest area
        for cls_name, bbox in max_bbox.items():
            if bbox is not None:
                x1, y1, x2, y2 = bbox
                conf = max_conf[cls_name]
                color = self.CLASS_COLORS.get(cls_name, self.BLUE_COLOR)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, self.THICKNESS)
                cv2.circle(frame, midpoints[cls_name], self.CIRCLE_SIZE, self.RED_COLOR, -1)
                cv2.putText(frame, f"{cls_name} {conf:.2f}", (x1, y1-10), self.FONT, self.FONT_SIZE, color, self.THICKNESS)

        # Connection line and midpoint
        if left_buoy in midpoints and right_buoy in midpoints and midpoints[left_buoy] and midpoints[right_buoy]:
            connector_midpoint = (int((midpoints[left_buoy][0] + midpoints[right_buoy][0]) / 2),
                                  int((midpoints[left_buoy][1] + midpoints[right_buoy][1]) / 2))

        # Mission Handler
        if mission == "Floating Ball":
            if connector_midpoint and left_buoy and right_buoy:
                a, b = connector_midpoint
                yaw = batas_bawah + a * ((batas_atas - batas_bawah) * (1/640))

                cv2.line(frame, midpoints[left_buoy], midpoints[right_buoy], self.WHITE_COLOR, self.THICKNESS)
                cv2.line(frame, (int(width/2), int(height/2)), connector_midpoint, self.WHITE_COLOR, self.THICKNESS)
                cv2.circle(frame, connector_midpoint, self.CIRCLE_SIZE, self.RED_COLOR, -1)

                if (a < int(width/2) - self.midpoint_deadzone):
                    cv2.putText(frame, "TURN", (560, 30), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
                    cv2.putText(frame, "LEFT", (560, 60), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
                elif (a > int(width/2) + self.midpoint_deadzone):
                    cv2.putText(frame, "TURN", (560, 30), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
                    cv2.putText(frame, "RIGHT", (560, 60), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)

            elif left_buoy in midpoints and midpoints[left_buoy] is not None:
                a, b = midpoints[left_buoy]
                yaw = batas_tengah + a * ((batas_tengah - batas_bawah) * (1/640))

                cv2.line(frame, (int(width/2), int(height/2)), midpoints[left_buoy], self.WHITE_COLOR, self.THICKNESS)
                cv2.circle(frame, midpoints[left_buoy], self.CIRCLE_SIZE, self.RED_COLOR, -1)

                if a > int(width/2) - self.single_deadzone:
                    cv2.putText(frame, "TURN", (560, 30), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
                    cv2.putText(frame, "RIGHT", (560, 60), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
                elif a < int(width/2) - self.single_deadzone:
                    cv2.putText(frame, "TURN", (560, 30), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
                    cv2.putText(frame, "LEFT", (560, 60), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)

            elif right_buoy in midpoints and midpoints[right_buoy] is not None:
                a, b = midpoints[right_buoy]
                yaw = batas_bawah + a * ((batas_tengah - batas_bawah) * (1/640))

                cv2.line(frame, (int(width/2), int(height/2)), midpoints[right_buoy], self.WHITE_COLOR, self.THICKNESS)
                cv2.circle(frame, midpoints[right_buoy], self.CIRCLE_SIZE, self.RED_COLOR, -1)

                if a < int(width/2) + self.single_deadzone:
                    cv2.putText(frame, "TURN", (560, 30), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
                    cv2.putText(frame, "LEFT", (560, 60), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)

        elif mission == "Kamikaze":
            target_buoy = left_buoy

            # ── YOLO utama, HSV backup ───────────────────────────────────────
            # Urutannya sengaja YOLO dulu: model bisa memisahkan red_buoy dari
            # orange_buoy, HSV tidak — hue merah dan oranye bersebelahan, jadi
            # HSV-duluan mengunci buoy oranye padahal target red_buoy (persis
            # gejala "kamikaze malah kejar orange"). HSV baru dipakai kalau YOLO
            # kosong DAN hsv_fallback_enabled=true; sebelumnya flag itu diterima
            # tapi tidak pernah dibaca, jadi HSV selalu jalan.
            hit = self._detect_by_yolo(target_buoy, boxes, confs, clss, min_buoy_area)

            if hit is not None:
                a, b, tx1, ty1, tx2, ty2, tcf, tcls = hit
                self.target_score = float(tcf)
                # Overlay CYAN = YOLO (ungu = HSV backup), supaya di video
                # keliatan detektor mana yang lagi memandu.
                overlay = self.CYAN_COLOR
                box_label = f"{tcls} YOLO {tcf:.2f}"
                mode_label = f"YOLO  {target_buoy}"
            elif hsv_fallback_enabled:
                hsv_hit = self._detect_by_hsv(frame, target_buoy, hsv_min_area)
                if hsv_hit is not None:
                    a, b, tx1, ty1, tx2, ty2 = hsv_hit
                    # HSV tidak punya confidence sendiri — blob yang lolos filter
                    # luas dihitung hit penuh.
                    self.target_score = 1.0
                    overlay = self.PURPLE_COLOR
                    box_label = f"{target_buoy} HSV"
                    mode_label = f"HSV-BACKUP  {target_buoy}"
                    hit = hsv_hit
                else:
                    hit = None

            if hit is not None:
                self.target_detected = True
                centroid = (a, b)
                yaw = batas_bawah + a * ((batas_atas - batas_bawah) * (1 / 640))

                cv2.rectangle(frame, (tx1, ty1), (tx2, ty2), overlay, self.THICKNESS)
                cv2.putText(frame, box_label, (tx1, max(15, ty1 - 8)),
                            self.FONT, 0.5, overlay, self.THICKNESS)
                cv2.line(frame, (int(width / 2), int(height / 2)), centroid, overlay, self.THICKNESS)
                cv2.circle(frame, centroid, self.CIRCLE_SIZE, overlay, -1)
                cv2.putText(frame, mode_label, (10, height - 20),
                            self.FONT, 0.6, overlay, self.THICKNESS)

                if a < int(width / 2) - self.midpoint_deadzone:
                    cv2.putText(frame, "TURN", (560, 30), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
                    cv2.putText(frame, "LEFT", (560, 60), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
                elif a > int(width / 2) + self.midpoint_deadzone:
                    cv2.putText(frame, "TURN",  (560, 30), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
                    cv2.putText(frame, "RIGHT", (560, 60), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
            else:
                # YOLO kosong DAN HSV kosong/mati → tidak ada target → yaw tengah
                cv2.putText(frame, f"{target_buoy}  NOT FOUND", (10, height - 20),
                            self.FONT, 0.6, self.RED_COLOR, self.THICKNESS)

        # Deadzone grid area
        cv2.circle(frame, (int(width/2), int(height/2)), self.CIRCLE_SIZE, self.RED_COLOR, -1)
        cv2.line(frame, (int(width/2)-self.single_deadzone, 0), (int(width/2)-self.single_deadzone, height), self.GREEN_COLOR, self.THICKNESS)
        cv2.line(frame, (int(width/2)+self.single_deadzone, 0), (int(width/2)+self.single_deadzone, height), self.GREEN_COLOR, self.THICKNESS)
        cv2.line(frame, (int(width/2)-self.midpoint_deadzone, 0), (int(width/2)-self.midpoint_deadzone, height), self.BLUE_COLOR, self.THICKNESS)
        cv2.line(frame, (int(width/2)+self.midpoint_deadzone, 0), (int(width/2)+self.midpoint_deadzone, height), self.BLUE_COLOR, self.THICKNESS)
        cv2.line(frame, (0,int(height/2)-self.midpoint_deadzone), (width, int(height/2)-self.midpoint_deadzone), self.RED_COLOR, self.THICKNESS)
        cv2.line(frame, (0,int(height/2)+self.midpoint_deadzone), (width, int(height/2)+self.midpoint_deadzone), self.RED_COLOR, self.THICKNESS)

        # FPS and YAW value
        cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
        cv2.putText(frame, f"YAW: {int(yaw)}", (10, 60), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)

        return frame, yaw
