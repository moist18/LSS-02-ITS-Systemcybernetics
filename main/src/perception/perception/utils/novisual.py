import numpy as np
import cv2

class BBoxVisualization():
    def __init__(self, cls_dict):
        self.cls_dict = cls_dict  # ['blue_box', 'green_box', 'green_buoy', 'green_dock', 'red_buoy', 'red_dock']

    def processor (self, frame, fps, mission, docking, left_buoy, right_buoy, boxes, confs, clss, width, height, batas_bawah, batas_tengah, batas_atas):
        is_finish = False
        connector_midpoint = None
        yaw = batas_tengah
        
        max_area = {
            'blue_box': 0,
            'green_box': 0,
            'green_buoy': 0,
            'green_dock': 0,
            'red_buoy': 0,
            'red_dock': 0
        }
        max_bbox = {
            'blue_box': None,
            'green_box': None,
            'green_buoy': None,
            'green_dock': None,
            'red_buoy': None,
            'red_dock': None
        }
        max_conf = {
            'blue_box': 0,
            'green_box': 0,
            'green_buoy': 0,
            'green_dock': 0,
            'red_buoy': 0,
            'red_dock': 0
        }
        midpoints = {
            'blue_box': None,
            'green_box': None,
            'green_buoy': None,
            'green_dock': None,
            'red_buoy': None,
            'red_dock': None
        }
        
        # Detected object
        for bb, cf, cl in zip(boxes, confs, clss):
            cl = int(cl)
            x1, y1, x2, y2 = map(float, bb)
            cls_name = self.cls_dict.get(cl, 'CLS{}'.format(cl))
            midpoint = (int((x1 + x2) / 2), int((y1 + y2) / 2))
            area = (x2 - x1) * (y2 - y1)

            if cls_name in max_area and area > max_area[cls_name]:
                max_area[cls_name] = area
                max_bbox[cls_name] = (int(x1), int(y1), int(x2), int(y2))
                max_conf[cls_name] = cf
                midpoints[cls_name] = midpoint

        # Connection line and midpoint
        if midpoints[left_buoy] and midpoints[right_buoy]:
            connector_midpoint = (int((midpoints[left_buoy][0] + midpoints[right_buoy][0]) / 2),
                                  int((midpoints[left_buoy][1] + midpoints[right_buoy][1]) / 2))
        
        # Mission Handler
        if mission == "Floating Ball":
            if connector_midpoint and left_buoy and right_buoy:
                a, b = connector_midpoint
                yaw = batas_bawah + a * ((batas_atas - batas_bawah) * (1/640))

            elif midpoints[left_buoy] is not None:
                a, b = midpoints[left_buoy]
                yaw = batas_tengah + a * ((batas_tengah - batas_bawah) * (1/640))

            elif midpoints[right_buoy] is not None:
                a, b = midpoints[right_buoy]
                yaw = batas_bawah + a * ((batas_tengah - batas_bawah) * (1/640))
                
        # elif mission.endswith("Imaging"):
        #     if midpoints["green_box"] is not None:
        #         a, b = midpoints["green_box"]
        #         yaw = batas_bawah + a * ((batas_atas - batas_bawah) * (1/640))

        # SUBTITUTION OF 'mission.endswith("Imaging")' IF 'blue_box' CLASS EXIST

        elif mission == "Surface Imaging":
            if midpoints["green_box"] is not None:
                a, b = midpoints["green_box"]
                yaw = batas_bawah + a * ((batas_atas - batas_bawah) * (1/640))

        elif mission == "Underwater Imaging":
            if midpoints["blue_box"] is not None:
                a, b = midpoints["blue_box"]
                yaw = batas_bawah + a * ((batas_atas - batas_bawah) * (1/640))

        elif mission == "Docking":
            is_finish = True
            if midpoints[docking] is not None:
                a, b = midpoints[docking]
                yaw = batas_bawah + a * ((batas_atas - batas_bawah) * (1/640))
        
        return frame, yaw, is_finish