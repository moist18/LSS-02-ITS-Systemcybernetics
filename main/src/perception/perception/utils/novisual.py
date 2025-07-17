import numpy as np
import cv2

class BBoxVisualization():
    def __init__(self, cls_dict):
        self.cls_dict = cls_dict  # ['blue_box', 'green_box', 'green_buoy', 'green_dock', 'red_buoy', 'red_dock']

    def processor(self, frame, fps, mission, docking, left_buoy, right_buoy, boxes, confs, clss, width, height, batas_bawah, batas_tengah, batas_atas):
        is_finish = False
        connector_midpoint = None
        detected_cls = ''
        yaw = 1500
        
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
                midpoints[cls_name] = midpoint
        
        # Bounding boxes with the largest area
        for cls_name, bbox in max_bbox.items():
            if bbox is not None:
                x1, y1, x2, y2 = bbox
                detected_cls = cls_name
        
        # Connection line and midpoint
        if midpoints['green_buoy'] and midpoints['red_buoy']:
            connector_midpoint = (int((midpoints['green_buoy'][0] + midpoints['red_buoy'][0]) / 2), 
                                  int((midpoints['green_buoy'][1] + midpoints['red_buoy'][1]) / 2))
        
        # Mission Handler
        if mission == "Floating Ball":
            if connector_midpoint and left_buoy and right_buoy:
                a, b = connector_midpoint
                yaw = batas_bawah + a * ((batas_atas - batas_bawah) * (1/640))
                    
            elif detected_cls == left_buoy:
                a, b = midpoints[detected_cls]
                yaw = batas_tengah + a * ((batas_tengah - batas_bawah) * (1/640))
            
            elif detected_cls == right_buoy:
                a, b = midpoints[detected_cls]
                yaw = batas_bawah + a * ((batas_tengah - batas_bawah) * (1/640))
                
        # elif mission.endswith("Imaging"):
        #     if detected_cls == "green_box":
        #         a, b = midpoints[detected_cls]
        #         yaw = batas_bawah + a * ((batas_atas - batas_bawah) * (1/640))
        
        # SUBTITUTION OF 'mission.endswith("Imaging")' IF 'blue_box' CLASS EXIST
        
        elif mission == "Surface Imaging":
            if detected_cls == "green_box":
                a, b = midpoints[detected_cls]
                yaw = batas_bawah + a * ((batas_atas - batas_bawah) * (1/640))
        
        elif mission == "Underwater Imaging":
            if detected_cls == "blue_box":
                a, b = midpoints[detected_cls]
                yaw = batas_bawah + a * ((batas_atas - batas_bawah) * (1/640))
        
        elif mission == "Docking":
            is_finish = True
            if detected_cls == docking:
                a, b = midpoints[detected_cls]
                yaw = batas_bawah + a * ((batas_atas - batas_bawah) * (1/640))
        
        return frame, yaw, is_finish