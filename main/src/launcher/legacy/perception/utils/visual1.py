import numpy as np
import cv2

class BBoxVisualization():
    def __init__ (self, cls_dict):
        self.cls_dict = cls_dict  # ['blue_box', 'green_box', 'green_buoy', 'green_dock', 'red_buoy', 'red_dock']
        
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
        
        # Variables
        self.midpoint_deadzone = 50
        self.single_deadzone = int(self.midpoint_deadzone * 4.5)

    def processor(self, frame, fps, mission, docking, left_buoy, right_buoy, boxes, confs, clss, width, height, batas_bawah, batas_tengah, batas_atas):
        is_finish = False
        connector_midpoint = None
        detected_cls = ''
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
        
        # Bounding boxes with the largest area
        for cls_name, bbox in max_bbox.items():
            if bbox is not None:
                x1, y1, x2, y2 = bbox
                detected_cls = cls_name
                conf = max_conf[cls_name]
                color = self.GREEN_COLOR if cls_name == 'green_buoy' else self.RED_COLOR if cls_name == 'red_buoy' else self.BLUE_COLOR
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, self.THICKNESS)
                cv2.circle(frame, midpoints[cls_name], self.CIRCLE_SIZE, self.RED_COLOR, -1)
                cv2.putText(frame, f"{cls_name} {conf:.2f}", (x1, y1-10), self.FONT, self.FONT_SIZE, color, self.THICKNESS)
        
        # Connection line and midpoint
        if midpoints['green_buoy'] and midpoints['red_buoy']:
            connector_midpoint = (int((midpoints['green_buoy'][0] + midpoints['red_buoy'][0]) / 2), 
                                  int((midpoints['green_buoy'][1] + midpoints['red_buoy'][1]) / 2))
        
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
                    
            elif detected_cls == left_buoy and midpoints[detected_cls] is not None:
                a, b = midpoints[detected_cls]
                yaw = batas_tengah + a * ((batas_tengah - batas_bawah) * (1/640))

                cv2.line(frame, (int(width/2), int(height/2)), midpoints[detected_cls], self.WHITE_COLOR, self.THICKNESS)
                cv2.circle(frame, midpoints[detected_cls], self.CIRCLE_SIZE, self.RED_COLOR, -1)

                if a > int(width/2) - self.single_deadzone:
                    cv2.putText(frame, "TURN", (560, 30), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
                    cv2.putText(frame, "RIGHT", (560, 60), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)

            elif detected_cls == right_buoy and midpoints[detected_cls] is not None:
                a, b = midpoints[detected_cls]
                yaw = batas_bawah + a * ((batas_tengah - batas_bawah) * (1/640))

                cv2.line(frame, (int(width/2), int(height/2)), midpoints[detected_cls], self.WHITE_COLOR, self.THICKNESS)
                cv2.circle(frame, midpoints[detected_cls], self.CIRCLE_SIZE, self.RED_COLOR, -1)

                if a < int(width/2) + self.single_deadzone:
                    cv2.putText(frame, "TURN", (560, 30), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
                    cv2.putText(frame, "LEFT", (560, 60), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
                
        # elif mission.endswith("Imaging"):
        #     if detected_cls == "green_box":
        #         a, b = midpoints[detected_cls]
        #         yaw = batas_bawah + a * ((batas_atas - batas_bawah) * (1/640))
        #        
        #         cv2.line(frame, (int(width/2), int(height/2)), midpoints[detected_cls], self.WHITE_COLOR, self.THICKNESS)
        #         cv2.circle(frame, midpoints[detected_cls], self.CIRCLE_SIZE, self.RED_COLOR, -1)
        #        
        #         if (a < int(width/2) - self.midpoint_deadzone):
        #             cv2.putText(frame, "TURN", (560, 30), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
        #             cv2.putText(frame, "LEFT", (560, 60), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
        #         elif (a > int(width/2) + self.midpoint_deadzone):
        #             cv2.putText(frame, "TURN", (560, 30), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
        #             cv2.putText(frame, "RIGHT", (560, 60), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
        
        # SUBTITUTION OF 'mission.endswith("Imaging")' IF 'blue_box' CLASS EXIST
        
        elif mission == "Surface Imaging":
            if detected_cls == "green_box" and midpoints[detected_cls] is not None:
                a, b = midpoints[detected_cls]
                yaw = batas_bawah + a * ((batas_atas - batas_bawah) * (1/640))

                cv2.line(frame, (int(width/2), int(height/2)), midpoints[detected_cls], self.WHITE_COLOR, self.THICKNESS)
                cv2.circle(frame, midpoints[detected_cls], self.CIRCLE_SIZE, self.RED_COLOR, -1)

                if (a < int(width/2) - self.midpoint_deadzone):
                    cv2.putText(frame, "TURN", (560, 30), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
                    cv2.putText(frame, "LEFT", (560, 60), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
                elif (a > int(width/2) + self.midpoint_deadzone):
                    cv2.putText(frame, "TURN", (560, 30), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
                    cv2.putText(frame, "RIGHT", (560, 60), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)

        elif mission == "Underwater Imaging":
            if detected_cls == "blue_box" and midpoints[detected_cls] is not None:
                a, b = midpoints[detected_cls]
                yaw = batas_bawah + a * ((batas_atas - batas_bawah) * (1/640))

                cv2.line(frame, (int(width/2), int(height/2)), midpoints[detected_cls], self.WHITE_COLOR, self.THICKNESS)
                cv2.circle(frame, midpoints[detected_cls], self.CIRCLE_SIZE, self.RED_COLOR, -1)

                if (a < int(width/2) - self.midpoint_deadzone):
                    cv2.putText(frame, "TURN", (560, 30), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
                    cv2.putText(frame, "LEFT", (560, 60), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
                elif (a > int(width/2) + self.midpoint_deadzone):
                    cv2.putText(frame, "TURN", (560, 30), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
                    cv2.putText(frame, "RIGHT", (560, 60), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)

        elif mission == "Docking":
            is_finish = True
            if detected_cls == docking and midpoints[detected_cls] is not None:
                a, b = midpoints[detected_cls]
                yaw = batas_bawah + a * ((batas_atas - batas_bawah) * (1/640))

                cv2.line(frame, (int(width/2), int(height/2)), midpoints[detected_cls], self.WHITE_COLOR, self.THICKNESS)
                cv2.circle(frame, midpoints[detected_cls], self.CIRCLE_SIZE, self.RED_COLOR, -1)

                if (a < int(width/2) - self.midpoint_deadzone):
                    cv2.putText(frame, "TURN", (560, 30), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
                    cv2.putText(frame, "LEFT", (560, 60), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
                elif (a > int(width/2) + self.midpoint_deadzone):
                    cv2.putText(frame, "TURN", (560, 30), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
                    cv2.putText(frame, "RIGHT", (560, 60), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
        
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
        
        return frame, yaw, is_finish