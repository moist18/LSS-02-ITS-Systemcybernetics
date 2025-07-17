#!/usr/bin/env python3

import rospy
import cv2
import os

from pathlib import Path
import sys
import os

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))

class Recorder():
    def __init__(self):
        rospy.init_node('recorder', anonymous=True)
        self.rate = rospy.Rate(20)
        
        # Variables
        self.fps = 20
        self.index = 1
        self.prev_time = 0
        self.size = (640, 480)
        self.width = 640
        self.height = 480
        self.video_dir = f'{ROOT}/video'
        self.fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.out = self.video_writer()
        
    def video_writer(self):
        while os.path.exists(os.path.join(self.video_dir, f'output{self.index}.avi')):
            self.index += 1
        
        video = os.path.join(self.video_dir, f'output{self.index}.avi')
        return cv2.VideoWriter(video, self.fourcc, self.fps, self.size)

    def calculate_fps(self):
        current_time = rospy.get_time()
        frame_fps = 1 / (current_time - self.prev_time)
        self.prev_time = current_time
        rospy.loginfo(f"FPS: {int(frame_fps)}")
    
    def run(self):
        try:
            self.cap = cv2.VideoCapture(0)
            self.cap.set(3, self.width)
            self.cap.set(4, self.height)
            
            while not rospy.is_shutdown():
                success, frame = self.cap.read()
                if not success:
                    break
                
                self.out.write(frame)
                self.calculate_fps()
                self.rate.sleep()
            
            self.cap.release()
            cv2.destroyAllWindows()
        except Exception as e:
            rospy.logerr(f"An error occurred: {e}")

if __name__ == "__main__":
    try:
        Recorder().run()
    except rospy.ROSInterruptException:
        pass