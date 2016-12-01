# USAGE
# python picamera_fps_demo.py
# python picamera_fps_demo.py --display 1

# import the necessary packages
from __future__ import print_function
from imutils.video.pivideostream import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera

import argparse
import imutils
import time
import cv2
import numpy as np

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-n", "--num-frames", type=int, default=100,
	help="# of frames to loop over for FPS test")
ap.add_argument("-d", "--display", type=int, default=0,
	help="Whether or not frames should be displayed")
args = vars(ap.parse_args())

# created a *threaded *video stream, allow the camera sensor to warmup,
# and start the FPS counter

print("THREADED frames from `picamera` module...")

vs = PiVideoStream().start()

time.sleep(1.0)
fps = FPS().start()

# loop over some frames...this time using the threaded stream
#while fps._numFrames < args["num_frames"]:
while True:

	frame = vs.read()
        frame = imutils.resize(frame, width=640)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2Luv)
        l,u,v = cv2.split(frame)
        frame = v
        corners = cv2.goodFeaturesToTrack(frame, 25, 0.01, 100)

        if corners is not None:
                corners = np.int0(corners)
                
                for i in corners:
                        x,y = i.ravel()
                        cv2.circle(frame,(x,y),3,255,-1)
                        
                fps.update()        
        
                key = cv2.waitKey(1) & 0xFF
                #cv2.imshow("Frame", frame)
                
                if key == ord("q"):
                        cv2.destroyAllWindows()
                        break
        #                vs.stop()
        #                fps.stop()

vs.stop()
fps.stop()
print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
