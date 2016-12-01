#!/usr/bin/python

import rospy

from picamera.array import PiRGBArray
from picamera import PiCamera

import time
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage

rospy.init_node('rpi_pydriver')
pub = rospy.Publisher("/rpi_pydriver/image/compressed", CompressedImage, queue_size=1)

ImgWidth = 640
ImgHeight = 480

camera = PiCamera(resolution=(ImgWidth, ImgHeight), framerate=10)

#camera.sharpness = 100
#camera.saturation = 100
#camera.ISO = 250
#camera.video_stabilization = False

#camera.exposure_mode ='auto'

#camera.awb_mode = 'off'
#camera.awb_mode = 'auto'
#camera.awb_mode = 'sunlight'
#camera.awb_mode = 'cloudy'
#camera.awb_mode = 'shade'
#camera.awb_mode = 'tungsten'
#camera.awb_mode = 'fluorescent'
#camera.awb_mode = 'incandescent'
#camera.awb_mode = 'flash'
#camera.awb_mode = 'horizon'

#camera.ISO = 500
camera.image_effect = 'saturation'
camera.saturation = 50

rawCapture = PiRGBArray(camera, size=(ImgWidth, ImgHeight))

#Fixed exposure
#warm up time 5 seconds

time.sleep(5)

camera.shutter_speed=camera.exposure_speed
expSpeed = camera.shutter_speed
camera.exposure_mode = 'off'

g = camera.awb_gains

camera.awb_mode = 'off'
camera.awb_gains = g

i=1

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    if rospy.is_shutdown():
        break

    print i
    image = frame.array
    
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
    pub.publish(msg)
    
##    cv2.imshow("img",image)
    #cv2.imwrite("SelectiveArea_Sat50_IfxSat_awbOFF_expOFF_ISO100_GAINS_ON%000005d.jpg" % i, image)
    
##    key = cv2.waitKey(1) & 0xFF

    rawCapture.truncate(0)
    
##    if key == ord("q"):
##        break

    i+=1
    
#camera.capture_sequence(['image%0004d.jpg' % i for i in range(10)])
