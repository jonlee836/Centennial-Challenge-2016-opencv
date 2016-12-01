#!/usr/bin/python

import rospy
from picamera import PiCamera
import io
import time
from sensor_msgs.msg import CompressedImage
from fractions import Fraction

rospy.init_node('rpi_pydriver')
rospy.loginfo('rpi_pydriver started')
pub = rospy.Publisher("/center_cam/image/compressed", CompressedImage, queue_size=1)

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
#camera.image_effect = 'saturation'
camera.saturation = 50

#Fixed exposure
#warm up time 5 seconds
rospy.loginfo('waiting for camera params to settle')
time.sleep(2.5)

camera.shutter_speed=camera.exposure_speed
camera.exposure_mode = 'off'

#g = camera.awb_gains
camera.awb_mode = 'off'
g = (Fraction(93, 64), Fraction(301, 256))
camera.awb_gains = g
rospy.loginfo("Using gains: %s"%str(g))

rospy.loginfo('start publishing image')
stream = io.BytesIO()
for _ in camera.capture_continuous(stream, 'jpeg', use_video_port=True):
    if rospy.is_shutdown():
        break
    t1 = time.time()
    
    stream.seek(0)
    data = stream.read()
    stream.seek(0)
    stream.truncate()
    
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = data
    pub.publish(msg)
    t2 = time.time()
    
    #print "%.4f"%(t2-t1)

