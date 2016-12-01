from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

camera = PiCamera(resolution=(640, 480), framerate=30)

camera.sharpness = 100
#camera.saturation = 100

camera.video_stabilization = True

camera.exposure_mode ='auto'

#camera.awb_mode = 'off'
camera.awb_mode = 'auto'
#camera.awb_mode = 'sunlight'
#camera.awb_mode = 'cloudy'
#camera.awb_mode = 'shade'
#camera.awb_mode = 'tungsten'
#camera.awb_mode = 'fluorescent'
#camera.awb_mode = 'incandescent'
#camera.awb_mode = 'flash'
#camera.awb_mode = 'horizon'

#camera.image_effect = 'saturation'
#camera.image_effect = 'deinterlace1'
#camera.image_effect = 'deinterlace2'

#-------------------------------------

#camera.meter_mode = 'average'
#camera.meter_mode = 'spot'
#camera.meter_mode = 'backlit'
#camera.meter_mode = 'matrix'

rawCapture = PiRGBArray(camera, size=(640,480))

time.sleep(0.5)

i=1

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    #camera.capture(['image%000005d.jpg' % i])
    image = frame.array

    cv2.imwrite("default_%000005d.jpg" % i, image)    
    cv2.imshow("img",image)
    key = cv2.waitKey(1) & 0xFF

    rawCapture.truncate(0)
    
    if key == ord("q"):
        break

    i+=1
    
#camera.capture_sequence(['image%0004d.jpg' % i for i in range(10)])
