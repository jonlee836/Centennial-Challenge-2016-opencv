from picamera.array import PiRGBArray
from picamera import PiCamera

import time
import cv2
from fractions import Fraction

def nothing(x):
    pass

InputImg = 'image'
guiImg   = 'gui'

cv2.namedWindow(InputImg)
cv2.namedWindow(guiImg)

setCon = 100
setBright = 50
setSharp = 0
setSat = 50
setExpcomp = 25

ImgWidth = 640
ImgHeight = 480

RedGain1 = 351
RedGain2 = 248

BlueGain1 = 347
BlueGain2 = 239

setISO = 200
recordImages = 0

expSpeed = 500
shutSpeed = 1000

cv2.createTrackbar('ISO',       guiImg,  setISO, 2000,  nothing)
cv2.createTrackbar('RedGain1' , guiImg, RedGain1, 500,  nothing)
cv2.createTrackbar('RedGain2' , guiImg, RedGain2, 500,  nothing)
cv2.createTrackbar('BlueGain1', guiImg, BlueGain1, 500, nothing)
cv2.createTrackbar('BlueGain2', guiImg, BlueGain2, 500, nothing)
cv2.createTrackbar('recordImages', guiImg, recordImages, 1, nothing)
cv2.createTrackbar('shutSpeed', guiImg, shutSpeed, 50000, nothing)
cv2.createTrackbar('setSat', guiImg, setSat, 100, nothing)

cv2.createTrackbar('setCon', guiImg, setCon, 200 ,nothing)
cv2.createTrackbar('setBright', guiImg, setBright, 100, nothing)
cv2.createTrackbar('setSharp', guiImg, setSharp, 100 ,nothing)
cv2.createTrackbar('setSat', guiImg, setSat, 100,nothing)
cv2.createTrackbar('setExpcomp', guiImg, setExpcomp, 50,nothing)

camera = PiCamera(resolution=(ImgWidth, ImgHeight), framerate=30)
rawCapture = PiRGBArray(camera, size=(ImgWidth, ImgHeight))

#warm up time 5 seconds
time.sleep(3)

#camera.ISO = setISO

#camera.shutter_speed = shutSpeed

#camera.shutter_speed = camera.exposure_speed
#camera.exposure_mode = 'off'
camera.awb_mode = 'off'

analogGain = camera.analog_gain
digitalGain = camera.digital_gain

i=1                               
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):  

    awb_RedBlueGains  = (Fraction(RedGain1, RedGain2), Fraction(BlueGain1, BlueGain2))
    recordImages      = cv2.getTrackbarPos('recordImages', guiImg)    

    RedGain1 = cv2.getTrackbarPos('RedGain1', guiImg)
    RedGain2 = cv2.getTrackbarPos('RedGain2', guiImg)

    BlueGain1 = cv2.getTrackbarPos('BlueGain1', guiImg)
    BlueGain2 = cv2.getTrackbarPos('BlueGain2', guiImg)

    setISO = cv2.getTrackbarPos('ISO', guiImg)
    
    shutSpeed    = cv2.getTrackbarPos('shutspeed', guiImg)
    setSat       = cv2.getTrackbarPos('setSat', guiImg)
    setCon       = cv2.getTrackbarPos('setCon', guiImg) - 100
    setBright    = cv2.getTrackbarPos('setBright', guiImg)
    setSharp     = cv2.getTrackbarPos('setSharp', guiImg)
    setExpcomp   = cv2.getTrackbarPos('setExpcomp', guiImg) - 25

    camera.contrast = setCon
    camera.brightness = setBright
    camera.sharpness = setSharp
    camera.saturation = setSat
    camera.exposure_compensation = setExpcomp
    camera.awb_gains = awb_RedBlueGains
    
    tempIso = camera.ISO		

    analogGain = camera.analog_gain
    digitalGain = camera.digital_gain
    
    g = awb_RedBlueGains

    (redGain,blueGain) = g
    
    print i, camera.exposure_speed, camera.shutter_speed, tempIso, analogGain, digitalGain, float(redGain), float(blueGain)
    
    image = frame.array
    imgname = "%05d-%s-%d-%d-%s-%s.jpg"%(i, str(g), camera.exposure_speed, tempIso, str(analogGain), str(digitalGain))
    
    cv2.imshow(InputImg, image)

    if recordImages == 1:
        i = 1
        cv2.imwrite(imgname, image)
    
    key = cv2.waitKey(1) & 0xFF

    rawCapture.truncate(0)
    
    if key == ord("q"):
        break

    i+=1
    
#camera.capture_sequence(['image%0004d.jpg' % i for i in range(10)])
