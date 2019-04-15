from collections import deque
import sys
import cv2
import cv2 as cv
import numpy as np
import serial
import argparse
import time
import json
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=32,
	help="max buffer size")
args = vars(ap.parse_args())

kernel = np.ones((5,5),np.uint8)

centerX = int(250)
centerY = int(250) 
# Take input from webcam
cap = cv2.VideoCapture(0)
 
# Reduce the size of video to 320x240 so rpi can process faster
cap.set(3,500)
cap.set(4,500)
 
def nothing(x):
    pass
# Creating a windows for later use
cv2.namedWindow('HueComp')
cv2.namedWindow('SatComp')
cv2.namedWindow('ValComp')
cv2.namedWindow('closing')
cv2.namedWindow('tracking')

# initialize the list of tracked points, the frame counter,
# and the coordinate deltas 
pts = deque(maxlen=args["buffer"])
counter = 0
(dX, dY) = (0, 0)
direction = ""

# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial('\\\\.\\COM7', 9600)
time.sleep(2)
print("Connection to arduino...")

# Creating track bar for min and max for hue, saturation and value
# You can adjust the defaults as you like
cv2.createTrackbar('hmin', 'HueComp',12,179,nothing)
cv2.createTrackbar('hmax', 'HueComp',37,179,nothing)
 
cv2.createTrackbar('smin', 'SatComp',96,255,nothing)
cv2.createTrackbar('smax', 'SatComp',255,255,nothing)
 
cv2.createTrackbar('vmin', 'ValComp',186,255,nothing)
cv2.createTrackbar('vmax', 'ValComp',255,255,nothing)

 
# My experimental values
# hmn = 12
# hmx = 37
# smn = 145
# smx = 255
# vmn = 186
# vmx = 255


while(1):
 
    buzz = 0
    _, frame = cap.read()
 
    #converting to HSV
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    hue,sat,val = cv2.split(hsv)
 
    # get info from track bar and appy to result
    hmn = cv2.getTrackbarPos('hmin','HueComp')
    hmx = cv2.getTrackbarPos('hmax','HueComp')
   
 
    smn = cv2.getTrackbarPos('smin','SatComp')
    smx = cv2.getTrackbarPos('smax','SatComp')
 
 
    vmn = cv2.getTrackbarPos('vmin','ValComp')
    vmx = cv2.getTrackbarPos('vmax','ValComp')
 
    # Apply thresholding
    hthresh = cv2.inRange(np.array(hue),np.array(hmn),np.array(hmx))
    sthresh = cv2.inRange(np.array(sat),np.array(smn),np.array(smx))
    vthresh = cv2.inRange(np.array(val),np.array(vmn),np.array(vmx))
 
    # AND h s and v
    tracking = cv2.bitwise_and(hthresh,cv2.bitwise_and(sthresh,vthresh))
 
    # Some morpholigical filtering
    dilation = cv2.dilate(tracking,kernel,iterations = 1)
    closing = cv2.morphologyEx(dilation, cv2.MORPH_CLOSE, kernel)
    closing = cv2.GaussianBlur(closing,(5,5),0)
 
    # Detect circles using HoughCircles
    circles = cv2.HoughCircles(closing,cv2.HOUGH_GRADIENT,2,120,param1=120,param2=50,minRadius=10,maxRadius=0)
    # circles = np.uint16(np.around(circles))
    cv2.line(frame,(centerX-20,centerX),(centerX+20,centerX),(0,255,0),2)
    cv2.line(frame,(centerY,centerY-20),(centerY,centerY+20),(0,255,0),2)
    
    #Draw Circles
    if circles is not None:
            for i in circles[0,:]:
                # If the ball is close, draw it in green
                if int(round(i[2])) > 35:
                    print("Location:", str(int(round(i[0]))), ":", str(int(round(i[1]))))
                    x = str(int(round(i[0])))
                    y = str(int(round(i[1])))
                    cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),int(round(i[2])),(0,255,0),5)
                    cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),2,(0,255,0),10)

                    data = "Location".format(x,y)
                    ser.write(data.encode())
                    
                # else draw it in red
                elif int(round(i[2])) < 30:
                    cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),int(round(i[2])),(0,0,255),5)
                    cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),2,(0,0,255),10)
                    buzz = 1

                                                                   
    #Show the result in frames
    cv2.imshow('HueComp',hthresh)
    cv2.imshow('SatComp',sthresh)
    cv2.imshow('ValComp',vthresh)
    cv2.imshow('closing',closing)
    cv2.imshow('tracking',frame)


	
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
 
cap.release()
serial.release()
cv2.destroyAllWindows()
