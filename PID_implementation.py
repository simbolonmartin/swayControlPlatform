import cv2
import numpy as np
# import globals
import math
import time
import csv
import sympy
import serial
from PyQt5 import QtGui
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QPixmap, QFont
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread
arduino = serial.Serial(port='COM30', baudrate=115200, timeout=.1)

font                   = cv2.FONT_HERSHEY_SIMPLEX
position               = (10,50)
fontScale              = 2
fontColor              = (255,255,0)

t2Old = -2

cam = cv2.VideoCapture(0)
cam.set(3, 360)
cam.set(4, 480)

old_contours = [100,100]
targetPosition = 0
Kp = 1
Ki = 0
Kd = 0.05
errorPositionOld = 0
sumError = 0
initializerTime = 1

def write_read(x):
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.01)
    # data = arduino.readline()
    # return data        

    
while (cam.isOpened()):  # wait until the camera is open
    if initializerTime==1:
        timeStart = time.time()
        initializerTime+=1
        timeNow = 0
    else:
        timeNow = time.time() - timeStart
        

    t1 = time.time()
    ret, img = cam.read()
    img = img[0:260, 80:370]    
    HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = np.array([0,110,153])     
    upper = np.array([54,240,255])
    HSV_clip = cv2.inRange(HSV,lower,upper)
    HSV_Blur = cv2.GaussianBlur(HSV_clip, (5, 5), cv2.BORDER_DEFAULT)
    kernel = np.ones((3, 3), np.uint8)
    erosion = cv2.erode(HSV_Blur, kernel, iterations=2)
    ret, threshold2 = cv2.threshold(erosion, 50, 255, cv2.THRESH_BINARY_INV)
    #######################################################################
    _, contours, hierarchy = cv2.findContours(HSV_Blur, cv2.RETR_TREE,
                                                cv2.CHAIN_APPROX_SIMPLE)

    i = 0
    areas = []
    for c in contours:
        A = cv2.contourArea(c)
        if A < 5500.:
            areas.append(A)

    if len(areas) != 0:
        max_index = np.argmax(areas)
        contours = contours[max_index]
    else:
        contours = old_contours

    
    M = cv2.moments(contours)
    if M["m00"] == 0:
        M["m00"] = 1536375  # 讓球被移除時不要讓被除數等於0 to make sure when the ball is remove,there will still be an select point
        cX, cY = 0, 0
    else:
        cX = int(M["m10"] / M["m00"])  # calculate x,y coordinate of center
        cY = int(M["m01"] / M["m00"])  # 算質心

    # print(cX, cY)
    
    cv2.circle(img, (cX, cY), 7, (0, 0, 255), -1)
    
    # cv2.rectangle(img, (int(cX - 35), int(cY - 35)), (int(cX + 35), int(cY + 35)), (255, 0, 0), 2)
    # cv2.circle(img, (int(cX), int(cY)), 3, (255, 0, 0), 6)
    # cv2.rectangle(img, (int(cX - 35), int(cY - 35)), (int(cX + 35), int(cY + 35)), (255, 0, 0), 2)
    #(this is necessary to avoid Python kernel form crashing)
    distance = str((cX - 140) * 0.22)

    positionNow = float(distance)
    
    errorPosition = targetPosition -  positionNow
    p = Kp* errorPosition
    d = Kd * (errorPosition - errorPositionOld)/0.05
    i = Ki *  sumError
    controlSignal = p + i + d
    controlSignal = -controlSignal 
    sumError = sumError + abs(errorPosition)/100
    errorPositionOld = errorPosition
    
    t2 = time.time()
    
    processingTime = t2Old-t2
    frequency = 1/processingTime
    t2Old = t2
    
    controlSignal = str(round(float(controlSignal)))
    timeNow = str(round(timeNow,2))
    
    write_read(controlSignal)


    

    print(distance, controlSignal, p ,i, d)

    cv2.putText(img, distance,
        position,
        font,
        fontScale,
        fontColor)
    cv2.imshow("PID Position Control",img)
    if cv2.waitKey(1) & 0XFF ==ord('q'):
        break

    