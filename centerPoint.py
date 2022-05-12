import cv2
import numpy as np
import globals
import math
import time
import csv
import sympy
from PyQt5 import QtGui
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QPixmap, QFont
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread


cam = cv2.VideoCapture(0)
cam.set(3, 360)
cam.set(4, 480)

old_contours = [100,100]
        
# print("old X:" + str(globals.degreeX_old) + ", old Y:" + str(globals.degreeY_old))
while (cam.isOpened()):  # wait until the camera is open
    ret, img = cam.read()
    img = img[0:260, 80:370]    
    HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = np.array([0,110,153])     #
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

    print(cX, cY)
    
    cv2.circle(img, (cX, cY), 7, (0, 0, 255), -1)
    # cv2.rectangle(img, (int(cX - 35), int(cY - 35)), (int(cX + 35), int(cY + 35)), (255, 0, 0), 2)
    # cv2.circle(img, (int(cX), int(cY)), 3, (255, 0, 0), 6)
    # cv2.rectangle(img, (int(cX - 35), int(cY - 35)), (int(cX + 35), int(cY + 35)), (255, 0, 0), 2)
    #(this is necessary to avoid Python kernel form crashing)
    cv2.imshow("Original",img)
    if cv2.waitKey(1) & 0XFF ==ord('q'):
        break
