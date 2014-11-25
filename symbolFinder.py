#!/usr/bin/python
#
# symbolFinder.py
#
# Author: Derek Campbell
# Date  : 22/10/2014
#
#  Copyright 2014  <guzunty@gmail.com>
#  
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#  
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#  
# This class uses OpenCV to detect symbols of the specified colour in
# the image captured from the Raspberry Pi camera.
#
# The position of the detected symbol in the image is available via the
# 'getPatch()' method.

import cv2
import threading
import time
import os
import numpy as np
import sys

def readThresholds():
    low = []
    high=[]
    try:
        f = open('filter.txt','r')
        line = f.readline().split(',')
        for i in range(3):
          low.append(int(line[i]))
        line = f.readline().split(',')
        for i in range(3):
          high.append(int(line[i]))
        print low,high
    
    except:
        print "exception"
        low = [30,160,80]
        high = [89,225,120]
    finally:
        f.close()
    return low,high
class SymbolFinder(threading.Thread):

  def __init__(self):
    super(SymbolFinder, self).__init__()
    self.cap = cv2.VideoCapture()
    self.active = True
    self.dataReady = False
    self.patch = None
    self.gate = threading.RLock()
    self.gate.acquire()
    self.spotFilter = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    self.maskMorph = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))  
    #self.low = (30,160,80)
    #self.high = (89,225,120)
    self.low,self.high = readThresholds()
    self.acquiredImage = False

  def run(self):
    self.throttle = 5
    self.gate.acquire()
    while (self.active == True):
      ret, raw = self.cap.read()
      self.frame = cv2.flip(raw,-1)
      self.acquiredImage = True
      imgHSV = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
      mask = cv2.inRange(imgHSV, np.array(self.low), np.array(self.high) )
      mask = cv2.erode(mask, self.spotFilter)    # Remove spots in image
      mask = cv2.dilate(mask, self.maskMorph)    # Merge holes in image
      # Find the contours in the mask
      contours, hierarchy = cv2.findContours(mask, 1, 2)
      # Find the contour with the greatest area
      area = 0.0
      contour = None
      for candidateContour in contours:
        candidateArea = cv2.contourArea(candidateContour)
        if candidateArea > area:
          area = candidateArea
          contour = candidateContour
      # Save the bounding rectangle for the contour
      if len(contours) > 0:
        self.patch = cv2.boundingRect(contour)
        self.dataReady = True
      self.gate.release()
      time.sleep(0.1)
      self.gate.acquire()

  def enable(self):
    self.cap.open(0)
    if (self.cap.isOpened()):
      #os.system("v4l2-ctl -p 4")
      self.cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 320)
      self.cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 240)
      self.gate.release()
    else:
      print("ERROR: symbolFinder.py 35 : Failed to open camera")
      self.active = False

  def disable(self):
    self.gate.acquire()
    self.cap.release()

  def stop(self):
    self.disable()
    self.active = False
    self.gate.release()
    

  def getPatch(self):
    self.dataReady = False
    return self.patch, self.frame

if __name__ == "__main__":
  letsGo = True
  sym = SymbolFinder()
  sym.enable()
  sym.start()

  while letsGo:

    
    try:
      if (sym.dataReady == True):
         patch, frame = sym.getPatch()
         x,y,w,h = patch
         cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
         cv2.imshow(" image ",frame)
      if (cv2.waitKey(1) == 27):
        break
    except KeyboardInterrupt:
      sym.stop()
      sym.join()
      letsGo = False