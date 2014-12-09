#!/usr/bin/python
#
# testSymbol.py based on the following
# PiTeR.py
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
# This is steves Lego robot main control program
#
# Enables control with a Wii remote. See the startup message near 
# the bottom of the script for button meanings.


import time
import struct
import os
import sys
import random
import symbolFinder
import symbolIdentifier
import numpy as np
import cv2
import wiiControl
import legoRobotClass


global symFinder

def shutdown():
  global symFinder
  symFinder.stop()
  symFinder.join()
###########################################################
# process image and look for symbol
def lookForSymbol(symFinder):
    global lastResult
    global consensus
    error = -1
    symCommand = "XX"
    if (symFinder.dataReady == True):
      patch, frame = symFinder.getPatch()
      row,column,depth = frame.shape
      imageCentre = column /2
      symbolCentre = patch[0] + patch [2]
      error = symbolCentre - imageCentre
      print error
      cv2.imshow("captured image",frame)
      patchArea = patch[2] * patch[3]
      symCommand = "RR"
      if (patchArea > 6000):
        # We're right in front of the patch, now identify the symbol
        result = symIdentifier.computeBestMatch(frame, patch)
        if (result != "" and result == lastResult):
          consensus = consensus + 1
          if (consensus >= CONS_THRESHOLD):
            # Perform the required action
            lastResult = ""
            #resultConsensus = 0
            if ( "straight" in result):
              print("Straight On")
              symCommand = "F"
            elif ("left" in result):
              print("turning left")
              symCommand = "L"
            elif ("right" in result):
              print("turning right")
              symCommand = "R"
            elif ("not_this_way"):
              print("turning around")
              symCommand = "B"
            else:
              print("Unknown symbol found: " + result)
              symCommand = "U"
        else:
          if (result != ""):
            lastResult = result
            consensus = 0
      elif (patchArea > 500):
          print("Approaching symbol ",patchArea)
          symCommand = "A"
      else:
        # We lost sight of our patch, back up scanning back and forth
        print("Backing up")
        symCommand = "D"
      
    return symCommand, error
###############################################################################
# start symbol finder thread
symFinder = symbolFinder.SymbolFinder()
symFinder.start()
symFinder.enable()


symIdentifier = symbolIdentifier.SymbolIdentifier("/home/pi/Pi-master/MagPi/symbols")
CONS_THRESHOLD = 4
consensus = 0
lastResult = ""

cap = cv2.VideoCapture()
cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 240)

##########################################
# first try and connect to a wii controller, if this fails then
# commands are taken from the keyboard

wii = None
try:
  wiiMaxAttempts = int(sys.argv[1])
except:
  wiiMaxAttempts = 100

wiiConnected = True

connected ,wii= wiiControl.attemptToConnect(wiiMaxAttempts,wii)
if connected >= wiiMaxAttempts:
  print "failed to connect - use keyboard", connected
  wiiConnected = False
  
if wiiConnected:
   wii.led = 15 # all leds on
###########################################################


robot = legoRobotClass.myThread(1, "Thread-1", 1)
		#Setup and start the thread

robot.setDaemon(True)
robot.start() 

# open file for debug
debugFile = open('robotdebug.txt','w')
debugFile.write('here we go \n')

wii.led = 0

letsRun = True
lastsymCommand = "-"
try:
  while letsRun:

    symCommand , error= lookForSymbol(symFinder)
    if lastsymCommand != symCommand:
       print "new symcommand ",symCommand
       lastsymCommand = symCommand
    if wiiConnected:
          Command = wiiControl.getCommand(wii)
    else:
          Command = raw_input("F=Forward\nB=Backward\nL=Left\nR=Right\nS=Stop\nX=Shutdown PI\n1=Follow\n2=quit program\n")
    # process command
    debugFile.write(symCommand+" "+str(error)+" "+Command+"\n")
    if Command == "1":
       # auto mode selected
       if symCommand == "F":
          robot.moveForwards()
          wii.led = 5
       elif symCommand == "L":
          robot.turn(-90)
          wii.led = 3
       elif symCommand == "R":
          robot.turn(+90)
          wii.led = 9
       else:
          robot.stopMoving()
          wii.led = 0
    elif Command == "F":
         robot.moveForwards()
         wii.led = 2
    elif Command == "B":
         robot.moveBackwards()
    elif Command == "L":
         robot.turnLeft()
         wii.led = 4
    elif Command == "R":
         robot.turnRight()
         wii.led = 8
    elif Command =="A" and lastCommand != "A":
         #add debounce by making sure command is changed
         robot.accelerate()
    elif Command == "D" and lastCommand != "D":
         robot.decelerate()
    elif Command == "2" or Command =="X":
         letsRun = False
         wiiControl.rumbleWii(wii)
    else:
         robot.stopMoving()
         wii.led = 0

except KeyboardInterrupt:
  print("Keyboard interrupt")
  debugFile.close()
  shutdown()    

except Exception as e:
  print ("Fatal error in test main loop: {0}".format(e))
  exc_type, exc_obj, exc_tb = sys.exc_info()
  fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
  print(exc_type, fname, exc_tb.tb_lineno)
  debugFile.close()
  shutdown()

finally:
  print "FINALLY"
  debugFile.close()
  robot.running = False
  shutdown()
  if Command =="X":
    os.system("sudo shutdown -h now")
