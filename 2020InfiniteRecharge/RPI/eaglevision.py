#!/usr/bin/python3

import time
import cv2
import numpy as np
import json
from networktables import NetworkTables
from cscore import CameraServer
from subprocess import call

NetworkTables.startClientTeam(1781);
NetworkTables.initialize(server='roborio-1781-frc.local')

sd = NetworkTables.getTable('SmartDashboard')

cellsCollected = 0
cellTimer = 0
powerOff = 0
heartBeatCounter = 0
heartBeat = 0

with open('/boot/frc.json') as f:
	config = json.load(f)

camera = config['cameras'][0]
width = camera['width']
height = camera['height']

img = np.zeros(shape=(240, 320, 3), dtype=np.uint8)

CameraServer.getInstance().startAutomaticCapture()
input_stream = CameraServer.getInstance().getVideo()
output_stream = CameraServer.getInstance().putVideo('Processed',width, height)

call("sudo bash -c /home/pi/camera_setup.sh", shell=True)

while True:

	# Video feed
	_, frame = input_stream.grabFrame(img)

	# Filter for yellow
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	lower_yellow = np.array([22,93,85])
	upper_yellow = np.array([45,255,255])
	blur = cv2.GaussianBlur(hsv, (15,15), 0)
	mask = cv2.inRange(blur,lower_yellow,upper_yellow)

	# Find all contours
	tmp = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	contours = tmp[0] if len(tmp) == 2 else tmp[1]

	# Order yellow blobs by size
	heartBeatCounter += 1
	if heartBeatCounter%50 == 0:
		heartBeat += 1
		sd.putNumber('heartBeat', heartBeat)

	area = []
	for cnt in contours:
		area.append(cv2.contourArea(cnt))
	area.sort()

	# If we've found nothing, then send a negative value
	if len(contours) == 0:
		x = -1
		sd.putNumber('x', x)
		print("No contours found")
	else:
		# Find largest yellow blob and apply a bounding rectangle to it
		c = max(contours, key = cv2.contourArea)
		x, y, w, h = cv2.boundingRect(c)
		# Draw rectangle on frame
		cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),2)
		# Draw a circle at it's center
		cv2.circle(frame, (int(x+w/2),int(y+h/2)), 7, (0,255,0), -1)
		cv2.drawContours(frame, [cnt], 0, (255,0,0), 3)
		print("Largest area: {}, x: {}".format(area[-1], x))
		# We'll only consider blobs whos area is at least 500
		if area[-1] >= 50:
			x = x+w/2
			print("sending X: {}".format(x))
		else:
			x = -1
		sd.putNumber('x', x)
	output_stream.putFrame(frame)
cap.release()
cv2.destroyAllWindows()

