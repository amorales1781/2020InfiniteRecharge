#!/usr/bin/python3

import time
import cv2
import numpy as np
from networktables import NetworkTables

NetworkTables.startClientTeam(1781);
NetworkTables.initialize(server='roborio-1781-frc.local')

sd = NetworkTables.getTable('SmartDashboard')


cellsCollected = 0
cellTimer = 0
powerOff = 0
heartBeatCounter = 0
heartBeat = 0

cap = cv2.VideoCapture(0)
cap.set(3,350)
cap.set(4,350)
cap.set(5, 20)

while True:
	sd.putNumber('piPower', powerOff)

	# Video feed
	_, frame = cap.read()
	# Filter for yellow
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	lower_yellow = np.array([22,93,85]) 
	upper_yellow = np.array([45,255,255])
	blur = cv2.GaussianBlur(hsv, (15,15), 0)
	mask = cv2.inRange(blur,lower_yellow,upper_yellow)
	# Show what has been identified as yellow in white on binary image
	cv2.imshow('Mask', mask)
	
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
		cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
		# Draw a circle at it's center
		cv2.circle(frame, (int(x+w/2),int(y+h/2)), 7, (255,255,255), -1)
		#cv2.drawContours(frame, [cnt], 0, (255,0,0), 3)
		print(x+w/2,y+h/2)
		print("Largest area: {}, x: {}".format(area[-1], x))
		# We'll only consider blobs whos area is at least 500
		if area[-1] >= 500:
			x = x+w/2
			print("sending X: {}".format(x))
			# I love you
			if int(y+h/2) >= 260:
				cellTimer+=1
				if cellTimer <= 1:
					cellsCollected += 1
			else: cellTimer = 0
			print("Cells Collected: {}".format(cellsCollected))
			sd.putNumber('cells', cellsCollected)
		else:
			x = -1
		sd.putNumber('x', x)
	cv2.imshow("Frame", frame)	
	powerOff = sd.getNumber('piPower', 0)
	if(powerOff == 1):
		from subprocess import call
		call("sudo poweroff", shell=True)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()



