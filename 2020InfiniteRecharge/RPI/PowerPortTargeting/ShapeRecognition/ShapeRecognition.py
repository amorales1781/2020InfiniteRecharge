import cv2
import numpy as np

class PortFinder:
	def __init__(self):
		self.img = cv2.imread('PowerPort.jpg', cv2.IMREAD_GRAYSCALE)
		_, self.threshold = cv2.threshold(self.img,140,255,cv2.THRESH_BINARY)
		_, self.contours, _ = cv2.findContours(self.threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		self.font = cv2.FONT_HERSHEY_COMPLEX
	
	def getPortCenter(self):
		for cnt in self.contours:
			area = cv2.contourArea(cnt)
			approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt , True), True)
		
			if len(approx)== 6 and area > 1000:
				M = cv2.moments(cnt)
				cX = int(M["m10"] / M["m00"])
				cY = int(M["m01"] / M["m00"])
				x1 = approx.ravel()[0]
				y1 = approx.ravel()[1]
				cv2.drawContours(self.img, [approx], 0, (0, 255, 0), 3)
				cv2.circle(self.img, (cX,cY), 7, (255,255,255), -1)
				cv2.putText(self.img, "Hexagon", (x1-50,y1), self.font, 1, (0))
				return cX, cY
	
	def showImages(self):
		cv2.imshow("shapes",self.img)
		cv2.imshow("Threshold", self.threshold)

portFinder = PortFinder()

print(portFinder.getPortCenter())
portFinder.showImages()	

		

cv2.waitKey(0)
cv2.destroyAllWindows()
