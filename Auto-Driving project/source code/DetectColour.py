import cv2
import numpy as np

cap = cv2.imread("0041-0740.jpg", cv2.IMREAD_ANYCOLOR)

while(1):

	frame = cap.read()
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	# define range of white color in HSV
	# change it according to your need !
	sensitivity = 15
	lower_white = np.array([0,0,255-sensitivity])
	upper_white = np.array([255,sensitivity,255])

	# Threshold the HSV image to get only white colors
	mask = cv2.inRange(hsv, lower_white, upper_white)
	# Bitwise-AND mask and original image
	res = cv2.bitwise_and(frame,frame, mask= mask)

	cv2.imshow('frame',frame)
	cv2.imshow('mask',mask)
	cv2.imshow('res',res)

	k = cv2.waitKey(5) & 0xFF
	if k == 27:
		break

	cv2.destroyAllWindows()