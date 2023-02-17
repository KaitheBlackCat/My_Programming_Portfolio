from jajuchaUtil import *
import os
import time

debug = True
finalDebug = True

def cls():
	os.system('cls' if os.name == 'nt' else 'clear')


def autoDrive_algorithm(original_img, canny_img, points, lines, LiDAR, prevComm, status, light):

	cls()
	height, width = canny_img.shape[:2]
	center = int(width / 2) - 2
	command = prevComm

	H1LD = points['H1LD']
	H1RD = points['H1RD']
	H2LD = points['H2LD']
	H2RD = points['H2RD']
	H3LD = points['H3LD']
	H3RD = points['H3RD']
	H1Y = int(height/3) * 1
	H2Y = int(height/3) * 2
	H3Y = int(height/3) * 3

	V1D = points['V1D']
	V2D = points['V2D']
	V3D = points['V3D']
	V4D = points['V4D']
	V5D = points['V5D']
	V6D = points['V6D']
	V7D = points['V7D']

	V1X = int(width/8) * 1
	V2X = int(width/8) * 2
	V3X = int(width/8) * 3
	V4X = int(width/8) * 4
	V5X = int(width/8) * 5
	V6X = int(width/8) * 6
	V7X = int(width/8) * 7

	XD1 = int(width/8) * 1
	XD2 = int(width/8) * 2
	XD3 = int(width/8) * 3

	leftLane, rightLane, endLane = getLane(lines)
	left = right = end = False
	if len(leftLane) != 0:
		left = True
	if len(rightLane) != 0:
		right = True
	if len(endLane) != 0:
		end = True


	if left and right:
		print('Right Line : ', rightLane)
		print('Right Lean : ', getLean(rightLane))
		print('Left Line : ', leftLane)
		print('Left Lean : ', getLean(leftLane))
	elif left:
		print('Left Line : ', leftLane)
		print('Left Lean : ', getLean(leftLane))
	elif right:
		print('Right Line : ', rightLane)
		print('Right Lean : ', getLean(rightLane))
	elif end:
		print('V1D : ', V1D)
		print('V2D : ', V2D)
		print('V3D : ', V3D)
		print('V4D : ', V4D)
		print('V5D : ', V5D)
		print('V6D : ', V6D)
		print('V7D : ', V7D)

	
	time.sleep(1)
	return command, status, light


