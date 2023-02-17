from jajuchaUtil import *
import os

debug = True
finalDebug = True


def autoDrive_algorithm(original_img, canny_img, points, lines, LiDAR, prevComm, status, light):


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
		status = ONSTRAIGHT

		if H2LD != 160 and H2RD != 160:
			if debug: print('center point calculated by H2 points %d, %d' % (H2LD, H2RD))
			centerPoint_X = (320 - H2LD + H2RD) / 2

		if centerPoint_X > center:
			if centerPoint_X > center + 20:
				command = 'S1170E'
			else:
				command = 'S1160E'
		elif centerPoint_X < center:
			if centerPoint_X < center - 20:
				command = 'S1130E'
			else:
				command = 'S1140E'
		else:
			command = 'S1150E'
	elif end:
		command = prevComm

	if status is ONSTRAIGHT:
		if left and right:
			if debug:
				print('Both Line Detected While Going Straight')
			if getLean(rightLane) < 0.75:
				if debug:
					print('Right Lean Exception')
				command = 'S1165E'
			if getLean(leftLane) > -0.75:
				if debug:
					print('Left Lean Exception')
				command = 'S1135E'
			if H2LD != 160 and H2RD != 160:
				if debug: print('Center Point (H2) : (%d , %d)' % (H2LD, H2RD))
				centerPoint_X = (320 - H2LD + H2RD)/2
			elif H3LD != 160 and H3RD != 160:
				if debug: print('Center Point (H3) : (%d, %d)' % (H3LD, H3RD))
				centerPoint_X = (320 - H3LD + H3RD) - 67
			else:
				return command, status, light

			if H1RD is -1 or H2RD is -1:
				if debug: print('Status Is Currently ONRIGHT')
				status = ONRIGHT
			elif centerPoint_X > center:
				if centerPoint_X > center + 20:
					command = 'S1170E'
				else:
					command = 'S1160E'
			elif centerPoint_X < center - 10:
				command = 'S1140E'
			else:
				command = 'S1150E'

		elif right and not left and end:
			if debug:
				print('case 3')
				print('Status changed straight to left corner')
			command = 'S1125E'
			status = ONLEFT
		elif not right and left and end:
			if debug:
				print('case 4')
				print('Status changed straight to right corner')
			command = 'S1175E'
			status = ONRIGHT

	elif status is ONLEFT:
		if right and not left and end:
			if debug:
				print('case 3 / ONLEFT')
			if V4D < 100:
				command = 'S1110E'
		elif right and left:
			if debug:
				print('case 1, 2 / ONLEFT')
			if (H2LD > 60 and H2RD > 60) or (H3LD > 65 and H2RD > 65):
				if debug:
					print('Status changed left corner tp straight')
				command = 'S1150E'
				status = ONSTRAIGHT
		elif end:
			print('Only Left')
			command = 'S1110E'
		else:
			print('OUT')
			command = prevComm

	elif status is ONRIGHT:
		print('ONRIGHT')

		if V4D < 100:
			command = 'S1180E'
			status = ONSCORNER
		else:
			command = 'S1150E'
			
	elif status is ONSCORNER:
		print('ONSCORNER')
		command = 'S1185E'
		if right and left:
			if debug:
				print('case 1, 2/ ONRIGHT')
			if H3LD > 155 and H3RD > 155:
				if debug:
					print('Status changed right corner to straight')
				command = 'S1150E'
				status = ONSTRAIGHT

	if finalDebug:
		print('========== Final Debuging ==========')
		print('Final Command : ', command)
		print('Right Line : ', rightLane)
		print('Right Lean : ', getLean(rightLane))
		print('Left Line : ', leftLane)
		print('Left Lean : ', getLean(leftLane))
		print('Status : ', status)
		print('Middle Point (H1) : (%d, %d)' % (H1LD, H1RD))
		print('Middle Point (H2) : (%d, %d)' % (H2LD, H2RD))
		print('Middle Point (H3) : (%d, %d)' % (H3LD, H3RD))
		print('End Point Distance : ', V4D)
		print('====================================')
		print('\n')
		print('\n')
		print('\n')
		print('\n')
		print('\n')
		print('\n')
		print('\n')
		print('\n')

	
	return command, status, light


