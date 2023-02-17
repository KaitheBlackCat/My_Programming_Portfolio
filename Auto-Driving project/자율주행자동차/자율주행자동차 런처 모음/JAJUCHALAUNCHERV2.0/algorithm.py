from jajuchaUtil import *
import cv2
import time

#각각의 값들은 첫번째 값은 직진 상태일 때, 두 번째 값은 급격한 우회전이 필요할 때, 세 번째 값은 급격한 좌회전이 필요로 할 때이다.
#각각의 부가값 또한 각각의 값에 따라 별도로 지정한다.
#이름 = [직진 상태, 급격한 우회전 필요 상태, 급격한 좌회전 필요 상태]

rightSlope = [0.8, 1.42, 0.69]
leftSlope = [-0.8, -0.69, -1.42]
fixValue = [0.1, 0, 0]

#이 값은 카메라와 도로 간 실제 중앙에 대한 추적이 다름을 보정하기 위한 값으로 실제로는 사용자가(당신이) 이 값을 일일히 조정해
#차량이 직선차로에서 정중앙을 주행하는지 파악해야 한다.

straightValue = 3

trafficlight_cascade = cv2.CascadeClassifier('trafficlight_cascade.xml') #신호등 인식을 위한 빅데이터 파일을 불러운다. 제대로 작동한 적은 없다.


def findTrafficLight(image): #신호등 인식을 위한 함수이다. 제대로 작동한 적은 없다.

    height, width = image.shape[:2]
    ROI_image = image[:50,: ]
    light = 1

    ROI_gray = cv2.cvtColor(ROI_image, cv2.COLOR_BGR2GRAY)
    trafficlight = trafficlight_cascade.detectMultiScale(ROI_gray, 1.1, 4)
    
    for (x,y,w,h) in trafficlight:

        if w < 35:
            break
        ROI =  ROI_gray[y:y+h, x:x+w]
        ROI_X = ROI.shape[1]
        ROI_Y = ROI.shape[0]
        
        light_position = -1
        
        for i in range(ROI_X):
            for j in range(ROI_Y):
                if ROI[i, j] > 125:
                    light_position = j

        if light_position > ROI_X / 2 + 4:
            light = 0

        return (x, y, w, h, light)
    return False

def autoDrive_algorithm(original_img, canny_img, points, lines, LiDAR, prevComm, status, light):


    height, width = canny_img.shape[:2]
    center = int(width / 2) - 2
    command = prevComm
    startLine = False
    light = 2


    result = findTrafficLight(original_img) #마찬가지로, 제대로 작동한 적 없다.
    if result != False:
    	x = result[0]
    	y = result[1]
    	w = result[2]
    	h = result[3]
    	light = result[4]
    	if light == 1:
    		cv2.rectangle(original_img, (x, y), (x+w, y+h), (0, 0, 255), 4)
    	elif light == 0:
    		cv2.rectangle(original_img, (x, y), (x+w, y+h), (0, 255, 0), 4)
    	prevlight = light

    st = False


    H1LD = points['H1LD'] #각각의 포인트를 나타내기 위한 변수 지정. 알고리즘 구현 화면에서 사용된다.
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

   

    leftLane, rightLane, endLane = getLane(lines) #직선의 행렬을 계산한다. 형태는 [x1, y1, x2, y2] 혹은 [[x1, y1, x2, y2], [x3, y3, x4, y4], ...]
    left = right = end = False
    if len(leftLane) != 0 : #행렬 길이가 0이라면 직선이 인식 안 된 상태임을 알 수 있다.
        left = True
    if len(rightLane) != 0 :
        right = True
    if len(endLane) != 0 :
        end = True

    balance = 0

    if light == 1:
    	command = 'S0150E'

    elif status is ONSTRAIGHT :
        #if LiDAR != 0 and LiDAR < 300 : #라이다는 시뮬레이션이나 실제 주행에서는 방해되기 때문에 라이다가 필요하다면 주석을 해제하고 아래에 if left~꼴을 elif로 수정하라.
        #	command = 'S1150E'
        
        if left and right : #양쪽 차선이 모두 존재하는 경우

            status = ONSTRAIGHT

            if H2LD != 160 and H2RD != 160: #일단 남겨두나 기울기를 통한 방법에는 필요가 없다. 
            	centerPoint_X = (320 - H2LD + H2RD) / 2
            	print(centerPoint_X)
            elif H3LD != 160 and H3RD != 160:
            	centerPoint_X = (320 - H3LD + H3RD) / 2
            	print(centerPoint_X)

            """
            if centerPoint_X > center + 5:
            	balance = 1
            	if centerPoint_X > center + 20:
            		command = 'S1162E'
            	elif centerPoint_X > center + 10:
            		command = 'S1157E'
            	else:
            		command = 'S1152E'
            elif centerPoint_X < center - 5:
            	balance = 2
            	if centerPoint_X < center - 20:
            		command = 'S1138E'
            	elif centerPoint_X < center - 10:
            		command = 'S1143E'
            	else:
            		command = 'S1148E'
            else:
            	balance = 3
            	command = 'S1150E'
            """ #기존 알고리즘으로 굳이 주행하고 싶다면 주석을 해재하고 아래의 기울기 인식을 통한 직선주행 알고리즘을 주석처리하길 바란다.
            if (getLean(rightLane) < rightSlope[0] + fixValue[0] or getLean(rightLane) > rightSlope[0] - fixValue[0]) or (getLean(leftLane) < leftSlope[0] + fixValue[0] and getLean(leftLane) > leftSlope[0] - fixValue[0]):  
            	balance = 1
            	if centerPoint_X > center + 20:
            		command = 'S1162E'
            	elif centerPoint_X > center + 10:
            		command = 'S1157E'
            	else:
            		command = 'S1152E'
            elif centerPoint_X < center - 5:
            	balance = 2
            	if centerPoint_X < center - 20:
            		command = 'S1138E'
            	elif centerPoint_X < center - 10:
            		command = 'S1143E'
            	else:
            		command = 'S1148E'
            else:
            	balance = 3
            	command = 'S1150E'

            		

        elif right and not left and end and V3D < 90 and V4D != 0:
        	command = 'S1135E'
        	status = ONLEFT

        elif not right and left and end and V5D < 90 and V4D != 0:

            command = 'S1165E'
            status = ONRIGHT


        
    elif status is ONLEFT :

        if right and not left and end :

        	if V3D < 80:
        		command = 'S1130E'
        	if V3D < 70:
        		command = 'S1125E'
        	if V3D < 60:
        		command = 'S1120E'
        	if V3D < 50:
        		command = 'S1115E'

        elif right and left and (H2LD > 60 and H2RD > 60) or (H3LD > 65 and H3RD > 65):
        	command = 'S1150E'
        	status = ONSTRAIGHT
                
        elif end:
        	if V3D < 80:
        		command = 'S1130E'
        	if V3D < 70:
        		command = 'S1125E'
        	if V3D < 60:
        		command = 'S1120E'
        	if V3D < 50:
        		command = 'S1115E'

        		
        else:
        	command = prevComm


    elif status == ONRIGHT:

        if not right and left and end:
        	if V4D < 80:
        		command = 'S1170E'
        	if V4D < 70:
        		command = 'S1175E'
        elif not right and left:
        	command = 'S1177E'
        	status = ONSCORNER
        elif right and left:
        	if V4D == 0 or V4D >= 80:
        	    command = 'S1150E'
        	    status = ONSTRAIGHT
        elif end:
        	if V4D < 80:
        		command = 'S1170E'
        	if V4D < 70:
        		command = 'S1175E'
        
        else: command = prevComm

    elif status == ONSCORNER :

    	if not right and left:
    		if V4D < 80:
    			command = 'S1178E'
    		if V4D < 70:
    			command = 'S1179E'
    		if V4D < 60:
    			command = 'S1180E'
    		if V4D < 50:
    			command = 'S1182E'

    	if right and left:
    		if (H2LD > 60 and H2RD > 60) or (H3LD > 65 and H3RD > 65):
    			command = 'S1150E'
    			status = ONSTRAIGHT

    
    print('========== Final Debuging ==========')
    print('')
    print('Final Command : ', command)

    if right:
        print('Right Line : True')
        print('Right Line : ', rightLane)
        print('Right Line Gradient : ', getLean(rightLane))
    else: print('Right Line : False')

    print('')

    if left:
        print('Left Line : True')
        print('Left Line : ', leftLane)
        print('Left Line Gradient : ', getLean(leftLane))
    else: print('Left Line : False')

    print('')

    if end:
    	print('End Line : True')
    	print('End Line : ', endLane)
    	print('End Line Gradient : ', getLean(endLane))
    else: print('End Line : False')

    print('')

    if status == ONSTRAIGHT: print('Status : Straight')
    elif status == ONLEFT: print('Status : Left')
    elif status == ONRIGHT: print('Status : Right')
    elif status == ONSCORNER: print('Status : S Corner')
    else : print('Status : Unknown')

    print('')

    if startLine == True: print('Start Line : True')
    else: print('Start Line : False')

    print('')

    if balance == 1: print('Balancing : Going Right')
    elif balance == 2: print('Balancing : Going Left')
    elif balance == 3: print('Balancing : Going Straight')
    else: print('Balancing : NONE')

    print('')

    if st: print('STOPED!')
    else : print('Keep going')

    print('')

    if light == 0: print('Light : Green')
    elif light == 1: print('Light : Red')
    else: print('Light : Not Detected')

    print('')

    print('LiDAR (mm) : ', LiDAR)
    print('')
    print('====================================')
    print('\n')
    print('\n')
    print('\n')
    
    return command, status, light