from jajuchaUtil import *

################################ set cascade ################################
trafficlight_cascade = cv2.CascadeClassifier('trafficlight_cascade.xml')
#############################################################################

def findTrafficLight(image):
    # ROI 설정
    height, width = image.shape[:2]
    ROI_image = image[:120, :]
    light = False # Red light
    
    # 학습 모델을 사용하여 신호등 위치 추정
    ROI_gray = cv2.cvtColor(ROI_image, cv2.COLOR_BGR2GRAY)
    # trafficlight = trafficlight_cascade.detectMultiScale(ROI_gray, 1.3, 5)
    trafficlight = trafficlight_cascade.detectMultiScale(ROI_gray, 1.1, 4)
    cv2.imshow('roi', ROI_gray)

    for (x,y,w,h) in trafficlight:
        # 신호등이 30cm 이상 밖에 있다고 판단하고 그대로 함수 종료
        if w < 35:
            break

        # 신호등이 30cm 이내 거리에 있다고 판단
        ROI = ROI_gray[y:y+h, x:x+w]        
        ROI_X = ROI.shape[1]
        ROI_Y = ROI.shape[0]

        light_position = -1
        
        # 신호등 위치에서 밝기를 기준으로 왼쪽과 오른쪽 중 어느곳에 불이 들어와 있는지 판단
        for i in range(ROI_X):
            for j in range(ROI_Y):
                if ROI[i, j] > 150: # 밝기 비교
                    light_position = j
        
        # 오른쪽에 불이 들어와 있다면 green을 리턴 (기본값 False)
        if light_position > ROI_X/2 + 4:
            light = True
        elif light_position == ROI_X/2 + 4:
            light = True
        elif light_position < ROI_X/2 + 4:
            light = False
        
        return (x, y, w, h, light)
    
    # 발견된 신호등이 없으면 그냥 False 리턴
    # False is red / True is green or yellow
    return False

def autoDrive_algorithm(original_img, canny_img, points, lines, LiDAR, prevComm, status, light):
    height, width = canny_img.shape[:2]
    center = int(width/2) - 2
    debug = True
    # debug = False
    command = prevComm

    result = findTrafficLight(original_img)
    
    if result != False:
        x = result[0]
        y = result[1]
        w = result[2]
        h = result[3]
        light = result[4]
        cv2.rectangle(original_img,(x,y),(x+w,y+h),(0,0,255),2)

    H1LD = points['H1LD']
    H1RD = points['H1RD']
    H2LD = points['H2LD']
    H2RD = points['H2RD']
    H3LD = points['H3LD']
    H3RD = points['H3RD']
    H1Y = int(height/4)*1
    H2Y = int(height/4)*2
    H3Y = int(height/4)*3

    V1D = points['V1D']
    V2D = points['V2D']
    V3D = points['V3D']
    V4D = points['V4D']
    V5D = points['V5D']
    V6D = points['V6D']
    V7D = points['V7D']
    V1X = int(width/8)*1        # 1 block distance by y axis
    V2X = int(width/8)*2        # 2 blocks distance by y axis
    V3X = int(width/8)*3        # 3 blocks distance by y axis
    V4X = int(width/8)*4        # 4 blocks distance by y axis
    V5X = int(width/8)*5        # 5 blocks distance by y axis
    V6X = int(width/8)*6        # 6 blocks distance by y axis
    V7X = int(width/8)*7        # 7 blocks distance by y axis

    XD1 = int(width/8)*1        # 1 block distance by x axis
    XD2 = int(width/8)*2        # 2 blocks distance by x axis
    XD3 = int(width/8)*3        # 3 blocks distance by x axis
    
    leftLane, rightLane, endLane = getLane(lines)
    left = right = end = False
    if len(leftLane) != 0:
        left = True
    if len(rightLane) != 0:
        right = True
    if len(endLane) != 0:
        end = True
#############################################################

    #라이다
    if LiDAR > 0 and LiDAR < 300:
        command = 'S0150E'
    #직진 상황
    elif status is ONSTRAIGHT:
        if right and left:
            if debug:
                print('case 1,2 / ONSTRAIGHT')
            if H2LD != 160 and H2RD != 160:
                centerPoint_X = (340 - H2LD + H2RD)/2 
                if debug: print('centerPoint_X - H2 : %d' % centerPoint_X )
            elif H3LD != 160 and H3RD != 160:
                centerPoint_X = (340 - H3LD + H3RD)/2 
                if debug: print('centerPoint_X - H3 : %d' % centerPoint_X )
            else: 
                return prevComm, status, light

            if centerPoint_X > center + 0:
                command = 'S1160E'
                if centerPoint_X > center + 50:
                    command = 'S1170E'
            elif center > centerPoint_X + 0:
                command = 'S1140E'
                if center > centerPoint_X + 50:
                    command = 'S1130E'
            else:
                command = 'S1150E'
                
        elif right and not left:
            print('status change : ONSTRAIGHT -> ONLEFT')
            status = ONLEFT
            command = 'S1140E'
        elif left and not right:
            print('status change : ONSTRAIGHT -> ONRIGHT')
            status = ONRIGHT
            command = 'S1160E'

    #좌회전 상황
    elif status is ONLEFT:
        if right and not left and end: # case 3
            if debug:
                print('case 3 / ONLEFT')
            if V4D < 70:
                command = 'S1130E' 
            if V4D < 40:
                command = 'S1115E'
        elif right and not left:
            command = 'S1130E'
        elif right and left:
            if debug:
                print('case 1,2 / ONLEFT')
            if (H2LD > 60 and H2RD > 60) or (H3LD > 65 and H3RD > 65):
                if debug:
                    print('status change : ONLEFT -> ONSTRAIGHT')
                command = 'S1150E'
                status = ONSTRAIGHT
        elif end:
            print('only end line detected')
            command = 'S1120E'
        elif not right and left:
            print('status change : !ONSCORNER!')
            status = ONSCORNER
        else:
            print('out of range')
            command = prevComm
    #우회전 상황
    elif status is ONRIGHT:
        if not right and left and end: # case 4
            if debug:
                print('case 4 / ONRIGHT')
            if V4D < 60:
                command = 'S1180E'
        elif not right and left:
            command = 'S1170E'
        elif right and left:
            if debug:
                print('case 1,2 / ONRIGHT')
            if (H2LD > 60 and H2RD > 60) or (H3LD > 65 and H3RD > 65):
                if debug:
                    print('status change : ONRIGHT -> ONSTRAIGHT')
                command = 'S1150E'
                status = ONSTRAIGHT
        elif end:
            print('only end line detected')
            command = 'S1180E'
        elif right and not left:
            if getLean(rightLane) > 0:
                command = S1130E
                status = ONLEFT
            else:            
                print('status change : !ONSCORNER!')
                status = ONSCORNER
        else:
            print('out of range')
            command = prevComm   
    #S코너 상황
    elif status is ONSCORNER:
        print('ONSCORNER')
        if right and not left:            
            command = 'S1115E'
        elif not right and left:
            if getLean(rightLane) > 0:
                command = S1130E
                status = ONLEFT            
            else:
                command = 'S1185E'
        elif right and left:
            if debug: 
                print('case 1,2 / ONRIGHT')
            command = 'S1150E'
            status = ONSTRAIGHT
            print('status change : ONSCORNER -> ONSTRAIGHT')

    return command, status, light