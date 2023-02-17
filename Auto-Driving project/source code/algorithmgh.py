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
        
        return (x, y, w, h, light)
    
    # 발견된 신호등이 없으면 그냥 False 리턴
    # False is red / True is green
    return False

def autoDrive_algorithm(original_img, canny_img, points, lines, LiDAR, prevComm, status, light):
    height, width = canny_img.shape[:2]
    center = int(width/2) - 2 #int => 버림하고 정수 받음 => pixel이 정수라서
    debug = True # debug 뭔지 모르면 멈추게 하는 거
    # debug = False
    command = prevComm

    result = findTrafficLight(original_img)
    
    if result != False: # != means not equal if문은 반드시 ':'이 들어간다.
        x = result[0]
        y = result[1]
        w = result[2]
        h = result[3]
        light = result[4]
        cv2.rectangle(original_img,(x,y),(x+w,y+h),(0,0,255),2) # B, G, R 순서

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
    
    leftLane, rightLane, endLane = getLane(lines) # []와 비슷 but 수정X
    left = right = end = False
    
    if len(leftLane) != 0:
        left = True
    if len(rightLane) != 0:
        right = True
    if len(endLane) != 0:
        end = True
    
    #if not light:
        #print('Traffic Light: RED')
        #command = 'S0150E'
        
    #if 0 < LiDAR and LiDAR < 300:
    #    print('Obstacle Detected at %dmm' % LiDAR)
    #    command = 'S0150E'


    elif left and right:
        
        
        if 10 < H2LD < 160 and 10 < H2RD < 160:
            status = ONSTRAIGHT
            print ('center point calculated by H2 points %d, %d' % (H2LD, H2RD))
            centerPoint_X = int (320 - H2LD + H2RD)/2 
            print('centerPoint_X=', centerPoint_X) 

            if centerPoint_X > center-5:
                if centerPoint_X > center:
                    command = 'S1160E'
                else:
                    command = 'S1155E'
            elif centerPoint_X < center-5:
                if centerPoint_X < center-10:
                    command = 'S1140E'
                else:
                    command = 'S1145E'

        elif V4D != 120 and V5D is 120:
            command = 'S1155E'
            status = ONSTRAIGHT

        else:
            command = prevComm

    elif right and not left and end:
        #print ('V3D=', V3D, 'V4D=', V4D)

        if V3D < 80:
            command = 'S1115E'
            status = ONLEFT

        else:
            command = prevComm

    elif status is ONLEFT:
        if end:
            command = 'S1115E'     

    elif not right and left and end:
        #print ('V5D=', V3D, 'V4D=', V4D)

        if V5D < 80:
            command = 'S1180E'
            status = ONRIGHT
        else:

            command = prevComm

    elif status is ONSTRAIGHT:
        if not right and not left and end:
            command = 'S1115E'


    else:
        command = prevComm

    # leftlane=[x1, y1, x2, y2]
    #print ('leftLane=',leftLane, 'rightLane=', rightLane, 'endLane=', endLane)
    #true or false
    #print (left, right, end)
    #print (status)
    return command, status, light