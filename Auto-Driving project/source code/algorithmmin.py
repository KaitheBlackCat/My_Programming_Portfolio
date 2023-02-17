from jajuchaUtil import *
import time
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
    H1Y = int(height/3)*1
    H2Y = int(height/3)*2
    H3Y = int(height/3)*3

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

        #LiDAR센서 코딩: LiDAR센서의 값이 0과 300사이이면 멈추라고 명령한다.
    if 0 < LiDAR and LiDAR < 170 :
        print('Obastacle Detected, distance : %dmm' %LiDAR)
        command = 'S0150E'

    else:
        if status == ONSTRAIGHT:
            if left and right: #오른쪽 차선과 왼쪽 차선이 보이는 경우
                if H2RD != 160 and H2LD !=160:
                    #차선의 중앙값을 구한다 나는 약간 오른쪽으로 치우쳐 있는 것이 회전하기에 좋을 것이라 생각하여 320을 340으로 바꾸었다.
                    centerpoint_X = int((350 - H3LD + H3RD)/2) 
                    #center는 카메라 기준이다(차의 위치)그리고 centerpoint는 차선의 중
                    print('centerpoint_X =',centerpoint_X) 
                    #차의 중앙을 우리가 설정한 차선의 중앙(살짝 오른쪽으로 치우쳐진)에 위치하게 하기위한 코딩
                    if centerpoint_X > center + 10: #차선의 중앙이 왼쪽으로 많이 치우쳐진 경우
                        command = 'S1160E'
                    elif centerpoint_X > center + 5: #차선의 중앙이 왼쪽으로 살짝 치우쳐진 경우
                        command = 'S1155E'
                    elif centerpoint_X < center - 10: #차선의 중앙이 오른쪽으로 많이 치우쳐진 경우
                        command = 'S1140E'
                    elif centerpoint_X < center - 5: #차선의 중앙이 오른쪽으로 살짝 치우쳐진 경우
                        command = 'S1145E'
                    else : #차선의 중앙과 차의 중앙이 거의 일치할 경우
                        command = 'S1150E'

            elif not left and right and end: #오른쪽 차선과 끝 차선만 보이는 경우
                status = ONLEFT #상태를 왼쪽으로 가는 것으로 바꾼다

            elif left and not right and end: #왼쪽 차선과 끝 차선만 보이는 경우
                status = ONRIGHT #상태를 오른쪽으로 가는 것으로 바꾼다

        elif status == ONLEFT: #상태가 왼쪽으로 가는 것일 경우
            if not left and right and end: #오른쪽 차선과 끝 차선만 보이는 경우
                print('V4D =', V4D)

                if V4D < 30: #끝 차선이 가까워질 경우
                    command = 'S1115E'

                elif V4D >30 and V4D <40: #끝 차선이 아직 좀 멀리 있을 경우
                    command = 'S1135E'

            elif left and right: #왼쪽 차선과 오른쪽 차선이 다시 보일 경우
                status = ONSTRAIGHT #상태를 직진 상태로 바꾼다

            elif not left and not right and end: # 끝 차선만 보이는 경우
                command = 'S1115E'

            else: #예상하지 못한 경우가 나오면 전 명령을 유지한다
                command = prevComm

        elif status == ONRIGHT: #상태가 오른쪽으로 가는 것일 경우 (밑은 상태가 왼쪽일 경우와 같다.)
            if left and not right and end: # 왼쪽 차선과 끝 차선만 보일 경우
                print('V4D =', V4D)

                if V4D < 30:
                    command = 'S1185E'

                elif V4D >30 and V4< 40:
                    command = 'S1165E'

                else:
                    command = prevComm

            elif left and right:
                status = ONSTRAIGHT

            elif not left and not right and end:
                command = 'S1185E'

            else:
                command = prevComm

    return command, status, light