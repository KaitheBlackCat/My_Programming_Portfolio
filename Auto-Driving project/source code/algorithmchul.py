from jajuchaUtil import *
import matplotlib.pyplot as plt
import numpy as np

################################ set cascade ################################
trafficlight_cascade = cv2.CascadeClassifier('trafficlight_cascade.xml')
#############################################################################

slope = []
V3Dvalue = []
V4Dvalue = []
V5Dvalue = []

global x, y
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

        light_positin = -1
        
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

    if 0 < LiDAR and LiDAR < 130 :

        print('Obastacle Detected, distance : %dmm' %LiDAR)

        command = 'S0150E'
    

    elif left and right :

        status = ONSTRAIGHT

        if H2RD != 160 and H2LD !=160 :

            if V1D != 0 and V1D !=120 and V7D !=0 and V7D !=120:

                print('center point calculated by H2 Point %d, %d' % (H2LD, H2RD))

                centerpoint_X = int((345 - H2LD + H2RD)/2) #차선의 중앙값을 구한다
                    
                print('centerpoint_X =',centerpoint_X) #center는 카메라 기준이다(차의 위치)그리고 centerpoint는 차선의 중간
                    
                if centerpoint_X > center + 30:

                    command = 'S1170E'

                elif centerpoint_X > center + 5:

                    command = 'S1161E'

                elif centerpoint_X < center - 30:

                    command = 'S1130E'

                elif centerpoint_X < center - 5:

                    command = 'S1140E'

                else :

                    command = 'S1150E'

    elif right and end :

        status = ONLEFT

        if V4D < 30:

            command = 'S1115E'

        if V4D > 30 and V4D < 50  :

            command = 'S1135E'
        
    elif left and end :

        status = ONRIGHT

        if V4D < 30 :

            command = 'S1185E'

        if V4D > 30 and V4D < 50 :

            command = 'S1165E'

    elif end and not right and not left :

        status = ONLEFT
  
        command = 'S1115E' 

    print('V1D =', V1D)
    print('V2D =', V2D)
    print('V3D =', V3D)
    print('V4D =', V4D)
    print('V5D =', V5D)
    print('V6D =', V6D)
    print('V7D =', V7D)

    try:
        if right == True:
            print("right exist")
            print(getLean(rightLane))
            slope.append(getLean(rightLane))

        if V3D >= 0:
            V3Dvalue.append(V3D)

        if V4D >= 0:
            V4Dvalue.append(V4D)

        if V5D >= 0:
            V5Dvalue.append(V5D)

        else:
            print("else")

    except:
        print("error")



    if len(slope) == 200:

        a = np.arange(0,len(slope))
        g = np.arange(0,len(V3Dvalue))
        i = np.arange(0,len(V4Dvalue))
        k = np.arange(0,len(V5Dvalue))

        b = slope
        h = V3Dvalue
        j = V4Dvalue
        l = V5Dvalue

        plt.xlim(0,201)

        plt.plot(a,b,'r', label = 'slope')
        plt.plot(g,h,'b', label = 'V3D')
        plt.plot(i,j,'m', label = 'V4D')
        plt.plot(k,l,'c', label = 'V5D')



        plt.legend()
        plt.show()

        print("V3Dlen = ", len(V3Dvalue))
        print("V4Dlen = ", len(V4Dvalue))
        print("V5Dlen = ", len(V5Dvalue))

        print("slope = ", len(slope))

    return command, status, light