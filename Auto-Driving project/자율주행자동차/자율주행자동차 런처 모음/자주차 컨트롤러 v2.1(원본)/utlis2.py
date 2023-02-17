import cv2
import numpy as np
 
def thresholding(img):
    imgHsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    lowerWhite = np.array([80,0,0])
    upperWhite = np.array([255,160,255])
    maskWhite = cv2.inRange(imgHsv,lowerWhite,upperWhite)
    return maskWhite
 
def warpImg(img,points,w,h,inv = False):
    pts1 = np.float32(points)
    pts2 = np.float32([[0,0],[w,0],[0,h],[w,h]])
    if inv:
        matrix = cv2.getPerspectiveTransform(pts2, pts1)
    else:
        matrix = cv2.getPerspectiveTransform(pts1,pts2)
    imgWarp = cv2.warpPerspective(img,matrix,(w,h))
    return imgWarp
 
def drawPoints(img,points,font):
    for x in range(4):
        cv2.circle(img,(int(points[x][0]),int(points[x][1])),15,(0,0,255),cv2.FILLED)
        cv2.putText(img,str(int(x+1)), (int(points[x][0])-8, int(points[x][1])+8), font, 0.7,(0,255,0),1)
    return img

def drawLines(img, y, center, line, status='NONE'):

    font=cv2.FONT_HERSHEY_SIMPLEX
    
    #이미지 한 가운데에 선을 그립니다.
    cv2.line(img, (center, 150), (center, 400), (0, 0, 0), 4)
    cv2.line(img, (center, 150), (center, 400), (255, 255, 0), 2)

     #구해진 추세선에 맞춰진 원을 그립니다.
    for i in range(200, 480, 20):
        cv2.circle(img, (int(line(i)), i), 5, (255, 0, 0), -1)
    
    #추세선을 그립니다.
    cv2.line(img, (int(line(200)), 200), (int(line(480)), 480),(255, 0, 0), 2)


    #왼쪽 오른쪽을 구별할 때 쓰는 보조선을 긋습니다.
    cv2.line(img, (0, 400), (640, 400), (0, 255, 0), 3)
    cv2.putText(img, 'Distinction Line', (0, 390), font, 0.7, (0, 255, 0), 2)

    #차선을 인식하는 한계선을 긋습니다.
    cv2.line(img, (0, 300), (640, 300), (0, 158, 232), 2)
    cv2.putText(img, 'Line Limits', (0, 290), font, 0.7, (0, 158, 232), 2)

    #steer 값을 결정하는 점을 찍습니다.
    cv2.circle(img, (int(line(y)),y), 7, (255, 0, 0), -1)
    cv2.circle(img, (int(line(y)),y), 5, (0, 255, 0), -1)

    #그 점을 기준으로 길이가 40픽셀인 직선을 그립니다.
    cv2.line(img, (int(line(y)), y - 20), (int(line(y)), y + 20),(0, 255, 0), 2)

    #가운데부터 지정된 점까지 직선을 그립니다.
    cv2.line(img, (center, y), (int(line(y)),y), (0, 0, 0), 4)
    cv2.line(img, (center, y), (int(line(y)),y), (255, 0, 0), 2)

    cv2.putText(img, str(abs(int(line(y) - center))), (int((center + line(y))/2) - 30, int(y) - 10), font, 1,(0,0,0),2)
    cv2.putText(img, "line = " + str(status), (int(center - 60),450), font, 0.7, (0,255, 0), 2)

    return img

def showValues(img, e, center, speed, steer, steerKorean, drivingStatus, frame, red, noLineCount = 0):

    font = cv2.FONT_HERSHEY_SIMPLEX

    #값이 잘 보이도록 사각형을 그림 위에 그립니다.
    #img = cv2.rectangle(img, (0,0), (140, 50),  (255, 255, 255), -1)

    #값이 잘 보이도록 사각형을 그립니다.
    img = cv2.rectangle(img, (0, 0), (640, 180), (255, 255, 255), -1)

    cv2.putText(img, "Driving Status", (int(center - 85), 25), font, 0.7, (255, 0, 0), 2)

    if drivingStatus == '전진':
        img = cv2.putText(img, "Straight", (int(center - 70), 60), font, 1, (255,0,0), 2)

    elif drivingStatus == '좌회전':
        img = cv2.putText(img, "Left", (int(center-30), 60), font, 1, (255,0,0), 2)

    elif drivingStatus == '우회전':
        img = cv2.putText(img, "Right", (int(center-35), 60), font, 1, (255,0,0), 2)

    #값을 표시합니다.
    cv2.putText(img, "e = " + str(int(e)), (520,45), font, 0.7,(255,0,0),2)

    cv2.putText(img, "Steer Status", (10, 25), font, 0.5, (255, 0, 0), 2)

    cv2.putText(img, "steer = " + str(int(steer)), (478,65), font, 0.7,(50,100,0),2)
    cv2.putText(img, steerKorean, (10, 55), font, 0.7, (255, 0, 0), 2)
    
    img = cv2.putText(img, "speed = "+ str(speed), (468, 25), font, 0.7, (255, 0, 0), 2)
    img = cv2.putText(img, "frame number : " + str(frame), (10, 90), font, 0.7, (255,0,0), 2)
    img = cv2.putText(img, "Traffic count : " + str(red), (440, 90), font, 0.7, (255, 0, 0), 2)
    img = cv2.putText(img, "No Line Count : " + str(noLineCount), (10, 120), font, 0.7,(255, 0,0), 2)

    if noLineCount > 4:
        img = cv2.putText(img, "M.A Activated!", (240, 120), font, 0.7, (0, 0, 255), 2)
    if red > 4:
        img = cv2.putText(img, "TRACFFIC STOP", (400, 120), font, 0.7, (0, 0, 255), 2)


    return img

 

def steerFunction(tilt, yee, modify, chatsu, choose):
    if choose:
        return tilt * ((yee - modify) ** chatsu) + modify
    else:
        return tilt * ((yee + modify) ** chatsu) + modify

def showDirection(steerValue): #steer 값에 따라 이 자동차가 어떤 조향을 나타내는지를 보여주는 함수. 좀더 '인간적인' 조향을 나타낸다.
    status = ''
    if steerValue < -60:
        print("조향 : 급격한 좌회전")
        status = "Left 4"
    elif steerValue >= -60 and steerValue < -40:
        print("조향 : 거센 좌회전")
        status = "Left 3"
    elif steerValue >= -40 and steerValue < -20:
        print("조향 : 조금 거센 좌회전")
        status = "Left 2"

    elif steerValue >= -20 and steerValue <-10 :
        print("조향 : 약한 좌회전")
        status = "Left 1"
    elif steerValue <= 20 and steerValue > 10:
        print("조향 : 약한 우회전")
        status = "Right 1"
    elif steerValue <= 40 and steerValue > 20:
        print("조향 : 조금 거센 우회전")
        status = "Right 2"
    elif steerValue <= 60 and steerValue > 40:
        print("조향 : 거센 우회전")
        status = "Right 3"
    elif steerValue > 60:
        print("조향 : 급격한 우회전")
        status = "Right 4"
    else:
        print("조향 : 직진") 
        status = "straight"

    if steerValue > 80:
        steerValue = 80
    elif steerValue < -80:
        steerValue = -80

    return steerValue, status

