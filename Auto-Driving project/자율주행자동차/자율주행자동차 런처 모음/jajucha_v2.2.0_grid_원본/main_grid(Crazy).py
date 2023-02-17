from jajucha.planning import BasePlanning
from jajucha.graphics import Graphics
from jajucha.control import mtx
import cv2
import numpy as np
import time
import datetime

nowDate = datetime.datetime.now()

Vx = [81, 162, 243, 324, 405, 486, 567]
LRy = [192, 128, 64]

e = 0

def imageVTexting(image, List):
    NumOfEmptyRoad = List.index(max(List))
    for i in range(0, 7):
        cv2.line(image, (Vx[i], 480), (Vx[i], 480 - List[i]), (211, 211, 211), 3)
    cv2.line(image, (Vx[NumOfEmptyRoad], 480), (Vx[NumOfEmptyRoad], 480 - List[NumOfEmptyRoad]), (0, 255, 0), 3)
    cv2.line(image, (0,480-255), (640, 480-255), (128, 128, 128), 3)
    cv2.putText(image, "MAX V LINE", (10, 480-255-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.rectangle(image, (5, 10), (120, 40), (255, 255, 255), -1)
    cv2.putText(image, "Max V =", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(image, str(NumOfEmptyRoad), (100, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)

    return image

def imageLTexting(image, List):

    cv2.line(image, (324, 480), (324, 480-255), (0, 212, 255), 4)

    for i in range(0, 3):
        cv2.line(image, (324, 480 - LRy[i]), (324-List[i], 480 - LRy[i]), (255, 255, 255), 3)

    return image

def imageRTexting(image, List, e, steer, MDS):

    for i in range(0, 3):
        cv2.line(image, (324, 480 - LRy[i]), (324+List[i], 480 - LRy[i]), (153, 91, 70), 3)

    cv2.rectangle(image, (5, 10), (100, 40), (255, 255, 255), -1)
    cv2.putText(image, "e = ", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)
    cv2.putText(image, str(e), (40, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)

    if MDS - 5 < steer < MDS + 5:
        cv2.putText(image, "STRAIGHT", (310, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    elif steer <= MDS - 5:
        cv2.putText(image, "TURN LEFT", (310, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    elif steer >= MDS + 5:
        cv2.putText(image, "TURN RIGHT", (310, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    else:
        cv2.putText(image, "????", (310, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

    return image


class Planning(BasePlanning):
    def __init__(self, graphics):
        super().__init__(graphics)
        # --------------------------- #
        self.vars.redCnt = 0  # 변수 설정
        self.vars.greenCnt = 0  # 변수 설정
        self.vars.stop = True
        self.vars.steer = 0
        self.vars.velocity = 0
        # self.vars.stopLength = 100
        # 300으로 설정하면 대략 25cm전에 정지. 200으로 설정하면 대략 16cm 전에 정지. 100으로 설정하면 대략 5cm 전에 정지.
        # 오차를 제외하면 정지 거리 y(cm)는 y = 1/12 x의 식을 만족함.
        # 따라서 아래에 있는 stopLength는 실제 정지했으면 하는 거리(cm)의 값을 작성하기 바람.
        self.vars.stopLength = 30
        self.vars.basicStepM = 25
        self.vars.frameNum = 1
        self.vars.drivingState = "Straight"
        self.vars.e = 0


    def process(self, t, frontImage, rearImage, frontLidar, rearLidar):

        

        """
        자주차의 센서 정보를 바탕으로 조향과 속도를 결정하는 함수
        t: 주행 시점으로부터의 시간 (초)
		frontImage: 전면 카메라 이미지
		rearImage: 후면 카메라 이미지
		frontLidar: 전면 거리 센서 (mm), 0은 오류를 의미함
		rearLidar: 후면 거리 센서 (mm), 0은 오류를 의미함
        """


        # [1] 라이다 처리

        if frontLidar <= 12 * self.vars.stopLength :
        	velocity = 0
        else :
        	velocity = 40

        #velocity = 40

        # canny 이미지 보기
        #canny = self.canny(frontImage)
        #self.imshow('canny', canny)

        # 차선 정보 파악
        V, L, R = self.gridFront(frontImage, cols=7, rows=3)
        # V, L, R = self.gridFront(frontImage, cols=7, rows=3) : 전방 이미지 행선, 열선 그리기
        # rearV, rearL, rearR = self.gridRear(rearImage, cols=4, rows=6) : 후방 이미지도 가능
        # rows : 행선값  cols : 열선값 (row, column) ex) rows=3 : 행선이 3개이고, 총 4개의 행 칸이 생성
        # L[0], L[1], L[2], R[0], R[1], R[2], V[0]~v[6]

        # 각 변수의 최댓값
        if V[3] == 255:  # V[i]가 잡히지 않은 경우
            ...
        if L[2] == 325:  # L[i]가 잡히지 않은 경우  (중앙 픽셀이 324라서 왼쪽으로 최대 324)
            ...
        if R[2] == 316:  # R[i]가 잡히지 않은 경우  (중앙 픽셀이 324라서 오른쪽으로 최대 315)
            ...



        # [2] 주행 처리
        #if L[2] < 325: 
            #print ('Left Line', end="// ")
        #    e = 334 - L[2]

        # 둘 다 잡히지 않은 경우
        #else:
        #    e = 0
        #else:
        #    return self.vars.steer, self.vars.velocity  # 이전 명령

        # 현 주행 상황 파악하기

        e = R[2] - L[2]

        #cv2.imshow('Front Grid Image', frontImage)

        steer = (1/12) * int(e ** 3) + self.vars.basicStepM  # 계수 1/3,
        if steer > 100:
            steer = 100
        elif steer < -100:
            steer = -100
        #steer = self.vars.basicStepM + steer
        # velocity = 40

        print ('L[0]=', L[0], 'L[1]=', L[1], 'L[2]=', L[2], end="  //  ")
        print ('R[0]=', R[0], 'R[1]=', R[1], 'R[2]=', R[2])
        print ('V[0]=', V[0], 'V[1]=', V[1], 'V[2]=', V[2], 'V[3]=', V[3], 'V[4]=', V[4], 'V[5]=', V[5], 'V[6]=', V[6])
        print('frontLidar=', frontLidar, end="..//..")
        print('rearLidar=', rearLidar, end="       => => =>    ")
        print('[e=', e, end="]  ")
        print('[steer=', steer, end="]  ")
        print('[velocity=', velocity, "]  ")
        print('[frame =', self.vars.frameNum,"]")
        #print('[drivingState =', drivingState, "]")
        print()

        #State 이미지 열기
        Vimage = frontImage.copy()
        imageVTexting(Vimage, V)
        self.imshow('V', Vimage)

        LRimage = frontImage.copy()
        imageLTexting(LRimage, L)
        imageRTexting(LRimage, R, e, steer, self.vars.basicStepM)
        self.imshow('LR',LRimage)

        self.vars.frameNum = self.vars.frameNum + 1


        #self.vars.drivingState = drivingState
        self.vars.e = e
        self.vars.steer = steer
        self.vars.velocity = velocity
        return self.vars.steer, self.vars.velocity


if __name__ == "__main__":
    g = Graphics(Planning)  # 자주차 컨트롤러 실행
    g.root.mainloop()  # 클릭 이벤트 처리
    g.exit()  # 자주차 컨트롤러 종료
