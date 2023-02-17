from jajucha.planning import BasePlanning
from jajucha.graphics import Graphics
from jajucha.control import mtx
import cv2
import numpy as np
import time
import datetime

nowDate = datetime.datetime.now()

speed = 0.001



def makeVx(length):
    Vx = []
    for i in range(length):
        Vx.append(round(640 / (length + 1)) * (i+1))
    return Vx

def makeLRy(length):
    LRy = []
    for i in range(length):
        LRy.append(round(255 / (length + 1)) *(i+1))
    return LRy 

def incline(x1, y1, x2, y2):
    if (x1 - x2) is not 0:
        return (y1 - y2) / (x1 - x2)
    else:
        return 5000

def sigmoid(x):
    return 1/(1+np.exp(-x))

def average(List):
    return sum(List) / len(List)

"""

이 아래서부터 class 선언 전까지 있는 모든 함수는 사용자가 자주차 알고리즘을 작성할 때 도움이 되기 위한 일종의 UI 함수들로, 실제 대회에 나가거나
컴퓨터 성능이 좋지 못한 경우에는 이 아래에 모든 함수를 주석 처리하여도 좋다. 단, 주석처리를 할 때 실제 주행 알고리즘이 돌아가는 process 함수에 적힌 
아래의 함수 구문들도 철저히 주석처리를 해야한다. 하지만 아래 함수들을 이용하는 것을 권장한다. 그렇지 않으면 이해가 어렵기 때문.


만약 이해하는 데에 어렵거나 힘든 점이 있으면 언제든 메일을 주어도 좋다. ingabbang@gmail.com
메일이 힘들다면 (메일을 주기 힘든 정도면 도대체 무슨 상황일까.) 동아리 담당 선생님께 여쭤보아라.

여러번 실행하고 또 반복하면서 이해하는 것이 가장 중요하다. 귀찮다고 생각하지 말고 끝까지 밀어붙여라.
너희가 해야 할 것은 단순하다. 나열하면 다음과 같다.

0. 직진 상태를 유지하는 주행부터 성공하기
1. 곡선 좌,우회전 주행부터 성공하기
2. 직각 좌,우회전 주행 성공하기
3. 직각과 곡선을 번갈아 적어도 10번 이상 모의주행해서 정상주행하면 다음으로, 아니면 1,2번으로 넘어가기.
4. 라이다를 통한 정지 알고리즘 (작성하는 데 1분걸림.)
5. 신호등 감지 알고리즘 작성 (많이 까다로움)
6. 5번까지 완료했으면 트랙을 마음대로 조성해가면서 5번 이상의 모의주행을 할 것. 여기까지 왔다면 대회날 할 수 있는 건 기도밖엔 없음.

"""

def imageVTexting(image, List, length):
    Vx = makeVx(length)
    NumOfEmptyRoad = List.index(max(List))

    for i in range(length):
        cv2.line(image, (Vx[i], 480), (Vx[i], 480 - List[i]), (211, 211, 211), 3)
        cv2.rectangle(image, (Vx[i]-20, 480-List[i]-25), (Vx[i] + 10, 480 - List[i] - 5), (255, 255, 255), -1)
        cv2.putText(image, str(List[i]), (Vx[i]-20, 480 - List[i] -10 ), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

    """

    for i in range(round((length-1)/2)):
        if List[i] is not 255 and List[length-1-i] is not 255:
            cv2.line(image, (Vx[i], 480 - List[i]), (Vx[length-1-i], 480 - List[length-1-i]), (0, 212, 255), 4)
            cv2.putText(image, str(round(incline(Vx[length-1-i], List[length-1-i], Vx[i], List[i]), 2)), (int((Vx[i] + Vx[length-1-i]) / 2) - 10, int(480 - (List[i] + List[length-1-i]) / 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

        elif List[i] is 255 and List[length-1-i] is 255:
            cv2.line(image, (Vx[i], 480), (Vx[length-1-i], 480), (0, 212, 255), 4)
            cv2.putText(image, str(round(incline(Vx[length-1-i], 0, Vx[i], 0), 2)), (int((Vx[i] + Vx[length-1-i]) / 2) - 10, 480), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

        elif List[i] is 255 and List[length-1-i] is not 255:
            cv2.line(image, (Vx[i], 480), (Vx[length-1-i], 480 - List[length-1-i]), (0, 212, 255), 4)
            cv2.putText(image, str(round(incline(Vx[length-1-i], List[length-1-i], Vx[i], 0), 2)), (int((Vx[i] + Vx[length-1-i]) / 2) - 10, int(480 - (List[length-1-i]) / 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

        elif List[length-1-i] is 255 and List[i] is not 255:
            cv2.line(image, (Vx[i], 480 - List[i]), (Vx[length-1-i], 480), (0, 212, 255), 4)
            cv2.putText(image, str(round(incline(Vx[length-1-i], 0, Vx[i], List[i]), 2)), (int((Vx[i] + Vx[length-1-i]) / 2) - 10, int(480 - (List[i]) / 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    """

    cv2.line(image, (Vx[NumOfEmptyRoad], 480), (Vx[NumOfEmptyRoad], 480 - List[NumOfEmptyRoad]), (0, 255, 0), 3)
    cv2.line(image, (0,480-255), (640, 480-255), (128, 128, 128), 3)
    cv2.putText(image, "MAX V LINE", (10, 480-255-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.rectangle(image, (5, 10), (120, 40), (255, 255, 255), -1)
    cv2.putText(image, "Max V =", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(image, str(NumOfEmptyRoad), (100, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)

    return image

def imageLTexting(image, List, length):
    LRy = makeLRy(length)
    count = 0

    cv2.line(image, (320, 480), (320, 480-255), (0, 212, 255), 4)

    for i in range(round(length/2) + 5, length):
        if List[i] < 320:
            cv2.line(image, (320, 225 + LRy[i]), (320-List[i], 225 + LRy[i]), (200, 200, 200), 3)
            count += 1
        else:
            count += 0
    return image, count

def imageRTexting(image, List, e, length):

    LRy = makeLRy(length)
    count = 0

    for i in range(round(length/2) + 5, length):
        if List[i] < 321:
            cv2.line(image, (320, 225 + LRy[i]), (320+List[i], 225 + LRy[i]), (153, 91, 70), 3)
            count += 1
        else:
            count += 0
        
    cv2.rectangle(image, (5, 10), (100, 40), (255, 255, 255), -1)
    cv2.putText(image, "e = ", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)
    cv2.putText(image, str(e), (40, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)

    return image, count

def middleTexting(image, ListR, ListL, length): 

    LRy = makeLRy(length)

    x = []
    y = []

    for i in range(len(ListR)):
        x.append(round(320 + (ListR[i] - ListL[i])/2)) 
        y.append(225 + LRy[i]) 
    for i in range(round(len(ListR) / 2), len(ListR)):
        if ListR[i-1] < 321 or ListL[i-1] < 320:
            cv2.line(image, (x[i], y[i]), (x[i-1], y[i-1]), (0,255,0), 3)

    return image

def definiteTurning(image, Lcount, Rcount): #이미지의 중심선에 수직인 직선들이 차선에 닿음을 기준으로 차량의 주행 상태를 결정함.
    if Lcount > Rcount and (Lcount - Rcount) > 2: #왼쪽 차선이 오른쪽 차선보다 인식됨이 더 많은 경우(예: 오른쪽 차선이 보이지 않음, 즉 우회전이 필요할 때)
        cv2.putText(image, "Turn Right, Level " + str(Lcount - Rcount), (180, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        state = "Turn Right"
    elif Lcount < Rcount and (Rcount - Lcount) > 2: #오른쪽 차선이 왼쪽 차선보다 인식됨이 더 많은 경우(예: 왼쪽 차선이 보이지 않음, 즉 좌회전이 필요할 때)
        cv2.putText(image, "Turn Left, Level " + str(Rcount - Lcount), (180, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        state = "Turn Left"
    elif ((Lcount - Rcount) < 3 or (Rcount - Lcount) < 3) and (Lcount != 0 and Rcount != 0): #두 횟수가 0인 경우에는 차선 인식이 이루어지지 않는 상황이므로 그 경우는 제외
        cv2.putText(image, "Straight", (180, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        state = "Straight"
    elif Lcount is 0 and Rcount is not 0: #왼쪽 차선이 인식이 아예 되지 않는 경우
        cv2.putText(image, "No Left Line Detected", (180, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        state = "No Detection L"
    elif Lcount is not 0 and Rcount is 0: #오른쪽 차선이 인식이 아예 되지 않는 경우
        cv2.putText(image, "No Right Line Detected", (180, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        state = "No Detection R"
    else: #차가 양쪽 차선을 인지하지 못하는 경우
        cv2.putText(image, "No All Line Detected", (180, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        state = "No Detection"

    return image, state

def checkingLineDetected(ListL, ListR, rows, previousE):
    for i in range(round(rows/2) + 5, rows):
        if ListL[i] + ListR[i] < 320:
            e = previousE
        else:
            e = ListR[i] - ListL[i]
            break

    return e

def linedetecting(ListL, ListR, rows):
    for i in range(round(rows/2) + 5, rows):
        if ListL[i] + ListR[i] < 320:
            return True

    return False

#차선이 하나라도 감지되면 이전에 감지된 주행상태에 따라 작동하기

def controlpad(img, ListL, ListR,ListV, rows, drv):
    cv2.putText(img, "Line Detection", (10, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA)
    cv2.putText(img, drv, (10, 35 + 20*(rows - (round(rows/2)+2))), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(img, "Avr V " + str(average(ListV)), (10, 35 + 20 * (rows - (round(rows/2) + 3))), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    for i in range(round(rows/2) + 5, rows):
        cv2.putText(img, "No. "+ str(i), (10, 35 + 20* (i-(round(rows/2)+5))), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
        if ListL[i] + ListR[i] < 320:
            cv2.circle(img, (90,  30 + 20* (i-(round(rows/2)+5))), 8, (0, 0, 255), -1)
        else:
            cv2.circle(img, (90,  30 + 20* (i-(round(rows/2)+5))), 8, (0, 255, 0), -1)

        if ListL[i] == 320:
            cv2.circle(img, (120, 30 + 20* (i-(round(rows/2)+5))), 6, (0, 212, 255), -1)
        else:
            cv2.circle(img, (120, 30 + 20* (i-(round(rows/2)+5))), 6, (255, 0, 0), -1)

        if ListR[i] == 321:
            cv2.circle(img, (140, 30 + 20* (i-(round(rows/2)+5))), 6, (0, 212, 255), -1)
        else:
            cv2.circle(img, (140, 30 + 20* (i-(round(rows/2)+5))), 6, (255, 0, 0), -1)

    return img




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
        # 자주차는 완벽하지 않다. 차량에 따라서 steer 값이 0이어도 직진을 하지 않는 경우가 있다. 그때 basicStepM 값을 조정해 직진하도록 만들어야 한다. 
        # 양수는 기본이 오른쪽으로 치우치게 한다. 그러니까 steer를 0으로 했을 때 자주차가 왼쪽으로 슬슬 나간다면 값을 양수로 둔다. ACCC 자주차를 다루면 이 값은
        # 건들이지 않아도 된다. 다른 자주차를 다루게 되면 우선 이 값부터 건들이는 것이 우선이다.
        #
        # 음수는 기본이 왼쪽으로 치우치게 한다. 위의 상황과 반대일 때 조정하면 된다.
        self.vars.frameNum = 1
        self.vars.drivingState = "Straight"
        self.vars.e = 0
        self.vars.cols = 10
        self.vars.rows = 50
        self.vars.Rcount = 0
        self.vars.Lcount = 0


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

        cntrlpad =  np.zeros((700, 160, 3), np.uint8) + 255

        # canny 이미지 보기
        #canny = self.canny(frontImage)
        #self.imshow('canny', canny)

        # 차선 정보 파악
        V, L, R = self.gridFront(frontImage, cols=self.vars.cols, rows=self.vars.rows) # 이 값을 바꿀 수 있다는 것을 명심하라!
        # V, L, R = self.gridFront(frontImage, cols=7, rows=3) : 전방 이미지 행선, 열선 그리기
        # rearV, rearL, rearR = self.gridRear(rearImage, cols=4, rows=6) : 후방 이미지도 가능
        # rows : 행선값  cols : 열선값 (row, column) ex) rows=3 : 행선이 3개이고, 총 4개의 행 칸이 생성
        # L[0], L[1], L[2], R[0], R[1], R[2], V[0]~v[6]

        # 각 변수의 최댓값
        if V[3] == 255:  # V[i]가 잡히지 않은 경우
            ...
        if L[2] == 325:  # L[i]가 잡히지 않은 경우  (중앙 픽셀이 324라서 왼쪽으로 최대 324) #이거 거짓말이다. L의 최댓값은 320이다.
            ...
        if R[2] == 316:  # R[i]가 잡히지 않은 경우  (중앙 픽셀이 324라서 오른쪽으로 최대 315) #이것도 거짓말이다. R의 최댓값은 321이다.
            ...

        #State 이미지 열기
        Vimage = frontImage.copy()
        imageVTexting(Vimage, V, self.vars.cols)
        self.imshow('V', Vimage)

        LRimage = frontImage.copy()
        LRimage, self.vars.Lcount = imageLTexting(LRimage, L, self.vars.rows)
        middleTexting(LRimage, R, L, self.vars.rows)
        LRimage, drivingState = definiteTurning(LRimage, self.vars.Lcount, self.vars.Rcount)



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

        if not linedetecting(L, R, self.vars.rows):

            if drivingState == "Straight": #직진 상황
                e = 0

            elif drivingState == "Turn Right": #우회전 상황
                e = 1.1 *  checkingLineDetected(L, R, self.vars.rows, self.vars.e)

            elif drivingState == "Turn Left": #좌회전 상황
                e = 1.1 * checkingLineDetected(L, R, self.vars.rows, self.vars.e)

            elif drivingState == "No Detection L": #왼쪽 차선 인식 못하는 상황
                e = 2 * checkingLineDetected(L, R, self.vars.rows, self.vars.e)

            elif drivingState == "No Detection R": #오른쪽 차선 인식 못하는 상황
                e = 2 * checkingLineDetected(L, R, self.vars.rows, self.vars.e)

            elif drivingState == "No Detection": #양쪽 차선 인식 못하는 상황
                e = self.vars.e #이전 주행상태 유지
            else:
                e = self.vars.e
        else:
            if average(V) <= 70:
                if self.vars.drivingState == "Turn Right":
                    e = 300
                elif self.vars.drivingState == "Turn Left":
                    e = -300
                elif self.vars.drivingState == "No Detection L":
                    e = -300
                elif self.vars.drivingState == "No Detection R":
                    e = 300
                else:
                    e = self.vars.e
            else:
                e = self.vars.e
            

        #cv2.imshow('Front Grid Image', frontImage)

        
        OnlyForPrinting = []

        for i in range(round(self.vars.rows/2)+5, self.vars.rows):
            OnlyForPrinting.append(L[i]+R[i])

        print(OnlyForPrinting)
        

        steer = round((1/3)*e + self.vars.basicStepM)
        if steer > 100:
            steer = 100
        elif steer < -100:
            steer = -100
        #steer = self.vars.basicStepM + steer
        velocity = 40

        LRimage, self.vars.Rcount = imageRTexting(LRimage, R, e, self.vars.rows)
        self.imshow('LR',LRimage)

        cntrlpad = controlpad(cntrlpad, L, R, V, self.vars.rows, drivingState)
        self.imshow('CONTROL', cntrlpad)

        self.vars.frameNum = self.vars.frameNum + 1

        self.vars.drivingState = drivingState
        self.vars.e = e
        self.vars.steer = steer
        self.vars.velocity = velocity
        return self.vars.steer, self.vars.velocity


if __name__ == "__main__":
    g = Graphics(Planning)  # 자주차 컨트롤러 실행
    g.root.mainloop()  # 클릭 이벤트 처리
    g.exit()  # 자주차 컨트롤러 종료