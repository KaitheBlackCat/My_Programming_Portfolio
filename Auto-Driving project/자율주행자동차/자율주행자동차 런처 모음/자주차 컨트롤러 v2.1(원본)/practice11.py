from jajucha.planning import BasePlanning
from jajucha.graphics import Graphics
from jajucha.control import mtx
import cv2
import numpy as np
import time
import utlis
import math



class Planning(BasePlanning):
    def __init__(self, graphics):
        super().__init__(graphics)
        # --------------------------- #
        self.vars.status = '전진'
        self.vars.line_detection_right = 300
        self.vars.line_detection_left = 300
        self.vars.pixel_difference_right = 130
        self.vars.pixel_difference_left = 200
        self.vars.first_line_choose = 400
        self.vars.steer = 0  # 조향값은 0인 상태로 변수 설정
        self.vars.velocity = 50  # 속도도 0인 상태로 변수 설정
        self.vars.steer_modify = 8


    def process(self, t, frontImage, rearImage, frontLidar, rearLidar):

        velocity = 50
        steer = 0
        laneImage = frontImage.copy()
        line = None
        frontLines, frontObject = self.processFront(frontImage) # 전면 카메라 이미지 처리
        frontLines.sort(key=lambda x:x[0, 1], reverse=True)
        canny = self.canny(frontImage)
        reds, greens = frontObject

        x = 320
        y = 479
        while y >= 0:
            if canny[y, x] > 0:
                break
            y -= 1

        center_x = mtx[0, 2]

        for i in range(len(frontLines)) :
            if frontLines[i][0, 1] < 300 : # (1) [No 460]은 적절한 값이 아닙니다!! / y1, y2, y3... [No 460]보다 작은가? 그러니까, 차선이 나타는 부분까지만 이미지를 자르자고 하는거야. 어디까지 잘라야 하는걸까?
                continue              # y값이 [No 460]보다 작으면 추세선 안하고 넘어가라(479~[No 460]까지만 추세선 찾기)
            x = frontLines[i][:, 0]    # x=[x1, x2, x3, ...]
            y = frontLines[i][:, 1]    # y=[y1, y2, y3, ...]
            coefficient = np.polyfit(y, x, 1)    # coefficient = [a, b] (단, x = ay + b) 
            line = np.poly1d(coefficient)        # line = ay + b (가장 밑에 닿은 차선의 추세선 식)
            if line(self.vars.first_line_choose) < center_x:   # (2) [No 450]은 적절한 값이 아닙니다!! / 
                      # 만약 line = a*[No 450] + b : y=[No 450]에서의 x값이 중앙보다 왼쪽이면
                line = 'left', i            # line = ('left', i) 인 class, tuple
                break
            else:                           # 만약 line = a*400 + b : y=400에서의 x값이 중앙보다 왼쪽이 아니면
                line = 'right', i           # line = ('right', i) 인 class, tuple
                break

        laneImage = frontImage.copy()

        '''
        if 0 < frontLidar < 500:
        	self.vars.status == '정지'
        	velocity = 0
        '''

        """
        자주차의 센서 정보를 바탕으로 조향과 속도를 결정하는 함수
        t: 주행 시점으로부터의 시간 (초)
		frontImage: 전면 카메라 이미지 480*640*3 [y,x,c] (x,y)좌표의 color(0~255) 정보. 카메라 640*480 이미지를 캘리브레이션 한 것.
		rearImage: 후면 카메라 이미지
		frontLidar: 전면 거리 센서 (mm), 0은 오류를 의미함
		rearLidar: 후면 거리 센서 (mm), 0은 오류를 의미함
        """

        # frontLines = [[x1, y1], [x2, y2], [x3, y3], ...]
        
        # frontLines = [[x1,y1],[x2,y2],[x3,y3], … ] , [[x1,y1], [x2,y2], … ]
        # y1>y2>y3... : 밑의 점부터 표현
        # frontLines[0]이 가장 왼쪽 차선, 빨(0), 주(1), 노(2), 초(3), 파(4), 남(5), 보(6)
        


        rearLines = self.processRear(rearImage) # 후면 카메라 이미지 처리

        reds, greens = frontObject # reds : n*3의 크기
        # reds: numpy array([[x1,y1,반지름], [x2,y2,반지름], ...])

        canny = self.canny(frontImage)  # canny image
        self.imshow('Canny', canny)   # canny image print
        
        cv2.imshow('Front Image Origin', laneImage) # lane image print

        if line is None:
            

            return self.vars.steer, self.vars.velocity
        if line[0] == 'right':
            line = frontLines[line[1]]    # 다시 line 재정의 
            x = line[:, 0]
            y = line[:, 1]
            coefficient = np.polyfit(y, x, 1)
            line = np.poly1d(coefficient)

            e = line(self.vars.line_detection_right) - center_x - self.vars.pixel_difference_right
            theImage = utlis.drawLines(laneImage, self.vars.line_detection_right, center_x, e, line, 'right')
        else:
            line = frontLines[line[1]]
            x = line[:, 0]
            y = line[:, 1]
            coefficient = np.polyfit(y, x, 1)
            line = np.poly1d(coefficient)
            e = line(self.vars.line_detection_left) - center_x + self.vars.pixel_difference_left
            theImage = utlis.drawLines(laneImage, self.vars.line_detection_left, center_x, e, line, 'left')

        steep = -coefficient[0]
        print("기울기 : ", steep)

        steer_seta = math.atan(steep)
        print("세타각 : ", steer_seta)

        #if steer_seta > 0 and steer_seta <= math.pi() / 2:
            


        '''
        if self.vars.status == '전진' :

            if e >= 100:
                self.vars.status = '우회전'
            if e <= -20:
                self.vars.status = '좌회전'

            if e >= -self.vars.steer_modify and e <= self.vars.steer_modify:
                steer = self.vars.steer_modify
            elif e < -self.vars.steer_modify:
                steer = utlis.steerFunction(0.3, e, self.vars.steer_modify, 1, True)
            elif e > self.vars.steer_modify:
                steer = utlis.steerFunction(0.3, e, self.vars.steer_modify, 1, False)

            print("e :", e)
            
        elif self.vars.status == '우회전':

            steer = 0.7*(e - 100) + 40.4

            print("e :", e)

            if e < 100:
                self.vars.status = '전진'

        elif self.vars.status == '좌회전':
            
            steer = 0.7*(e + 20) - 7.6

            print("e :", e)

            if e > -20:
                self.vars.status = '전진'
            

        elif self.vars.status == '정지':
            pass

        '''

        #self.vars.steer = utlis.showDirection(steer, self.vars.steer_modify)

        if steer > 80:
            steer = 80
        if steer < -80:
            steer = -80



        font=cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(theImage, "steer = " + str(int(steer)), (478,440), font, 0.7,(0,255,0),1)
        cv2.imshow('Front image', theImage)
        #print(self.vars.status)
        #print('속도 : ', velocity)
        print('스티어 : ', steer)

        self.vars.steer = steer


        
        return self.vars.steer, velocity


if __name__ == "__main__":
    g = Graphics(Planning) # 자주차 컨트롤러 실행
    g.root.mainloop() # 클릭 이벤트 처리
    g.exit() # 자주차 컨트롤러 종료