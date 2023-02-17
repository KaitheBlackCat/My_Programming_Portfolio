from jajucha.planning import BasePlanning
from jajucha.graphics import Graphics
from jajucha.control import mtx
import cv2
import numpy as np
import time
import utlis2 as utlis



class Planning(BasePlanning):
    def __init__(self, graphics):
        super().__init__(graphics)
        # --------------------------- #
        self.vars.status = '전진'
        self.vars.line_detection_right = 300
        self.vars.line_detection_left = 325
        self.vars.pixel_difference_right = 140
        self.vars.pixel_difference_left = 210
        self.vars.first_line_choose = 400
        self.vars.steer = 0  # 조향값은 0인 상태로 변수 설정
        self.vars.velocity = 50  # 속도도 0인 상태로 변수 설정
        self.vars.steer_modify = 8
        self.vars.steerKorean = ''
        self.vars.stop = False
        self.vars.redCnt = 0
        self.vars.e = 0
        self.vars.noLineCount = 0
        self.vars.mainLine = ''
        self.vars.frame = 0


    def process(self, t, frontImage, rearImage, frontLidar, rearLidar):

        self.vars.frame = self.vars.frame + 1

        img = np.zeros((140,640,3), np.uint8)

        font=cv2.FONT_HERSHEY_SIMPLEX

        velocity = 0

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

        if not self.vars.stop :         # 차가 운동중일 때  
            if not reds :      # 운행중 빨강불이므로 멈춤!
                print(reds)
                self.vars.redCnt = self.vars.redCnt + 1
                if self.vars.redCnt > 10 :
                    self.vars.redCnt = 10
                #print('( STOP )', ' Red : ',self.vars.redCnt)
                self.vars.status = '정지'

            else :
                self.vars.redCnt = self.vars.redCnt - 1
                if self.vars.redCnt < 0 :
                    self.vars.redCnt = 0
                #print('( GO )', ' Red : ',self.vars.redCnt)
                self.vars.status = '전진'

            print("redCnt : ", self.vars.redCnt)
                

        if self.vars.stop :             # 차가 멈추어 있을 때 
            if reds :
                print(reds)
                self.vars.redCnt = self.vars.redCnt + 2
                if self.vars.redCnt > 10 :
                    self.vars.redCnt = 10
                #print('( STOP )', ' Red : ',self.vars.redCnt)
                #return self.vars.steer, 0
                self.vars.status = '정지'
            else :
                self.vars.redCnt = self.vars.redCnt - 1
                if self.vars.redCnt < 0 :
                    self.vars.redCnt = 0
                #print('( GO )', ' Red : ',self.vars.redCnt)
                #return self.vars.steer, 50
                self.vars.status = '전진'
            #print("redCnt : ", redCnt)
        

        if self.vars.redCnt > 3 or 0 < frontLidar < 300 :
            self.vars.stop = True
            print('( 정지! )', ' Red : ',self.vars.redCnt)
            self.vars.status = '정지'
            print('속도 : ', 0)
            self.vars.velocity = 0
              # 신호등때문에 멈추었다고 표시하고
        else :
            print('( 출발! )', ' Red : ',self.vars.redCnt)
            self.vars.status = '전진'
            self.vars.velocity = 50

        for i in range(len(frontLines)) :
            if frontLines[i][0, 1] < 340 : # (1) [No 460]은 적절한 값이 아닙니다!! / y1, y2, y3... [No 460]보다 작은가? 그러니까, 차선이 나타는 부분까지만 이미지를 자르자고 하는거야. 어디까지 잘라야 하는걸까?
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

        print("front Lidar :", frontLidar)

        '''
        if 0 < frontLidar < 300:
            self.vars.status = '정지'
            #theImage = cv2.putText(laneImage, "Lidar STOP", (int(center_x-35), 20), font, 1, (255,0,0), 2)
            print('속도 : ', 0)
            self.vars.velocity = 0
        else:
            self.vars.status = '전진'
            self.vars.velocity = 50
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
        #self.imshow('Canny', canny)   # canny image print
        
        #cv2.imshow('Front Image Origin', laneImage) # lane image print

        if self.vars.status == '정지':
            print("정지합니다!")
            if 0 < frontLidar < 300:
                theImage = cv2.putText(laneImage, "Lidar STOP", (int(center_x-70), 450), font, 1, (255,0,0), 2)
            if self.vars.redCnt > 3:
                theImage = cv2.putText(laneImage, "T.F. STOP", (int(center_x-60), 470), font, 1, (255,0,0), 2)
                        
            self.vars.velocity = 0
        else:
            print("정지하지 않습니다!")
            self.vars.velocity = 50

            if line is None:
                print("선이 인식되지 않습니다!")
                self.vars.noLineCount += 1
                print("인식불가횟수: ", self.vars.noLineCount)

                img = utlis.showValues(img, self.vars.e, center_x, self.vars.velocity, self.vars.steer, self.vars.steerKorean, self.vars.status, self.vars.frame, self.vars.redCnt, self.vars.noLineCount)
                cv2.imshow('Instrument panel', img)


                if self.vars.noLineCount > 3:
                    print("선 인식이 안된지 총 네번이 지났습니다. 민서 알고리즘 실행")
                    if self.vars.steer > 0:
                        self.vars.steer = 80
                    if self.vars.steer < 0:
                        self.vars.steer = -80
                
                self.vars.steer, self.vars.status_a = utlis.showDirection(self.vars.steer)
                print("스티어 :", self.vars.steer)
                print("속도(self):", self.vars.velocity)

                return self.vars.steer, self.vars.velocity
            if line[0] == 'right':
                self.vars.noLineCount = 0
                print("주차선이 오른쪽입니다!")
                line = frontLines[line[1]]    # 다시 line 재정의 
                x = line[:, 0]
                y = line[:, 1]
                coefficient = np.polyfit(y, x, 1)
                line = np.poly1d(coefficient)

                self.vars.e = line(self.vars.line_detection_right) - center_x - self.vars.pixel_difference_right
                theImage = utlis.drawLines(laneImage, self.vars.line_detection_right, center_x, line, 'right')

            else:
                self.vars.noLineCount = 0
                print("주차선이 왼쪽입니다!")
                line = frontLines[line[1]]
                x = line[:, 0]
                y = line[:, 1]
                coefficient = np.polyfit(y, x, 1)
                line = np.poly1d(coefficient)
                self.vars.e = line(self.vars.line_detection_left) - center_x + self.vars.pixel_difference_left
                theImage = utlis.drawLines(laneImage, self.vars.line_detection_left, center_x, line, 'left')

            if self.vars.status == '전진' :
                print("전진 상황!")

                if self.vars.e >= 100:
                    self.vars.status = '우회전'
                    print("급격한 변화! 우회전 모드 실행")
                if self.vars.e <= -30:
                    self.vars.status = '좌회전'
                    print("급격한 변화! 좌회전 모드 실행")

                if self.vars.e >= -self.vars.steer_modify and self.vars.e <= self.vars.steer_modify:
                    steer = self.vars.steer_modify
                elif self.vars.e < -self.vars.steer_modify:
                    steer = utlis.steerFunction(0.3, self.vars.e, self.vars.steer_modify, 1, True)
                elif self.vars.e > self.vars.steer_modify:
                    steer = utlis.steerFunction(0.3, self.vars.e, self.vars.steer_modify, 1, False)

                #print("e :", e)
                
            elif self.vars.status == '우회전':
                

                steer = 1.2*(self.vars.e - 100) + 40.4

                #print("e :", e)

                if self.vars.e < 100:
                    self.vars.status = '전진'
                    print("우회전에서 다시 전진으로")

            elif self.vars.status == '좌회전':

                
                steer = 1.2*(self.vars.e + 30) + 1.4

                #print("e :", e)

                if self.vars.e > -30:
                    self.vars.status = '전진'
                    print("좌회전에서 다시 전진으로")
            

        #steer = 0  # 조향 권장 -80~80 (범위 -100~100)
        #velocity = 50 # 속도 권장 -50~50 (범위 -330~300)

        self.vars.steer, self.vars.steerKorean = utlis.showDirection(steer)

        

        img = utlis.showValues(img, self.vars.e, center_x, self.vars.velocity, self.vars.steer, self.vars.steerKorean, self.vars.status, self.vars.frame, self.vars.redCnt)

        cv2.imshow('Front image', theImage)
        cv2.imshow('Instrument panel', img)
        print("스타투스 : ", self.vars.status)
        print('속도 : ', self.vars.velocity)
        print('스티어 : ', self.vars.steer)


        return self.vars.steer, self.vars.velocity


if __name__ == "__main__":
    g = Graphics(Planning) # 자주차 컨트롤러 실행
    g.root.mainloop() # 클릭 이벤트 처리
    g.exit() # 자주차 컨트롤러 종료