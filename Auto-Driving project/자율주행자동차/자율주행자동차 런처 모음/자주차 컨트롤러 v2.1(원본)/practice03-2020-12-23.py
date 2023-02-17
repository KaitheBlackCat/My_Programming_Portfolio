from jajucha.planning import BasePlanning
from jajucha.graphics import Graphics
from jajucha.control import mtx
import cv2
import numpy as np
import time

''' ===============================
이 코딩은 다음과 같은 특징을 가졌습니다. 코딩 완성의 우선순위는 다음과 같습니다.
참고로 이 코딩이 정답은 아닙니다!!! 여러분은 더 좋은 코딩을 만들 수 있습니다.
여기서 제시하는 코딩을 배우면서 더 적합한 알고리즘을 만드는 것이 목표입니다!!!

(1) "차선 선택"에서
    예) if frontLines[i][0, 1] < 460: # [No 460]은 적절한 값이 아닙니다!!
    이 의미는 460이란 값이 적절한 값이 아님을 의미합니다. 이때 표시를 [No 460]이라고 하겠습니다.
    즉 [No 460]이라는 뜻은 그 행의 460이라는 값을 적절한 값으로 바꾸어 넣으라는 의미입니다.
    (1) 460으로 등장하는 [No 460]은 적절한 값으로 고쳐주세요.
        이 값을 적절한 값으로 찾으면 추세선의 점선이 보입니다.
    (2) 450으로 등장하는 [No 450]은 적절한 값으로 고쳐주세요.
        이 값은 가장 잘 등장하는 값으로 중앙에서 치우친 정도를 파악합니다.
    (3) 100으로 등장하는 [No 100]은 적절한 값으로 고쳐주세요.
        우차선으로 기준으로 중앙값에 해당하는 픽셀차이는 얼마일까요?
    (4) 110으로 등장하는 [No 110]은 적절한 값으로 고쳐주세요.
        좌차선을 기준으로 중앙값에 해당하는 픽셀차이는 얼마일까요?
    (5) 2으로 등장하는 [No 2]은 적절한 값으로 고쳐주세요.
        픽셀차이와 조향값 차이를 해결하는 1차식의 기울기에 해당하는 값은?
    (6) 3으로 등장하는 [No 3]은 적절한 값으로 고쳐주세요.
        픽셀차이와 조향값 차이를 해결하는 1차식의 상수항에 해당하는 값은?
    (7) 10으로 등장하는 [No 10]은 적절한 값으로 고쳐주세요.
        자주차 속력의 적절한 값은? 상수가 좋을까? 변수가 좋을까?

(2) "라이다 처리" 는 직접 여러분들이 코딩을 완성해야 합니다.

(3) "신호등 처리?"라고 표현된 곳은
    i) [No 1]은 적절한 값이 아닙니다! => 적절한 값 찾아야 합니다.
    ii) 차가 멈추어 있을 때의 코딩이 아직 없습니다. => 적절한 코딩을 완성해야 합니다.


'''

class Planning(BasePlanning):
    def __init__(self, graphics):
        super().__init__(graphics)
        # --------------------------- # 초기 변수를 설정하는 함수
        self.vars.redCnt = 0  # 빨강불 카운트 변수 설정
        self.vars.greenCnt = 0  # 녹색불 카운트 변수 설정
        self.vars.stop = True # 차가 처음에 정지해 있음을 정의한 변수 설정
        self.vars.steer = 0  # 조향값은 0인 상태로 변수 설정
        self.vars.velocity = 0  # 속도도 0인 상태로 변수 설정

        self.vars.steer_modify = -8 # 자주차가 직진하는 값으로, 서보모터의 상태에 따라 자주차의 steer 값이 0 임에도 직진하지 않는 경우가 있다.
                                   # 따라서, 이를 조정하여 자주차가 직진하도록 만들어야 한다. 양수는 오른쪽, 음수는 왼쪽으로 가게 된다.

        self.vars.velocity_max = 50 # 앞으로 주행할 시의 속도 최댓값. 50 이상을 넘기지 않도록 하자.

        self.vars.image_line_detect_x_value_right = 400 # 이미지가 인식할 추세선 위 파란 점의 위치를 정하는 값. 값이 작을 수록 이미지 상단의 파란 점으로 설정된다.
        self.vars.image_line_detect_x_value_final = 400 # 왼쪽으로 기준선을 정할지, 오른쪽으로 정할지 결정하기 위한 기준점으로,
                                                        # 가장 아래쪽에 위치한 직선을 기준으로 왼쪽선을, 혹은 오른쪽선을 기준으로 잡을지가 결정된다.
        self.vars.image_line_detect_x_value_left = 320  # 왼쪽으로 기준선이 결정되었을 때 왼쪽 추세선 위의 어느 점을 고를지 결정. 300이상을 추천함
        self.vars.line_and_middle_line_pixel_difference_right = 260 # 각 추세선 위의 파란점과 중앙으로부터의 픽셀 차이. 이 값을 조정하여 차량의 위치를 조정한다.
                                                                    # 값이 작을수록 차량이 인식하는 차선(오른쪽, 왼쪽)에 가까이 붙으며, 값이 클수록 인식하는 차선으로부터 멀어진다. 
        self.vars.line_and_middle_line_pixel_difference_left = 220  # 위와 동일

        self.vars.tilt = 0.001                                      # steer 값을 조정하는 함수의 최고차항의 계수

    def process(self, t, frontImage, rearImage, frontLidar, rearLidar):
        """
        자주차의 센서 정보를 바탕으로 조향과 속도를 결정하는 함수
        t: 주행 시점으로부터의 시간 (초)
        frontImage: 전면 카메라 캘리된 이미지(640X480)
        rearImage: 후면 카메라 캘리된 이미지
        frontLidar: 전면 거리 센서 (mm), 0은 오류를 의미함, 0<x<2000 (2m 최대거리)
        rearLidar: 후면 거리 센서 (mm), 0은 오류를 의미함
        """
        frontLines, frontObject = self.processFront(frontImage)  # 전면 카메라 이미지 처리
        # frontLines = [[x1,y1],[x2,y2],[x3,y3], … ] , [[x1,y1], [x2,y2], … ]
        # frontLines[0]이 가장 왼쪽 차선, 빨(0), 주(1), 노(2), 초(3), 파(4), 남(5), 보(6)
        rearLines= self.processRear(rearImage) # 후면 카메라 이미지 처리

        # 신호등 처리
        reds, greens = frontObject # reds : n*3의 크기
        # reds: numpy array([[x1,y1,반지름], [x2,y2,반지름], ...])

        # canny image 출력 
        canny = self.canny(frontImage)
        self.imshow('Canny Image', canny)

        # 분석을 위한 y값 설정 
        x = 320
        y = 479
        while y >= 0:
            if canny[y, x] > 0:
                break
            y -= 1
        # print('479-y=', 479-y)

        '''
        #신호등 처리?
        if not self.vars.stop:         # 차가 운동중이면 
            if reds:                     # 빨간불이면 
                self.vars.redCnt += 1      # 빨간불 카운트 +1
            else:                        # 빨간물 아니면
                self.vars.redCnt = 0       # 다시 빨강 카운트는 0
            if self.vars.redCnt >= [No 1]:    # [No 1]은 적절한 값이 아닙니다!! 만약 카운트가 [No 1] 보다 크거나 같으면 빨강불로 인식하겠다는 의미입니다!
                self.vars.greenCnt = 0     # 빨강불이면 녹색불 카운트는 0, 
                self.vars.stop = True      # 운행중 빨강불이므로 멈춤!

        if self.vars.stop:             # 차가 멈추어 있을 때 어떻게 할까? (언제 움직이지?)


        if self.vars.stop:             # 차가 멈추어 있다면
            print('Light stop!!!', 'Red Cnt=',self.vars.redCnt, 'Green Cnt=',self.vars.greenCnt  )  # 신호등때문에 멈추었다고 표시하고
            return 0, 0                  # 조향 0, 속도 0 함수값 리턴!
        '''
        '''
        # 라이다 처리
        if 
        '''
        # 차선 선택
        center_x = mtx[0, 2]       # 이미지의 가운데, center_x=320 부근의 값을 가지게 됩니다.
        # print ('center_x=', center_x)
        line = None
        frontLines.sort(key=lambda x:x[0, 1], reverse=True)
        # frontLines를 정렬(x[0,1]을 기준으로 내림차순으로 정렬)
        # frontLines = [[x1,y1],[x2,y2],[x3,y3], … ] , [[x1,y1], [x2,y2], … ]
        # y1>y2>y3... : 밑의 점부터 표현
        # print ('frontLines=', frontLines)
        for i in range(len(frontLines)) :
            if frontLines[i][0, 1] < 340 : # (1) [No 460]은 적절한 값이 아닙니다!! / y1, y2, y3... [No 460]보다 작은가? 그러니까, 차선이 나타는 부분까지만 이미지를 자르자고 하는거야. 어디까지 잘라야 하는걸까?
                continue              # y값이 [No 460]보다 작으면 추세선 안하고 넘어가라(479~[No 460]까지만 추세선 찾기)
            x = frontLines[i][:, 0]    # x=[x1, x2, x3, ...]
            y = frontLines[i][:, 1]    # y=[y1, y2, y3, ...]
            coefficient = np.polyfit(y, x, 1)    # coefficient = [a, b] (단, x = ay + b) 
            line = np.poly1d(coefficient)        # line = ay + b (가장 밑에 닿은 차선의 추세선 식)
            if line(self.vars.image_line_detect_x_value_final) < center_x:   # (2) [No 450]은 적절한 값이 아닙니다!! / 
                      # 만약 line = a*[No 450] + b : y=[No 450]에서의 x값이 중앙보다 왼쪽이면
                line = 'left', i            # line = ('left', i) 인 class, tuple
                break
            else:                           # 만약 line = a*400 + b : y=400에서의 x값이 중앙보다 왼쪽이 아니면
                line = 'right', i           # line = ('right', i) 인 class, tuple
                break
        # print('line', line)
        laneImage = frontImage.copy()
        if line is None:                    # line 이 없으면
            # No line
            print('차선이 감지되지 않습니다! 차량이 차선 외로 넘어갔을 것입니다.')
            return self.vars.steer, self.vars.velocity
        if line[0] == 'right':              # line 이 우차선이면
            print('오른쪽 차선 기준')
            # follow right
            # left = frontLines[line[1]-1]
            line = frontLines[line[1]]    # 다시 line 재정의 
            x = line[:, 0]
            y = line[:, 1]
            coefficient = np.polyfit(y, x, 1)
            line = np.poly1d(coefficient)
            print('직선 방정식 : ', line)
            e = line(self.vars.image_line_detect_x_value_right) - center_x - self.vars.line_and_middle_line_pixel_difference_right  # (2) [No 450]과 (3) [No 100]은 적절한 값이 아닙니다!!
            # 추세선 line = a*[No 450] + b 에서 line([No 450])은 추세선에서 y=[No 450]일 때의 x의 값을 의미하며
            # line([No 450])값이 중앙값(center_x)의 값보다 몇 픽셀 커야 자주차는 중앙에 위치한 것일까요?
            # 이 몇 픽셀이 바로 [No 100]에 해당하며 적절한 값을 찾아주세요.
            print('직선의 방정식 y값(', self.vars.image_line_detect_x_value_right, ') 기준 x축 값 : ', line(self.vars.image_line_detect_x_value_right)) # (2) [No 450]의 적절한 값을 넣어주세요.
            print('이미지 가운데 좌표 :', center_x, 'x축 값과 가운데 좌표간 픽셀 차이 : ', e)
            for i in range(200, 480, 20):
                cv2.circle(laneImage, (int(line(i)), i), 5, (255, 0, 0), -1)
            # cv2.circle(frontImage, (int(f(400)), 400), 5, (255, 0, 0), -1)
            cv2.imshow('Front Lane Image', laneImage)
        else:
            print('왼쪽 차선 기준')
            # follow left
            line = frontLines[line[1]]
            x = line[:, 0]
            y = line[:, 1]
            coefficient = np.polyfit(y, x, 1)
            line = np.poly1d(coefficient)
            print('직선의 방정식 : ', line)
            e = line(self.vars.image_line_detect_x_value_left) - center_x + self.vars.line_and_middle_line_pixel_difference_left  # (2) [No 450]과 (4) [No 110]은 적절한 값이 아닙니다!!
            # 추세선 line = a*[No 450] + b 에서 line([No 450])은 추세선에서 y=[No 450]일 때의 x의 값을 의미하며
            # line([No 450])값이 중앙값(center_x)의 값보다 몇 픽셀 작아야 자주차는 중앙에 위치한 것일까요?
            # 이 몇 픽셀이 바로 [No 110]에 해당하며 적절한 값을 찾아주세요.
            print('직선의 방정식 y값(', self.vars.image_line_detect_x_value_left, ') 기준 x축 값', line(self.vars.image_line_detect_x_value_left)) # (2) [No 450]의 적절한 값을 넣어주세요.
            print('이미지 가운데 좌표 :', center_x, 'x축 값과 가운데 좌표간 픽셀 차이 :', e)
            for i in range(200, 480, 20):
                cv2.circle(laneImage, (int(line(i)), i), 5, (255, 0, 0), -1)
            # cv2.circle(frontImage, (int(f(400)), 400), 5, (255, 0, 0), -1)
            cv2.imshow('Front Lane Image', laneImage)

        if e >= -5 and e <= 5:
            steer = self.vars.steer_modify
        elif e < -5:
            steer = self.vars.tilt * ((e + 5) ** 3) + self.vars.steer_modify
        elif e > 5:
            steer = self.vars.tilt * ((e - 5) ** 3) + self.vars.steer_modify

        #steer = self.vars.tilt * e + self.vars.steer_modify # (5) [No 2]와 (6) [No 3]은 적절한 값이 아닙니다. steer는 조향값이며, e는 픽셀의 차이입니다.
        self.vars.steer = steer
        print('steer : ', steer)

        if steer < -60:
            print("조향 : 급격한 좌회전")
        elif steer >= -60 and steer < -40:
            print("조향 : 거센 좌회전")
        elif steer >= -40 and steer < -20:
            print("조향 : 조금 거센 좌회전")
        elif steer >= -20 and steer <-10 :
            print("조향 : 약한 좌회전")
        elif steer <= 20 and steer > 10:
            print("조향 : 약한 우회전")
        elif steer <= 40 and steer > 20:
            print("조향 : 조금 거센 우회전")
        elif steer <= 60 and steer > 40:
            print("조향 : 거센 우회전")
        elif steer > 60:
            print("조향 : 급격한 우회전")
        else:
            print("조향 : 직진")

        if self.vars.steer == 0:
            self.vars.steer = self.vars.steer_modify
        elif self.vars.steer > 80:
            self.vars.steer = 80
        elif self.vars.steer < -80:
            self.vars.steer = -80

        print("self.vars.steer 값 : ", self.vars.steer)

        # 픽셀과 조향값과는 어떤 관계가 있을까요? 이 관계를 가장 적절한 1차식으로 만들어 봅시다.
        # 참고로 조향값 steer는 -80~80 을 추천합니다. [-100~100] 사이값은 이론적인 값이며 실제 이 값을 넣으면
        # 기계에 무리가 옵니다. 절대 넣지 않기를 조언합니다.
        velocity = (-1 * self.vars.velocity_max / (320 * 320)) * ((e ** 2) - 320 * 320)     # [No 10]은 적절한 값이 아닐 수 있습니다. velocity는 속도입니다.

        print("속도 :", velocity)

        # -50 ~ 50의 값을 추천합니다. [-300 ~ 300] 사이값은 이론적인 값이며
        # 실제 이 값을 넣으면 기계에 무리가 옵니다. 절대 넣지 않기를 조언합니다.
        # 이 velocity를 e값의 1차 함수로 넣을 수도 있습니다.

        # cv2.waitKey(1)
        # self.vars.steer = steer
        self.vars.velocity = velocity
        return self.vars.steer, self.vars.velocity


if __name__ == "__main__":
    g = Graphics(Planning) # 자주차 컨트롤러 실행
    g.root.mainloop() # 클릭 이벤트 처리
    g.exit() # 자주차 컨트롤러 종료