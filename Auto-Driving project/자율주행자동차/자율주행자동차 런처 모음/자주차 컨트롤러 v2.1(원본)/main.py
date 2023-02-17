from jajucha.planning import BasePlanning
from jajucha.graphics import Graphics
from jajucha.control import mtx
import cv2
import numpy as np
import time

class Planning(BasePlanning):
    def __init__(self, graphics):
        super().__init__(graphics)
        # --------------------------- #
        self.vars.counter = 0  # 변수 설정

    def process(self, t, frontImage, rearImage, frontLidar, rearLidar):
        laneImage = frontImage.copy()
        """
        자주차의 센서 정보를 바탕으로 조향과 속도를 결정하는 함수
        t: 주행 시점으로부터의 시간 (초)
		frontImage: 전면 카메라 이미지 480*640*3 [y,x,c] (x,y)좌표의 color(0~255) 정보. 카메라 640*480 이미지를 캘리브레이션 한 것.
		rearImage: 후면 카메라 이미지
		frontLidar: 전면 거리 센서 (mm), 0은 오류를 의미함
		rearLidar: 후면 거리 센서 (mm), 0은 오류를 의미함
        """
        frontLines, frontObject = self.processFront(frontImage) # 전면 카메라 이미지 처리
        # frontLines = [[x1,y1],[x2,y2],[x3,y3], … ] , [[x1,y1], [x2,y2], … ]
        # y1>y2>y3... : 밑의 점부터 표현
        # frontLines[0]이 가장 왼쪽 차선, 빨(0), 주(1), 노(2), 초(3), 파(4), 남(5), 보(6)
        rearLines = self.processRear(rearImage) # 후면 카메라 이미지 처리

        reds, greens = frontObject # reds : n*3의 크기
        # reds: numpy array([[x1,y1,반지름], [x2,y2,반지름], ...])

        canny = self.canny(frontImage)  # canny image
        self.imshow('Canny', canny)   # canny image print
        
        cv2.imshow('Front Image', laneImage) # lane image print

        for i in range(0, 80):
        	self.vars.counter += 1
        	print(self.vars.counter)


        for i in range(0, 80):
        	self.vars.counter -= 1
        	print(self.vars.counter)


        steer = 0  # 조향 권장 -80~80 (범위 -100~100)
        velocity = 40 # 속도 권장 -50~50 (범위 -330~300)
        return steer, velocity


if __name__ == "__main__":
    g = Graphics(Planning) # 자주차 컨트롤러 실행
    g.root.mainloop() # 클릭 이벤트 처리
    g.exit() # 자주차 컨트롤러 종료
