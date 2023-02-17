import time
from algorithmchul import *
from time import sleep
import glob
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


PRINTMSG = True

class JajuchaSimulation:

    def __init__(self): 
        
        # threshold for path planning
        self.threshold = 15
        self.onLeftCorner = False
        self.onRightCorner = False
        self.command = 'S0160E'
        self.status = ONSTRAIGHT
        self.light = True # False is red / True is green
        self.cnt = 0

    def imageProcessing(self, image, LiDAR):
        try:
            undist_image = cv2.undistort(image, mtx, dist, None, mtx)

            # set ROI
            height, width = undist_image.shape[:2]
            ROI_image = undist_image[120:240, :]

            #gray_img = grayscale(image)
            gray_img = cv2.cvtColor(ROI_image, cv2.COLOR_BGR2GRAY)
            blur_img = gaussian_blur(gray_img, 3)
            canny_img = canny(blur_img, 100, 500)

            # Hough transform
            vertices = np.array([[(50,height),(width/2-45, height/2+60), (width/2+45, height/2+60), (width-50,height)]], dtype=np.int32)
            hough_img, lines = hough_lines(canny_img, 1, 1 * np.pi/180, 55, 20, 10)

            # convert color gray to bgr in order to detect edge
            canny_img = cv2.cvtColor(canny_img, cv2.COLOR_GRAY2BGR)

            # get contact points
            points = getContactPoints(canny_img)

            # draw contact points

            canny_img = drawContactPoints(canny_img, points)
            
            # draw cross line
            canny_img = draw_crossLines(canny_img)

            # draw hough lines
            hough_img = weighted_img(hough_img, canny_img)

            # get drive command / No LiDAR data 
            self.command, self.status, self.light = autoDrive_algorithm(undist_image, canny_img, points, lines, LiDAR, self.command, self.status, self.light)

            cv2.imshow('undist video', undist_image)
            cv2.imshow('canny_img', canny_img)
            cv2.imshow('hough_img', hough_img)
            fig = plt.figure()
            ax = Axes3D(fig)
            X = np.arange(0,320,1)
            Y = np.arange(0,120,1)
            X, Y = np.meshgrid(X,Y)

            ax.plot_surface(X,Y,0, rstride = 1, cstride = 1, cmap = 'hot')
            plt.show()
            

            # Emergency Stop
            
            key = cv2.waitKey(100000)    
            if key == ord('q'):
                print("Quit")
                self.command = 'S0150E'
                cv2.destroyAllWindows()
                return False, self.command
        
        except Exception as e:
            print(e)

        return True, self.command

    def runSelfDriving(self):
        data_set = glob.glob("simulation/*.jpg")
        cnt = 0
        for filename in data_set:
            image = cv2.imread(filename)
            status, command = self.imageProcessing(image, 0)
            print('%d frame: %s' % (cnt, command))
            cnt+=1
            if status == False:
                break  

if __name__ == '__main__':
    JAJUCHA = JajuchaSimulation()
    JAJUCHA.runSelfDriving()



