import cv2
import numpy as np
import time
import socket
import struct
from time import sleep
from algorithmty1220 import *
import timeit

PRINTMSG = True
# PRINTMSG = False

class VideoStreaming:
    def __init__(self):
        
        # threshold for path planning
        self.threshold = 15 
        self.onLeftCorner = False
        self.onRightCorner = False
        self.command = 'S0150E'
        self.status = ONSTRAIGHT
        self.light = True # False is red / True is green
        self.cnt = 0
        
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect(server_address)
        print('Connecting to the server...')
        self.sendComm = self.client_socket
        self.connection = self.client_socket.makefile('rb')

    def imageProcessing(self, image, LiDAR):
        try:
            undist_image = cv2.undistort(image, mtx, dist, None, mtx)

            # set ROI
            height, width = undist_image.shape[:2]
            ROI_image = undist_image[120:240, :]

            #gray_img = grayscale(image)
            gray_img = cv2.cvtColor(ROI_image, cv2.COLOR_BGR2GRAY)
            blur_img = gaussian_blur(gray_img, 3)
            canny_img = canny(blur_img, 100, 400)
            # canny_img = canny(blur_img, 100, 400)

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

            # get drive command
            self.command, self.status, self.light = autoDrive_algorithm(undist_image, canny_img, points, lines, LiDAR, self.command, self.status, self.light)

            cv2.imshow('canny_img', canny_img)
            cv2.imshow('hough_img', hough_img)

        
        except Exception as e:
            print(e)
        finally:
            cv2.imshow('undist video', undist_image)

            # Emergency Stop
            key = cv2.waitKey(1)    
            if key == ord('q'):
                print("Quit")
                self.command = 'S0150E'
                return False, self.command
        
        return True, self.command

    def runSelfDriving(self):        
        try:
            print('Streaming...')
            print("Press 'q' to exit")

            while True:
                start = timeit.default_timer()
                # Obtain the length of the frame streamed over the connection. If image_len = 0, close the
                # connectionw
                image_len = struct.unpack('<L', self.connection.read(struct.calcsize('<L')))[0]
                if not image_len:
                    break

                # Store bytes in a string
                recv_bytes = b''
                recv_bytes += self.connection.read(image_len+4)

                # Read an image from buffer in memory
                image = cv2.imdecode(np.fromstring(recv_bytes[:-4], dtype=np.uint8), cv2.IMREAD_COLOR)
                
                # Read LiDAR data from buffer in memory
                LiDAR = int(recv_bytes[-4:].decode())
                
                status, command = self.imageProcessing(image, LiDAR)
                if PRINTMSG: print(command)
                #command = 'S1150E'
                self.sendComm.send(getAlignedWheelAngle(command, WHEELALIGN).encode())

                stop = timeit.default_timer()
                # print('Communication latency: %fms' % ((stop - start)*1000))
                if (stop - start)*1000 > 1000:
                    if PRINTMSG:
                        print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Network High Latency !!!!!!')

                if status == False:
                    break

        except Exception as e:
            print(e)
            
        finally:
            print('Closing the connection.')
            self.connection.close()
            self.client_socket.close()

if __name__ == '__main__':
    JAJUCHA = VideoStreaming()
    JAJUCHA.runSelfDriving()
