import socket
import struct
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from jajuchaUtil import *

class ServoTest:
    def __init__(self):
        
        # threshold for path planning
        self.command = 'S0150E'
       
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect(server_address)
        print('Connecting to the server...')
        self.sendComm = self.client_socket
        self.connection = self.client_socket.makefile('rb')

    def runServoTest(self):        
        try:
            print('Servo Motor Test')
            cnt = 150
            phase = 1

            while True:
                # Obtain the length of the frame streamed over the connection. If image_len = 0, close the
                # connectionw
                image_len = struct.unpack('<L', self.connection.read(struct.calcsize('<L')))[0]
                if not image_len:
                    break

                # Store bytes in a string
                recv_bytes = b''
                recv_bytes += self.connection.read(image_len+4)
                
                if phase == 1:
                    command = 'S3%dE' % cnt
                    command = getAlignedWheelAngle(command, WHEELALIGN)
                    self.sendComm.send(command.encode())
                    cnt += 5
                    if cnt > 180:
                        phase = 2
                elif phase == 2:
                    command = 'S3%dE' % cnt
                    command = getAlignedWheelAngle(command, WHEELALIGN)
                    self.sendComm.send(command.encode())
                    cnt -= 5
                    if cnt < 120:
                        phase = 3
                elif phase == 3:
                    command = 'S3%dE' % cnt
                    command = getAlignedWheelAngle(command, WHEELALIGN)
                    self.sendComm.send(command.encode())
                    cnt += 5
                    if cnt > 150:
                        command = getAlignedWheelAngle('S0150E', WHEELALIGN)
                        self.sendComm.send(command.encode())
                        break
		    

        except Exception as e:
            print(e)
            
        finally:
            print('Closing the connection.')
            self.connection.close()
            self.client_socket.close()

if __name__ == '__main__':
    JAJUCHA = ServoTest()
    JAJUCHA.runServoTest()