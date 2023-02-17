import socket
import struct
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from jajuchaUtil import *

class DCMotorTest:
    def __init__(self):
        
        # threshold for path planning
        self.command = 'S0150E'
       
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect(server_address)
        print('Connecting to the server...')
        self.sendComm = self.client_socket
        self.connection = self.client_socket.makefile('rb')

    def runDCMotorTest(self):        
        try:
            print('DC Motor Test')
            cnt = 0

            while True:
                # Obtain the length of the frame streamed over the connection. If image_len = 0, close the
                # connectionw
                image_len = struct.unpack('<L', self.connection.read(struct.calcsize('<L')))[0]
                if not image_len:
                    break

                # Store bytes in a string
                recv_bytes = b''
                recv_bytes += self.connection.read(image_len+4)
                
                if cnt < 10:
                    command = getAlignedWheelAngle('S1150E', WHEELALIGN)
                    self.sendComm.send(command.encode())
                elif cnt > 18:
                    command = getAlignedWheelAngle('S0150E', WHEELALIGN)
                    self.sendComm.send(command.encode())
                    break
                elif cnt >= 10:
                    command = getAlignedWheelAngle('S2150E', WHEELALIGN)
                    self.sendComm.send(command.encode())
                cnt += 1

        except Exception as e:
            print(e)
            
        finally:
            print('Closing the connection.')
            self.connection.close()
            self.client_socket.close()

if __name__ == '__main__':
    JAJUCHA = DCMotorTest()
    JAJUCHA.runDCMotorTest()