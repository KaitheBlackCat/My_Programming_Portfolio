import socket
import struct
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from jajuchaUtil import *

class LiDARTest:
    def __init__(self):
        
        # threshold for path planning
        self.command = 'S0150E'
       
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect(server_address)
        print('Connecting to the server...')
        self.sendComm = self.client_socket
        self.connection = self.client_socket.makefile('rb')

    def runLiDARTest(self):        
        try:
            print('LiDAR Test')
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

                # Read LiDAR data from buffer in memory
                LiDAR = int(recv_bytes[-4:].decode())
                if LiDAR == 0:
                    print('No LiDAR Sensor detected.')
                else:
                    print('LiDAR data: %d mm' % LiDAR)
                
                command = getAlignedWheelAngle('S0150E', WHEELALIGN)
                self.sendComm.send(command.encode())

        except Exception as e:
            print(e)
            
        finally:
            print('Closing the connection.')
            self.connection.close()
            self.client_socket.close()

if __name__ == '__main__':
    JAJUCHA = LiDARTest()
    JAJUCHA.runLiDARTest()