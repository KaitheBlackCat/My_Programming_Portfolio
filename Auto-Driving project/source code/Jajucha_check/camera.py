import socket
import struct
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from jajuchaUtil import *

class CameraTest:
    def __init__(self):
        
        # threshold for path planning
        self.command = 'S0150E'
       
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect(server_address)
        print('Connecting to the server...')
        self.sendComm = self.client_socket
        self.connection = self.client_socket.makefile('rb')

    def runCameraTest(self):        
        try:
            print('Streaming...')
            print("Press 'q' to exit")

            while True:
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
                cv2.imshow('Streaming Image', image)

                key = cv2.waitKey(1)    
                if key == ord('q'):
                    cv2.destroyAllWindows()
                    break
                    
                command = getAlignedWheelAngle('S0150E', WHEELALIGN)
                self.sendComm.send(self.command.encode())

        except Exception as e:
            print(e)
            
        finally:
            print('Closing the connection.')
            self.connection.close()
            self.client_socket.close()

if __name__ == '__main__':
    JAJUCHA = CameraTest()
    JAJUCHA.runCameraTest()