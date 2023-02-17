import cv2
import numpy as np
import socket
import struct

server_address = ('169.254.22.198', 45713)

class VideoStreamingTest:
    def __init__(self):
        
        # threshold for path planning
        self.command = 'S0160E'
       
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect(server_address)
        print('Connecting to the server...')
        self.sendComm = self.client_socket
        self.connection = self.client_socket.makefile('rb')

        self.runAutoCar()

    def runAutoCar(self):        
        try:
            print('Streaming...')
            print("Press 'q' to exit")
			
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

                # Read an image from buffer in memory
                image = cv2.imdecode(np.fromstring(recv_bytes[:-4], dtype=np.uint8), cv2.IMREAD_COLOR)
                cv2.imshow('Recording..', image)
                filename = 'savedImage/%04d.jpg' % cnt
                cnt += 1
                cv2.imwrite(filename, image)
                
                key = cv2.waitKey(1)
                if key == ord('q'):
                    cv2.destroyAllWindows()
                    break
                    
                self.sendComm.send(self.command.encode())

        except Exception as e:
            print(e)
            
        finally:
            print('Closing the connection.')
            self.connection.close()
            self.client_socket.close()

if __name__ == '__main__':
    VideoStreamingTest()
