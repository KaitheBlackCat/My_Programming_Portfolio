import io
import time
import picamera
import struct
import socket
import pigpio
import binascii
import smbus

server_address = ('169.254.11.161', 45713)

camera = picamera.PiCamera()
camera.resolution = (320, 240)
camera.framerate = 10
time.sleep(2)           # camera warmup

start = time.time()
stream = io.BytesIO()

###############################################################

def vehicle_backward():
    pi.write(24, 0)
    pi.write(23, 1)
    
def vehicle_forward():
    pi.write(24, 1)
    pi.write(23, 0)

def vehicle_stop():
    pi.write(23, 0)
    pi.write(24, 0)
    vehicle_turn(1500)

def vehicle_steeringTest():
    pi.write(23, 0)
    pi.write(24, 0)

def vehicle_turn(angle):
    # 800 - 2000
    # 1500 is Center
    pi.set_servo_pulsewidth(18, angle)

def bswap(val):
    return struct.unpack('<H', struct.pack('>H', val))[0]
def mread_word_data(adr, reg):
    return bswap(bus.read_word_data(adr, reg))
def mwrite_word_data(adr, reg, data):
    return bus.write_word_data(adr, reg, bswap(data))
def makeuint16(lsb, msb):
    return ((msb & 0xFF) << 8)  | (lsb & 0xFF)
def VL53L0X_decode_vcsel_period(vcsel_period_reg):
    vcsel_period_pclks = (vcsel_period_reg + 1) << 1;
    return vcsel_period_pclks;
    
def startServer(connection, client_address, sendComm):
    try:
        # use video-port for captures
        for frame in camera.capture_continuous(stream, 'jpeg', use_video_port=True):
            connection.write(struct.pack('<L', stream.tell()))
            connection.flush()
            stream.seek(0)

            # LiDAR
            distance = '%04d' % (0)
            try:
                val1 = bus.write_byte_data(address, VL53L0X_REG_SYSRANGE_START, 0x01)
                data = bus.read_i2c_block_data(address, 0x14, 12)
                if makeuint16(data[11], data[10]) < 1500 and makeuint16(data[11], data[10]) > 20:
                    distance = '%04d' % (makeuint16(data[11], data[10]))
            except Exception as e:
                pass
            
            connection.write(stream.read()+distance.encode())
            stream.seek(0)
            stream.truncate()
            command = sendComm.recv(6).decode()
            if command[0] == 'S' and command[5] == 'E':
                vehicle_turn(int(command[2:5])*10)
                time.sleep(0.1)
                
                if command[1] == '1':
                    vehicle_forward()
                elif command[1] == '0':
                    vehicle_stop()
                elif command[1] == '2':
                    vehicle_backward()
                elif command[1] == '3': # Test mode
                    vehicle_steeringTest()
            else:
                print('Msg Error!!')


        connection.write(struct.pack('<L', 0))
            
    finally:
        print('[Server] Closing the connection...')
        connection.close()


print('##### Starting Server #####')
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(server_address)
pi = pigpio.pi()
pi.set_mode(18, pigpio.OUTPUT) # Steering Motor
pi.set_mode(23, pigpio.OUTPUT) # Wheel Motor
pi.set_mode(24, pigpio.OUTPUT) # Wheel Motor


# LiDAR Sensor Setup
try:
    VL53L0X_REG_IDENTIFICATION_MODEL_ID		= 0x00c0
    VL53L0X_REG_IDENTIFICATION_REVISION_ID		= 0x00c2
    VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD	= 0x0050
    VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD	= 0x0070
    VL53L0X_REG_SYSRANGE_START			= 0x000

    VL53L0X_REG_RESULT_INTERRUPT_STATUS 		= 0x0013
    VL53L0X_REG_RESULT_RANGE_STATUS 		= 0x0014

    address = 0x29
    bus = smbus.SMBus(1)
except Exception as e:
    print('No LiDAR device detected.')
    
while True:
    print('[Server] ### Listening for connection...')
    server_socket.listen(1)
    connection, client_address = server_socket.accept()
    sendComm = connection
    connection = connection.makefile('wb')
    vehicle_stop()
    time.sleep(1) # GPIO warmup
    try:
        print('[Server] %s is now connected.' % client_address[0])
        startServer(connection, client_address, sendComm)
    except Exception as e:
        pass
    finally:
        print('[Server] %s is now disconnected.' % client_address[0])
        vehicle_stop()
        time.sleep(2)

steer.stop()
server_socket.close()
