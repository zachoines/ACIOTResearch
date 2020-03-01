# import tensorflow as tf
# import numpy as np

# from PPO2.Train import Train
# from PPO2.Test import Test
# from PPO2.Config import config_pacman as config

from smbus2 import SMBus
import numpy as np
import time
import serial
import struct
from enum import Enum
import os

class direction(Enum):
    FORWARD = 0
    REVERSE = 1
    LEFT = 2
    RIGHT = 3



# Open i2c bus 1 and read one byte from address 80, offset 0
def toFloat(byteArray):
    data_bytes = np.array(byteArray, dtype=np.uint8)
    return data_bytes.view(dtype=np.float32)

def toInt16(byteArray):
    data_bytes = np.array(byteArray, dtype=np.uint8)
    return data_bytes.view(dtype=np.int16)

def reverseList(lst): 
    return [ele for ele in reversed(lst)] 

def split_list(a_list):
    half = len(a_list)//2
    return a_list[:half], a_list[half:]

def getSensorReadings():
    with SMBus(1) as bus:
        bus.write_byte(9, 1)
        # a, b = split_list(bus.read_i2c_block_data(9, 0, 2))
        # c = toFloat(a)
        # d = toFloat(b)
        # print(c)
        # print(d)
        byte_array = bus.read_i2c_block_data(9, 0, 2)
        e = toInt16(byte_array)
        print(e)

def moveServo1(angle):
    commandBits = 0x2
    dataBits = angle
    dataBits &= 0xFF
    finalCommand = (commandBits << 8) + dataBits
    finalCommand &= 0xFFFF

    with SMBus(1) as bus:
        # structBytes = struct.pack("H", finalCommand)
        structBytes = finalCommand.to_bytes(2, byteorder="big", signed=False)
        commandByteArray = bytearray(structBytes)
        finalCommandBytes = list(commandByteArray)
        bus.write_i2c_block_data(9, 0, finalCommandBytes)
        bus.read_i2c_block_data(9, 0, 2)
        
def wheel1(percent, direction = "FORWRD"):

    

    commandBits = 0x4
    dataBits = angle
    dataBits &= 0xFF
    finalCommand = (commandBits << 8) + dataBits
    finalCommand &= 0xFFFF

    with SMBus(1) as bus:
        # structBytes = struct.pack("H", finalCommand)
        structBytes = finalCommand.to_bytes(2, byteorder="big", signed=False)
        commandByteArray = bytearray(structBytes)
        finalCommandList = list(commandByteArray)
        bus.write_i2c_block_data(9, 0, finalCommandList)
        bus.read_i2c_block_data(9, 0, 2)

def wheel2(speed, direction):
    commandBits = 0x5
    dataBits = angle
    dataBits &= 0xFF
    finalCommand = (commandBits << 8) + dataBits
    finalCommand &= 0xFFFF

    with SMBus(1) as bus:
        # structBytes = struct.pack("H", finalCommand)
        structBytes = finalCommand.to_bytes(2, byteorder="big", signed=False)
        commandByteArray = bytearray(structBytes)
        finalCommandList = list(commandByteArray)
        bus.write_i2c_block_data(9, 0, finalCommandList)
        bus.read_i2c_block_data(9, 0, 2)

def moveServo2(angle):
    commandBits = 0x3
    dataBits = angle
    dataBits &= 0xFF
    finalCommand = (commandBits << 8) + dataBits
    finalCommand &= 0xFFFF

    with SMBus(1) as bus:
        # structBytes = struct.pack("H", finalCommand)
        structBytes = finalCommand.to_bytes(2, byteorder="big", signed=False)
        commandByteArray = bytearray(structBytes)
        finalCommandList = list(commandByteArray)
        bus.write_i2c_block_data(9, 0, finalCommandList)
        bus.read_i2c_block_data(9, 0, 2)
        

def testServos():
    # Servo sweep from 0 position to 180
    for i in range(90):
        moveServo1(i)
        time.sleep(.1)

    # Servo sweep from 180 position to 0
    for j in reversed(range(90)):
        moveServo2(j)
        time.sleep(.1)

    # Servo sweep from 180 position to 0
    for i in reversed(range(90)):
        moveServo1(i)
        time.sleep(.1)

    # Servo sweep from 0 position to 180
    for j in range(90):
        moveServo2(j)
        time.sleep(.1)

def sendCommand():  
    pass

def receiveResponse():
    pass

def main():
    serial_port = serial.Serial(
        port="/dev/ttyTHS1",
        baudrate=9600,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
    )
    # Wait a second to let the port initialize
    # time.sleep(1)

    try:
        # Send a simple header
        serial_port.write("Hi there!!\r\n".encode())
        while True:
            if serial_port.inWaiting() > 0:
                data = serial_port.read()
                print(data)
                serial_port.write(data)




    except KeyboardInterrupt:
        print("Exiting Program")

    except Exception as exception_error:
        print("Error occurred. Exiting Program")
        print("Error: " + str(exception_error))

    finally:
        serial_port.close()
        pass
    

def PPO_Start():
    tf.keras.backend.set_floatx('float64')
    np.random.seed(42)
    tf.random.set_seed(42)

    train_session = Train(config)

    if (train_session.start()):
        Test(config)

if __name__ == '__main__':
    for i in range(1000):
        # getSensorReadings()  
        testServos()
        time.sleep(1)
    # main()

    
    
    

