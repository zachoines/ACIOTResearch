from smbus2 import SMBus
import Jetson.GPIO as GPIO
import numpy as np
import serial


class EnvironmentalProcessingAndActuationUnit(object):

    def __init__(self, i2c_address=0x9, servo_1_default=0x5A, servo_2_default=0x5A):
        self.__i2c_address = i2c_address
        self.__servo_1_default = servo_1_default
        self.__servo_2_default = servo_2_default
        self.__serial_port = serial.Serial(
            port="/dev/ttyTHS1",
            baudrate=9600,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )

    def __del__(self):
        self.__serial_port.close()

    def __toFloat(self, byteArray):
        data_bytes = np.array(byteArray, dtype=np.uint8)
        return data_bytes.view(dtype=np.float32)

    def __toDouble(self, byteArray):
        data_bytes = np.array(byteArray, dtype=np.uint8)
        return data_bytes.view(dtype=np.float64)

    def __toInt16(self, byteArray):
        data_bytes = np.array(byteArray, dtype=np.uint8)
        return data_bytes.view(dtype=np.int16)

    def __toLong(self, byteArray):
        data_bytes = np.array(byteArray, dtype=np.long)
        return data_bytes.view(dtype=np.long)

    def __reverseList(self, lst):
        return [ele for ele in reversed(lst)]

    def __split_list(self, a_list):
        half = len(a_list)//2
        return a_list[:half], a_list[half:]

    def getLinearAccelData(self):
        commandBits = 0x7
        dataBits = 0x00
        dataBits &= 0xFF
        finalCommand = (commandBits << 8) + dataBits
        finalCommand &= 0xFFFF

        with SMBus(1) as bus:
            structBytes = finalCommand.to_bytes(2, byteorder="big", signed=False)
            commandByteArray = bytearray(structBytes)
            finalCommandBytes = list(commandByteArray)
            bus.write_i2c_block_data(9, 0, finalCommandBytes)
            bytes_array = bus.read_i2c_block_data(9, 0, 24)
            xBytes, yBytes, zBytes = np.array_split(bytes_array, 3)
            linearAccelData = [self.__toDouble(xBytes), self.__toDouble(yBytes), self.__toDouble(zBytes)]       
            return linearAccelData

    def getSonicSensorReadings(self):
        commandBits = 0x4
        dataBits = 0x00
        dataBits &= 0xFF
        finalCommand = (commandBits << 8) + dataBits
        finalCommand &= 0xFFFF

        with SMBus(1) as bus:
            structBytes = finalCommand.to_bytes(2, byteorder="big", signed=False)
            commandByteArray = bytearray(structBytes)
            finalCommandBytes = list(commandByteArray)
            bus.write_i2c_block_data(9, 0, finalCommandBytes)

            bytes_array = bus.read_i2c_block_data(9, 0, 8)
            distance1bytes, distance2bytes = self.__split_list(bytes_array) 
            distance_one = self.__toFloat(distance1bytes)
            distance_two = self.__toFloat(distance2bytes)

            return [distance_one, distance_two]

    # Call the hardware level EPUA reset command
    def __reset(self):
        commandBits = 0x8
        dataBits = 0xFF
        finalCommand = (commandBits << 8) + dataBits
        finalCommand &= 0xFFFF

        with SMBus(1) as bus:
            structBytes = finalCommand.to_bytes(2, byteorder="big", signed=False)
            commandByteArray = bytearray(structBytes)
            finalCommandBytes = list(commandByteArray)
            bus.write_i2c_block_data(self.__i2c_address, 0, finalCommandBytes)
            bus.read_i2c_block_data(self.__i2c_address, 0, 2)

    def resetServos(self):
        self.moveServo1(self.__servo_1_default)
        self.moveServo2(self.__servo_2_default)

    def moveServo1(self, angle):
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
            bus.write_i2c_block_data(self.__i2c_address, 0, finalCommandBytes)
            bus.read_i2c_block_data(self.__i2c_address, 0, 2)
    
    def moveServo2(self, angle):
        commandBits = 0x3
        dataBits = angle
        dataBits &= 0xFF
        finalCommand = (commandBits << 8) + dataBits
        finalCommand &= 0xFFFF

        with SMBus(1) as bus:
            structBytes = finalCommand.to_bytes(2, byteorder="big", signed=False)
            commandByteArray = bytearray(structBytes)
            finalCommandList = list(commandByteArray)
            bus.write_i2c_block_data(self.__i2c_address, 0, finalCommandList)
            bus.read_i2c_block_data(self.__i2c_address, 0, 2)

    def pan(self, angle):
        self.moveServo1(angle)

    def tilt(self, angle):
        self.moveServo2(angle)
      
    def setWheelSpeeds(self, ws1, ws2):

        with SMBus(1) as bus:

            commandBits = 0x5
            dataBits = ws1
            dataBits &= 0xFF
            finalCommand = (commandBits << 8) + dataBits
            finalCommand &= 0xFFFF

            structBytes = finalCommand.to_bytes(2, byteorder="big", signed=False)
            commandByteArray = bytearray(structBytes)
            finalCommandList = list(commandByteArray)
            bus.write_i2c_block_data(self.__i2c_address, 0, finalCommandList)
            bus.read_i2c_block_data(self.__ic2_address, 0, 2)

            commandBits = 0x6
            dataBits = ws2
            dataBits &= 0xFF
            finalCommand = (commandBits << 8) + dataBits
            finalCommand &= 0xFFFF

            structBytes = finalCommand.to_bytes(2, byteorder="big", signed=False)
            commandByteArray = bytearray(structBytes)
            finalCommandList = list(commandByteArray)
            bus.write_i2c_block_data(self.__i2c_address, 0, finalCommandList)
            bus.read_i2c_block_data(self.__i2c_address, 0, 2)

    # Serial communication with EPU
    def __parseEPUData(self, data):
        distanceOne = self.__toFloat(data[0:4])
        distanceTwo = self.__toFloat(data[4:8])
        linearAccelX = self.__toDouble(data[8:16])
        linearAccelY = self.__toDouble(data[16:24])
        linearAccelZ = self.__toDouble(data[24:32])
        distanceToTarget = self.__toInt16(data[32:34])
        timestamp = self.__toLong(data[34:40])
        return [distanceOne, distanceTwo, linearAccelX, linearAccelY, linearAccelZ, distanceToTarget, timestamp]


    def readSensorDataUART(self):
        receivedBytes = []
        newData = False
        numBytes = 40
        recvInProgress = False
        ndx = 0
        startMarker = 0x3C
        endMarker = 0x3E
        command = 0x41
        rb = 0

        try:
            # for byte in message:
            self.__serial_port.write('<A>'.encode('ascii'))

            while not newData:
                if self.__serial_port.inWaiting() > 0:

                    rb = self.__serial_port.read()

                    if recvInProgress:
                        if ord(rb) != endMarker:
                            receivedBytes.append(ord(rb))
                            ndx += 1
                            if ndx >= numBytes:
                                ndx = numBytes - 1

                        else:
                            recvInProgress = False
                            ndx = 0
                            newData = True

                    elif ord(rb) == startMarker:
                        recvInProgress = True

            if newData:
                self.__parseEPUData(receivedBytes)
                receivedBytes = []
                newData = False
            else:
                return [None, None, None, None, None, None, None]

        except Exception as exception_error:
            print("Error occurred. Exiting Program")
            print("Error: " + str(exception_error))
