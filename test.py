# import tensorflow as tf
# import numpy as np

# from PPO2.Train import Train
# from PPO2.Test import Test
# from PPO2.Config import config_pacman as config

from smbus2 import SMBus
import Jetson.GPIO as GPIO
import numpy as np
# import tensorflow as tf
import time
import serial

INTERRUPT_PIN = 7

# typedef struct {
#   int16_t accel_offset_x; /**< x acceleration offset */
#   int16_t accel_offset_y; /**< y acceleration offset */
#   int16_t accel_offset_z; /**< z acceleration offset */

#   int16_t mag_offset_x; /**< x magnetometer offset */
#   int16_t mag_offset_y; /**< y magnetometer offset */
#   int16_t mag_offset_z; /**< z magnetometer offset */

#   int16_t gyro_offset_x; /**< x gyroscrope offset */
#   int16_t gyro_offset_y; /**< y gyroscrope offset */
#   int16_t gyro_offset_z; /**< z gyroscrope offset */

#   int16_t accel_radius; /**< acceleration radius */

#   int16_t mag_radius; /**< magnetometer radius */
# } adafruit_bno055_offsets_t;

# Open i2c bus 1 and read one byte from address 80, offset 0


def toFloat(byteArray):
    data_bytes = np.array(byteArray, dtype=np.uint8)
    return data_bytes.view(dtype=np.float32)


def toDouble(byteArray):
    data_bytes = np.array(byteArray, dtype=np.uint8)
    return data_bytes.view(dtype=np.float64)


def toInt16(byteArray):
    data_bytes = np.array(byteArray, dtype=np.uint8)
    return data_bytes.view(dtype=np.int16)


def reverseList(lst):
    return [ele for ele in reversed(lst)]


def split_list(a_list):
    half = len(a_list)//2
    return a_list[:half], a_list[half:]


def getLinearAccelData():
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
        linearAccelData = [toDouble(xBytes), toDouble(yBytes), toDouble(zBytes)]
        print(linearAccelData)
        return linearAccelData


def getSonicSensorReadings():
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
        distance1bytes, distance2bytes = split_list(bytes_array) 
        distance_one = toFloat(distance1bytes)
        distance_two = toFloat(distance2bytes)

        print(distance_one)
        print(distance_two)

        return distance_one, distance_two


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
       

def setWheelSpeeds(ws1, ws2):

    with SMBus(1) as bus:

        commandBits = 0x5
        dataBits = ws1
        dataBits &= 0xFF
        finalCommand = (commandBits << 8) + dataBits
        finalCommand &= 0xFFFF

        structBytes = finalCommand.to_bytes(2, byteorder="big", signed=False)
        commandByteArray = bytearray(structBytes)
        finalCommandList = list(commandByteArray)
        bus.write_i2c_block_data(9, 0, finalCommandList)
        bus.read_i2c_block_data(9, 0, 2)

        commandBits = 0x6
        dataBits = ws2
        dataBits &= 0xFF
        finalCommand = (commandBits << 8) + dataBits
        finalCommand &= 0xFFFF

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
        structBytes = finalCommand.to_bytes(2, byteorder="big", signed=False)
        commandByteArray = bytearray(structBytes)
        finalCommandList = list(commandByteArray)
        bus.write_i2c_block_data(9, 0, finalCommandList)
        bus.read_i2c_block_data(9, 0, 2)


def testAccelSensors():
    for _ in range(10):
        getLinearAccelData()


def testSonicSensors():
    for _ in range(10):
        getSonicSensorReadings()


def testWheels():
    for i in range(0, 100, 10):
        setWheelSpeeds(i, i)
        time.sleep(1)

    for i in reversed(range(0, 100, 10)):
        setWheelSpeeds(i, i)
        time.sleep(1)


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


# Serial communication with EPU
def parseEPUData(data):
    # float distance_to_obstacle[2]
    # double acceleration[3]
    # unsigned short distance_to_target
    # unsigned long timestamp
    print(data)


def readSensorDataUART(serial_port):
    receivedBytes = []
    newData = False
    numBytes = 40
    recvInProgress = False
    ndx = 0
    startMarker = 0x3C
    endMarker = 0x3E
    command = 0x41
    rb = 0

    # GPIO.output(INTERRUPT_PIN, GPIO.LOW)

    message = [startMarker, command, endMarker]

    # for byte in message:
    serial_port.write(0x3C413E)

    while not newData:
        if serial_port.inWaiting() > 0:

            rb = serial_port.read()

            if recvInProgress:
                if rb != endMarker:
                    receivedBytes.append(rb)
                    ndx += 1
                    if ndx >= numBytes:
                        ndx = numBytes - 1

                else:
                    recvInProgress = False
                    ndx = 0
                    newData = True

            elif rb == startMarker:
                recvInProgress = True

    if newData:
        parseEPUData(receivedBytes)
        receivedBytes = []
        newData = False

    # GPIO.output(INTERRUPT_PIN, GPIO.HIGH)


def testUARTSlave():
    serial_port = serial.Serial(
        port="/dev/ttyTHS1",
        baudrate=9600,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
    )

    #serial_port.close()
    #serial_port.open()

    try:
        for _ in range(1000):
            readSensorDataUART(serial_port)

    except KeyboardInterrupt:
        print("Exiting Program")

    except Exception as exception_error:
        print("Error occurred. Exiting Program")
        print("Error: " + str(exception_error))

    finally:
        serial_port.close()


def main():
    pass


# def PPO_Start():
#     tf.keras.backend.set_floatx('float64')
#     np.random.seed(42)
#     tf.random.set_seed(42)
#     train_session = Train(config)

#     if (train_session.start()):
#         Test(config)


if __name__ == '__main__':
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(INTERRUPT_PIN, GPIO.OUT, initial=GPIO.HIGH)

    # testWheels()
    # testAccelSensors()
    testUARTSlave()
    # testSonicSensors()
    # testServos()
    # main()

    GPIO.cleanup()