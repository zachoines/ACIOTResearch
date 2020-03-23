from Hardware import EnvironmentalProcessingAndActuationUnit as EPAU
import time

epau = EPAU()

def test():

    epau.resetServos()

    for i in range(45, 135, 1):
        epau.pan(i)
        time.sleep(0.01)


    epau.resetServos()

    for i in range(0, 45, 1):
        epau.tilt(i)
        time.sleep(0.01)


    epau.resetServos()


if __name__ == '__main__':
    test()