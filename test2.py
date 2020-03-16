import multiprocessing
from multiprocessing import Manager
from multiprocessing import Process
from TargetDetectAndTrack.objcenter import ObjCenter
from TargetDetectAndTrack.pid import PID
import signal
import time
import cv2
import sys
from Hardware import EnvironmentalProcessingAndActuationUnit as EPAU

# import ptvsd
# ptvsd.debug_this_thread()

# Init our connection arduino system with its sensors and actuators
epau = EPAU()

# define the range for the motors
servoRange = (15, 175)


def gstreamer_pipeline(
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=60,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


def signal_handler(sig, frame):
    print("Exiting...")
    sys.exit()


def obj_center(objX, objY, centerX, centerY, isRunning):
    time.sleep(2.0)

    # initialize the object center finder
    obj = ObjCenter()
    cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

    
    while isRunning:
        if cap.isOpened():
            ret, frame = cap.read()
            (H, W) = frame.shape[:2]
            centerX.value = W // 2
            centerY.value = H // 2

            objectLoc = obj.update(frame, (centerX.value, centerY.value))
            ((objX.value, objY.value), rect) = objectLoc

            # extract the bounding box and draw it
            if rect is not None:
                (x, y, w, h) = rect
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0),
                    2)

            # display the frame to the screen
            # cv2.imshow("Pan-Tilt Face Tracking", frame)
            # cv2.waitKey(1)
    cap.release()

def pid_process(output, p, i, d, objCoord, centerCoord, isRunning):

    # create a PID and initialize it
    p = PID(p.value, i.value, d.value)
    p.initialize()

    # loop indefinitely
    while isRunning:
        # calculate the error
        error = centerCoord.value - objCoord.value

        # update the value
        output.value = p.update(error)

def in_range(val, start, end):
    # determine the input vale is in the supplied range
    return (val >= start and val <= end)

def set_servos(pan, tlt, isRunning):

    while isRunning:
        # time.sleep(2.0)
        # the pan and tilt angles are reversed
        panAngle = -1 * pan.value
        tltAngle = -1 * tlt.value

        # if the pan angle is within the range, pan
        if in_range(panAngle, servoRange[0], servoRange[1]):
            epau.pan(int(panAngle))
            print("Pan angle: " + str(panAngle))

        # if the tilt angle is within the range, tilt
        if in_range(tltAngle, servoRange[0], servoRange[1]):
            epau.tilt(int(tltAngle))
            print("Tilt angle: " + str(tltAngle))

    time.wait(1)

# check to see if this is the main body of execution
if __name__ == "__main__":
    multiprocessing.set_start_method('spawn', True)

    with Manager() as manager:
        
        # Indicator to terminiate processes
        # signal.signal(signal.SIGINT, signal_handler)
        isRunning = manager.Value("i", 1)

        # Object center coordinates
        centerX = manager.Value("i", 0)
        centerY = manager.Value("i", 0)

        # Object coordinates
        objX = manager.Value("i", 0)
        objY = manager.Value("i", 0)

        # Pan/Tilt angles
        pan = manager.Value("i", 0)
        tlt = manager.Value("i", 0)

        # Pan PID
        panP = manager.Value("f", 0.09)
        panI = manager.Value("f", 0.08)
        panD = manager.Value("f", 0.002)

        # Tilt PID
        tiltP = manager.Value("f", 0.11)
        tiltI = manager.Value("f", 0.10)
        tiltD = manager.Value("f", 0.002)

        # Init processes
        processObjectCenter = Process(target=obj_center, args=(objX, objY, centerX, centerY, isRunning))
        processPanning = Process(target=pid_process, args=(pan, panP, panI, panD, objX, centerX, isRunning))
        processTilting = Process(target=pid_process, args=(tlt, tiltP, tiltI, tiltD, objY, centerY, isRunning))
        processSetServos = Process(target=set_servos, args=(pan, tlt, isRunning))

        # Start processes
        processObjectCenter.start()
        processPanning.start()
        processTilting.start()
        processSetServos.start()


        jobs = [processObjectCenter, processPanning, processTilting, processSetServos]
        # Join processes
        for _ in range(10000):
            # time.sleep(2)
            pass

        for i in jobs:
            i.terminate()
        sys.exit(0)
        # processObjectCenter.join()
        # processPanning.join()
        # processTilting.join()
        # processSetServos.join()