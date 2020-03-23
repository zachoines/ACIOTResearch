import threading
from TargetDetectAndTrack.ObjectDetector import CascadeDetector
from TargetDetectAndTrack.pid import PID
import time
import cv2
from Hardware import EnvironmentalProcessingAndActuationUnit as EPAU
import random

# Init our connection arduino system with its sensors and actuators
epau = EPAU()
servo1Range = (-45, 45)
servo2Range = (-22.5, 22.5)
lock = threading.Lock()


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


def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def detecter_thread():

    currentTries = 0
    maxAttempts = 5

    # initialize the object center finder
    cd = CascadeDetector()
    cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

    global isRunning
    global centerX
    global centerY
    global objY
    global objX

    while isRunning:
        if cap.isOpened():
            ret, frame = cap.read()

            if not ret:
                continue

            lock.acquire()
            try:
            
                (H, W) = frame.shape[:2]
                centerX = x = W // 2
                centerY = y = H // 2

                objectLoc = cd.detect(frame, (x, y))
                ((x, y), rect) = objectLoc

                objY = y
                objX = x

                if rect is None:
                    currentTries += 1
                    if currentTries >= maxAttempts:
                        currentTries = 0
                        # epau.resetServos() 
            finally:
                lock.release()
                time.sleep(random.random())

            
    cap.release()

def pan_tilt_thread(pt, it, dt, pp, ip, dp):

    # Create a PID's and initialize
    p_pan = PID(pp, ip, dp)
    p_pan.initialize()
    p_tilt = PID(pt, it, dt)
    p_tilt.initialize()

    global isRunning

    global centerY
    global objY
    global tltAngle

    global centerX
    global objX
    global panAngle

    # loop indefinitely
    while isRunning:
        lock.acquire()
        try:
            tlt_error = centerY - objY
            pan_error = centerX - objX
            tltAngle = p_tilt.update(tlt_error) 
            panAngle = p_pan.update(pan_error)
        finally:
            lock.release()
            time.sleep(random.random())

def servo_thread():
    global isRunning
    global tltAngle
    global panAngle

    while isRunning:
            
        # calculate the error
        lock.acquire()

        try:
            # if in_range(panAngle, servo1Range[0], servo1Range[1]):
            #     epau.pan(int(map(panAngle, servo1Range[0], servo1Range[1], 45, 135)))

            if in_range(tltAngle, servo2Range[0], servo2Range[1]):
                epau.tilt(int(map(tltAngle, servo2Range[0], servo2Range[1], 0, 45)))

        finally:
            lock.release()
            time.sleep(random.random())
            
            

def in_range(val, start, end):
    return (val >= start and val <= end)

# check to see if this is the main body of execution
if __name__ == "__main__":

    epau.resetServos()

    isRunning = 1
    tltAngle = -90
    panAngle = -90

    # Object center coordinates
    centerX = 0
    centerY = 0

    # Object coordinates
    objX = 0
    objY = 0

    # Pan/Tilt angles
    pan = 0
    tlt = 0

    # # Pan PID
    # panP = 0.09
    # panI = 0.08
    # panD = 0.002

    # # Tilt PID
    # tiltP = 0.1
    # tiltI = 0.10
    # tiltD = 0.002

    # Pan PID
    panP = 0.09
    panI = 0.08
    panD = 0.01

    # Tilt PID
    tiltP = 0.02
    tiltI = 0.01
    tiltD = 0.007

    # Init processes
    ObjectCenterThread = threading.Thread(target=detecter_thread, args=())
    ServoThread = threading.Thread(target=servo_thread, args=())
    PanTiltThread = threading.Thread(target=pan_tilt_thread, args=(tiltP, tiltI, tiltD, panP, panI, panD))

    jobs = [ObjectCenterThread, PanTiltThread, ServoThread]

    for job in jobs:
        job.start()

    for job in jobs:
        job.join()



