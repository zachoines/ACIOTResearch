import threading
from TargetDetectAndTrack.objcenter import ObjCenter
from TargetDetectAndTrack.pid import PID
import time
import cv2
from Hardware import EnvironmentalProcessingAndActuationUnit as EPAU

# Init our connection arduino system with its sensors and actuators
epau = EPAU()
# servoRange = (15, 175)
servoRange = (-90, 90)
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

def obj_center():
    time.sleep(2.0)

    currentTries = 0
    maxAttempts = 10

    # initialize the object center finder
    obj = ObjCenter()
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
                time.sleep(1)
                continue
            
            (H, W) = frame.shape[:2]
            centerX = x = W // 2
            centerY = y = H // 2

            objectLoc = obj.update(frame, (x, y))
            ((x, y), rect) = objectLoc

            # if rect is None:
                
            #     currentTries += 1
            #     if currentTries >= maxAttempts:
            #         currentTries = 0
            #         epau.resetServos() 
            #         print("No face found.")       

            objY = y
            objX = x
            
    cap.release()

def tilt_thread(p, i, d):

    # create a PID and initialize it
    p = PID(p, i, d)
    p.initialize()

    global isRunning
    global centerY
    global objY

    # loop indefinitely
    while isRunning:

        lock.acquire()

        try:

            # calculate the error
            center_coord = centerY
            object_coord = objY
            # error = (center_coord - object_coord)**2 / 2.0
            error = center_coord - object_coord
            # error = map(error, 0, (720 // 2), 0.0, 1.0)
            tltAngle = p.update(error)

            if in_range(tltAngle, servoRange[0], servoRange[1]):
                tltAngle = map(tltAngle, -90, 90, 0, 180)
                epau.tilt(int(tltAngle))
                print("Tilt angle: " + str(tltAngle))

        finally:
            lock.release()
            
            

def pan_thread(p, i, d):

    # create a PID and initialize it
    p = PID(p, i, d)
    p.initialize()

    global isRunning
    global centerX
    global objX

    while isRunning:
        
        # calculate the error
        lock.acquire()

        try:

            object_coord = objX
            center_coord = centerX
            # error = (center_coord - object_coord)**2 / 2.0
            error = center_coord - object_coord
            # error = map(error, 0, (1280 // 2), 0.0, 1.0)
            panAngle = p.update(error)
 
            if in_range(panAngle, servoRange[0], servoRange[1]):
                panAngle = map(panAngle, -90, 90, 0, 180)
                epau.pan(int(panAngle))
                print("Pan angle: " + str(panAngle))

        finally:
                lock.release()
            

def in_range(val, start, end):
    return (val >= start and val <= end)

# check to see if this is the main body of execution
if __name__ == "__main__":

    epau.resetServos()

    isRunning = 1

    # Object center coordinates
    centerX = 0
    centerY = 0

    # Object coordinates
    objX = 0
    objY = 0

    # Pan/Tilt angles
    pan = 0
    tlt = 0

    # Pan PID
    panP = 0.09
    panI = 0.08
    panD = 0.002

    # Tilt PID
    tiltP = 0.0
    tiltI = 0.0
    tiltD = 0.0

    # Init processes
    ObjectCenterThread = threading.Thread(target=obj_center, args=())
    # PanningThread = threading.Thread(target=pan_thread, args=(panP, panI, panD))
    TiltingThread = threading.Thread(target=tilt_thread, args=(tiltP, tiltI, tiltD))

    # Start processes
    ObjectCenterThread.start()
    # PanningThread.start()
    TiltingThread.start()

    # jobs = [ObjectCenterThread, PanningThread, TiltingThread]
    jobs = [ObjectCenterThread, TiltingThread]
    # jobs = [ObjectCenterThread, PanningThread]

    for job in jobs:
        job.join()
