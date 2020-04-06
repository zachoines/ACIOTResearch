import threading
from TargetDetectAndTrack.ObjectDetector import CascadeDetector
from TargetDetectAndTrack.pid import PID
import time
import cv2
import numpy as np
from Hardware import EnvironmentalProcessingAndActuationUnit as EPAU

# Init our connection arduino system with its sensors and actuators
epau = None
servo1Range = (-65, 65)
servo2Range = (-65, 65)
# lock = threading.Lock()

# Frame dimentions
frame_width=3264
frame_height=2464

def gstreamer_pipeline(
    capture_width=frame_width,
    capture_height=frame_height,
    display_width=1280,
    display_height=720,
    framerate=10,
    flip_method=6,
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

def in_range(val, start, end):
    return (val >= start and val <= end)


def single_threaded_tracking(showImage=True):
    
    initialized = False
    try:
        epau = EPAU()
        initialized = True
        print("SUCCESS: Connection with remote EPAU established!")
    except:
        print("ERROR: Issue initializing remote EPAU.")
    finally:
        if not initialized:
            print("Exiting now...")

    epau.resetServos()
    # servoTest(epau)
    # epau.resetServos()

    tltAngle = 0
    panAngle = 90

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
    panP = 0.02
    panI = 0.01
    panD = 0.007

    # Tilt PID
    tiltP = 0.02
    tiltI = 0.01
    tiltD = 0.007

    # Misc variables
    currentTries = 0
    currentResets = 0
    maxAttempts = 20

    # initialize the object center finder
    cd = CascadeDetector()
    cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

    # init the PIDS
    p_pan = PID(panP, panI, panD)
    p_pan.initialize()
    p_tilt = PID(tiltP, tiltI, tiltD)
    p_tilt.initialize()
    

    while True:
        if cap.isOpened():
            ret, frame = cap.read()

            if not ret:
                continue

            # First capture a target
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
                    currentResets += 1
                    print("ERROR: No face found, resetting servos. Num resets so far: " + str(currentResets))
                    currentTries = 0
                    # epau.resetServos()
            else:

                # Now determine error for tracking adustment
                tlt_error = centerY - objY
                pan_error = centerX - objX
                tltAngle = p_tilt.update(tlt_error)
                panAngle = p_pan.update(pan_error)

                print("Here are the current angles. Pan: " + str(panAngle) + ", and Tilt: " + str(tltAngle))

                # Show to user
                if showImage:
                    (x, y, w, h) = rect
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                    cv2.imshow("Pan-Tilt Face Tracking", frame)
                    cv2.waitKey(1)


                # Lastly reach out to remote hardware for update
                try:
                    
                    if in_range(panAngle, servo1Range[0], servo1Range[1]):
                        epau.pan(int(map(panAngle, servo1Range[0], servo1Range[1], 25, 155)))

                    if in_range(tltAngle, servo2Range[0], servo2Range[1]):
                        epau.tilt(int(map(tltAngle, servo2Range[0], servo2Range[1], 25, 155)))
                except:
                    print("ERROR: Disconnected with EPU.")
                finally:
                    time.sleep(.2)


def muilti_threaded_tracking():
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
                    # time.sleep(random.random())
        
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
                # time.sleep(random.random())

    def servo_thread():
        global isRunning
        global tltAngle
        global panAngle

        while isRunning:       
            # calculate the error
            lock.acquire()

            try:
                if in_range(panAngle, servo1Range[0], servo1Range[1]):
                    epau.pan(int(map(panAngle, servo1Range[0], servo1Range[1], 45, 135)))

                if in_range(tltAngle, servo2Range[0], servo2Range[1]):
                    epau.tilt(int(map(tltAngle, servo2Range[0], servo2Range[1], 0, 45)))

            finally:
                lock.release()
                # time.sleep(random.random())       

    initialized = False
    try:
        epau = EPAU()
        initialized = True
    except:
        print("ERROR: Issue initializing remote EPU.")
    finally:
        if not initialized:
            print("Exiting now...")

    epau.resetServos()

    isRunning = 1
    tltAngle = 0
    panAngle = 90

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
    tiltP = 0.1
    tiltI = 0.10
    tiltD = 0.002

    # Init processes
    ObjectCenterThread = threading.Thread(target=detecter_thread, args=())
    ServoThread = threading.Thread(target=servo_thread, args=())
    PanTiltThread = threading.Thread(target=pan_tilt_thread, args=(tiltP, tiltI, tiltD, panP, panI, panD))

    jobs = [ObjectCenterThread, PanTiltThread, ServoThread]

    for job in jobs:
        job.start()

    for job in jobs:
        job.join()


def muilti_process_tracking(showImage=True):
    import multiprocessing
    from multiprocessing import Lock, Array
    import time
    import random
    import ctypes

    def camera_process(isRunning, frame):

        cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

        while isRunning.value:
            try:
                frame.acquire()
                if cap.isOpened():
                    ret, newFrame = cap.read()

                    if not ret:
                        continue
                    else:
                        np_frame = np.frombuffer(frame.get_obj(), dtype=ctypes.c_ubyte).reshape((720, 1280, 3))
                        np.copyto(np_frame, newFrame)
            finally:
                frame.release()
                        
        cap.release() 

    def object_detecter_process(isRunning, frame, centerX, centerY, objY, objX):
        currentTries = 0
        maxAttempts = 5

        # initialize the object center finder
        cd = CascadeDetector()

        while isRunning.value:          
            try:
                frame.acquire()
                centerX.value = x = frame_width // 2
                centerY.value = y = frame_height // 2
                np_frame = np.frombuffer(frame.get_obj(), dtype=ctypes.c_ubyte).reshape((720, 1280, 3))
                objectLoc = cd.detect(np_frame, (x, y))
                ((x, y), rect) = objectLoc

                objY.value = y
                objX.value = x
                    
                currentTries += 1
                if currentTries >= maxAttempts:
                    currentTries = 0
                    # epau.resetServos() 

                # Show to user
                if showImage and rect is not None:
                    (x, y, w, h) = rect
                    cv2.rectangle(np_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.imshow("Pan-Tilt Face Tracking", np_frame)
                    cv2.waitKey(1)          
            finally:
                frame.release()
                        
        cap.release()

    def capture_and_detect_process(isRunning, centerX, centerY, objY, objX):

        currentTries = 0
        maxAttempts = 5

        # initialize the object center finder
        cd = CascadeDetector()
        cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

        while isRunning.value:
            if cap.isOpened():
                ret, frame = cap.read()

                if not ret:
                    continue
                
                try:
                    (H, W) = frame.shape[:2]
                    centerX.value = x = W // 2
                    centerY.value = y = H // 2

                    objectLoc = cd.detect(frame, (x, y))
                    ((x, y), rect) = objectLoc

                    objY.value = y
                    objX.value = x

                    if rect is None:
                        currentTries += 1
                        if currentTries >= maxAttempts:
                            currentTries = 0
                            # epau.resetServos() 
                    else:
                        # Show to user
                        if showImage:
                            (x, y, w, h) = rect
                            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                            cv2.imshow("Pan-Tilt Face Tracking", frame)
                            cv2.waitKey(1)

                    
                finally:
                    pass
                    # time.sleep(random.random())
                        
        cap.release()

    def pan_tilt_process(pt, it, dt, pp, ip, dp, centerX, centerY, objY, objX, panAngle, tltAngle, isRunning):

        # Create a PID's and initialize
        p_pan = PID(pp.value, ip.value, dp.value)
        p_pan.initialize()
        p_tilt = PID(pt.value, it.value, dt.value)
        p_tilt.initialize()

        # loop indefinitely
        while isRunning.value:
            try:
                tlt_error = centerY.value - objY.value
                pan_error = centerX.value - objX.value
                tltAngle.value = p_tilt.update(tlt_error) 
                panAngle.value = p_pan.update(pan_error)
            finally:
                pass
                # time.sleep(random.random())
                
    def servo_process(panAngle, tltAngle, isRunning):
        lastPan = 0
        lastTilt = 0

        while isRunning.value:
            currentPan = panAngle.value
            currentTilt = tltAngle.value
            try:
                if currentPan != lastPan and in_range(currentPan, servo1Range[0], servo1Range[1]):
                    lastPan = currentPan
                    epau.pan(int(map(currentPan, servo1Range[0], servo1Range[1], 25, 155)))

                if currentTilt != lastTilt and in_range(currentTilt, servo2Range[0], servo2Range[1]):
                    lastTilt = currentTilt
                    epau.tilt(int(map(currentTilt, servo2Range[0], servo2Range[1], 25, 155)))

            finally:
                time.sleep(.2)     

    manager = multiprocessing.Manager()
    epau = None
    initialized = False
    try:
        # epau = EPAU()
        initialized = True
        # epau.resetServos()
    except:
        print("ERROR: Issue initializing remote EPU.")
    finally:
        if not initialized:
            print("Exiting now...")
            exit(1)

    # Init shared array in memory 
    lock = Lock()
    isRunning = manager.Value('i', 1)
    mockFrame = np.ones(shape=(720, 1280, 3), dtype=ctypes.c_ubyte)
    frame = Array(ctypes.c_ubyte, (720 * 1280 * 3), lock=lock)
    np_frame = np.frombuffer(frame.get_obj(), dtype=ctypes.c_ubyte).reshape((720, 1280, 3))
    np.copyto(np_frame, mockFrame)
    
    # Pan/Tilt angles
    tltAngle = manager.Value('f', 0, lock=False)
    panAngle = manager.Value('f', 90, lock=False)

    # Object center coordinates
    centerX = manager.Value('f', 0, lock=False)
    centerY = manager.Value('f', 0, lock=False)

    # Object coordinates
    objX = manager.Value('f', 0, lock=False)
    objY = manager.Value('f', 0, lock=False)

    # Pan PID
    panP = manager.Value('f', 0.02, lock=False) 
    panI = manager.Value('f', 0.01, lock=False) 
    panD = manager.Value('f', 0.007, lock=False)

    # Tilt PID
    tiltP = manager.Value('f', 0.02, lock=False)
    tiltI = manager.Value('f', 0.01, lock=False)
    tiltD = manager.Value('f', 0.007, lock=False)

    # Init processes

    captureAndDetectProcess = multiprocessing.Process(
        target=capture_and_detect_process,
        args=(isRunning, centerX, centerY, objY, objX)
    )

    CameraProcess = multiprocessing.Process(
        target=camera_process,
        args=(isRunning, frame)
    )

    ObjectDetectorProcess = multiprocessing.Process(
        target=object_detecter_process,
        args=(isRunning, frame, centerX, centerY, objY, objX)
    )

    ServoProcess = multiprocessing.Process(
        target=pan_tilt_process,
        args=(tiltP, tiltI, tiltD, panP, panI, panD, centerX, centerY, objY, objX, panAngle, tltAngle, isRunning)
    )

    PanTiltProcess = multiprocessing.Process(
        target=servo_process,
        args=(panAngle, tltAngle, isRunning)
    )

    # jobs = [CameraProcess, ObjectDetectorProcess, PanTiltProcess]
    jobs = [captureAndDetectProcess, PanTiltProcess]

    for job in jobs:
        job.start()

    for job in jobs:
        job.join()


# check to see if this is the main body of execution
if __name__ == "__main__":
    muilti_process_tracking(showImage=False)
    # single_threaded_tracking()
