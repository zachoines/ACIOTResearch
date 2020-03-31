import threading
from TargetDetectAndTrack.ObjectDetector import CascadeDetector
from TargetDetectAndTrack.pid import PID
import time
import cv2
from Hardware import EnvironmentalProcessingAndActuationUnit as EPAU

# Init our connection arduino system with its sensors and actuators
epau = None
servo1Range = (-65, 65)
servo2Range = (-65, 65)
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


def servoTest(epau):
    for i in range(-90, 90, 5):
        j = int(map(i, servo1Range[0], servo1Range[1], 0, 180))
        print("Current angle is: " + str(i))
        epau.pan(j)
        epau.tilt(j)
        time.sleep(1)


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
                    time.sleep(1)




def muilti_threaded_tracking():
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

    def detecter_process(isRunning, centerX, centerY, objY, objX):

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
        while isRunning.value:

            try:
                if in_range(panAngle.value, servo1Range[0], servo1Range[1]):
                    epau.pan(int(map(panAngle.value, servo1Range[0], servo1Range[1], 25, 155)))

                if in_range(tltAngle.value, servo2Range[0], servo2Range[1]):
                    epau.tilt(int(map(tltAngle.value, servo2Range[0], servo2Range[1], 25, 155)))

            finally:
                pass
                # time.sleep(random.random())      

    manager = multiprocessing.Manager()

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

    isRunning = manager.Value('i', 1)

    # Pan/Tilt angles
    tltAngle = manager.Value('f', 0, lock=True)
    panAngle = manager.Value('f', 90, lock=True)

    # Object center coordinates
    centerX = manager.Value('f', 0, lock=True)
    centerY = manager.Value('f', 0, lock=True)

    # Object coordinates
    objX = manager.Value('f', 0, lock=True)
    objY = manager.Value('f', 0, lock=True)

    # Pan PID
    panP = manager.Value('f', 0.02, lock=True) 
    panI = manager.Value('f', 0.01, lock=True) 
    panD = manager.Value('f', 0.007, lock=True)

    # Tilt PID
    tiltP = manager.Value('f', 0.02, lock=True)
    tiltI = manager.Value('f', 0.01, lock=True)
    tiltD = manager.Value('f', 0.007, lock=True)


    # Init processes
    ObjectCenterProcess = multiprocessing.Process(
        target=detecter_process,
        args=(isRunning, centerX, centerY, objY, objX)
    )

    ServoProcess = multiprocessing.Process(
        target=pan_tilt_process,
        args=(tiltP, tiltI, tiltD, panP, panI, panD, centerX, centerY, objY, objX, panAngle, tltAngle, isRunning)
    )

    PanTiltProcess = multiprocessing.Process(
        target=servo_process,
        args=(panAngle, tltAngle, isRunning)
    )

    jobs = [ObjectCenterProcess, PanTiltProcess, ServoProcess]

    for job in jobs:
        job.start()

    for job in jobs:
        job.join()


# check to see if this is the main body of execution
if __name__ == "__main__":
    muilti_process_tracking()
    # single_threaded_tracking()
