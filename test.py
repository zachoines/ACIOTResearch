from Hardware import EnvironmentalProcessingAndActuationUnit as EPAU
import time
import cv2

def gstreamer_pipeline(
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=29,
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

def testServos():
    epau = EPAU()
    epau.resetServos()

    for i in range(45, 135, 1):
        epau.pan(i)
        time.sleep(0.01)


    epau.resetServos()

    for i in range(0, 45, 1):
        epau.tilt(i)
        time.sleep(0.01)


    epau.resetServos()


def testUsbCamera():
    # cap = cv2.VideoCapture(1)
    frame = None
    cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
    while True:
        if cap.isOpened():
            ret, frame = cap.read()
            # cv2.imshow("Pan-Tilt Face Tracking", frame)
            # cv2.waitKey(1)


if __name__ == '__main__':
    testUsbCamera()