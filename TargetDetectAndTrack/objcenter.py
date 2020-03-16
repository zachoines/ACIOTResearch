import cv2
import time

class ObjCenter:
    def __init__(self):
        # Face detector
        self.detector = cv2.CascadeClassifier(
        "/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_default.xml")
        time.sleep(2.0)


    def update(self, frame, frameCenter):

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        rects = self.detector.detectMultiScale(gray, scaleFactor=1.05,
            minNeighbors=9, minSize=(30, 30),
            flags=cv2.CASCADE_SCALE_IMAGE)

        # check to see if a face was found
        if len(rects) > 0:
            print("Found face")
            # extract the bounding box and center cords
            (x, y, w, h) = rects[0]
            faceX = int(x + (w / 2.0))
            faceY = int(y + (h / 2.0))

            return ((faceX, faceY), rects[0])
        else:
            return (frameCenter, None)

        