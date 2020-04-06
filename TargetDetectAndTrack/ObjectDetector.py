from abc import ABC, abstractmethod 
import cv2
import time
import numpy as np

class Detector(ABC):
    def detect(self):
        pass


class CascadeDetector(Detector):
    def __init__(self):
        # Face detector
        self.face_detector = cv2.CascadeClassifier("/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_default.xml")
        self.eye_detector = cv2.CascadeClassifier("/usr/local/share/opencv4/haarcascades/haarcascade_eye.xml")
        time.sleep(2.0)


    def detect(self, frame, frameCenter, sleep=0.0):
        # time.sleep(sleep)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_detector.detectMultiScale(gray, scaleFactor=1.05,
            minNeighbors=9, minSize=(30, 30),
            flags=cv2.CASCADE_SCALE_IMAGE)

        # faces = self.face_detector.detectMultiScale(gray, 1.3, 5)

        # check to see if a face was found
        if len(faces) > 0:
            
            (x, y, w, h) = faces[0]
            faceX = int(x + (w / 2.0))
            faceY = int(y + (h / 2.0))
            print("Found face")

            return ((faceX, faceY), faces[0])

            # eyes = self.eye_detector.detectMultiScale(gray[y:y+h, x:x+w])

            # if len(eyes) > 1:
            #     print("Found face")
            #     return ((faceX, faceY), faces[0])
            # else:
            #     print("ERROR: No face found!")
            #     return (frameCenter, None)
        else:
            print("ERROR: No face found!")
            return (frameCenter, None)

        