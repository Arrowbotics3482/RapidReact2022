# trying to use CV

import cv2 as cv
from matplotlib import pyplot as plt

CAMERA_ID = 1

#connecting to camera
#cap = cv.VideoCapture(0)
#ret, frame = cap.read()
#plt.imshow(frame)
#cap.release()

def take_photo():
    cap = cv.VideoCapture(CAMERA_ID)
    ret, frame = cap.read()
    cv.imwrite('webcamphoto.jpg', frame)
    cap.release()

cap = cv.VideoCapture(CAMERA_ID)
while cap.isOpened():
    ret, frame = cap.read()

    cv.imshow('Webcam', frame)
    if cv.waitKey(1) & 0xff == ord('q'):
        break
cap.release()
cv.destroyAllWindows()
cap = cv.VideoCapture(CAMERA_ID)
cap.isOpened()
cap.release()

