import cv2
import numpy as np
import time

def nothing(x):
    pass

timeList = []
counter = 0
timeTotal = 0

device = cv2.VideoCapture(0)
cv2.namedWindow("Trackbars")

cv2.createTrackbar("UBlue", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("UGreen", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("URed", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("LBlue", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("LGreen", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("LRed", "Trackbars", 0, 255, nothing)

while True:
    start = time.time()
    ret, frame = device.read()
    
    uBlue = cv2.getTrackbarPos("UBlue", "Trackbars")
    uGreen = cv2.getTrackbarPos("UGreen", "Trackbars")
    uRed = cv2.getTrackbarPos("URed", "Trackbars")
    lBlue = cv2.getTrackbarPos("LBlue", "Trackbars")
    lGreen = cv2.getTrackbarPos("LGreen", "Trackbars")
    lRed = cv2.getTrackbarPos("LRed", "Trackbars")

    lowerRange = np.array([lBlue,lGreen,lRed])
    upperRange = np.array([uBlue,uGreen,uRed])

    mask = cv2.inRange(frame, lowerRange, upperRange)
    cv2.imshow("Frame",frame)

    cv2.imshow("Mask",mask)

    end = time.time()
    timeList.append(end-start)
    timeTotal += end-start
    
    counter+= 1

    if(counter > 30):
        fps = counter / timeTotal
        print(fps)
        counter = 0
        timeTotal = 0

    if cv2.waitKey(1) == 27:
        break
    
device.release()
cv2.destroyAllWindows()

