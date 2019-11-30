#Source code: https://www.youtube.com/watch?v=WKKRt27WJ0w
#last edited by Alina 11.09.19

import cv2
import numpy as np

device = cv2.VideoCapture(0)
while True:
    ret, frame = device.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lowerRange = np.array([110,50,50])
    upperRange = np.array([130,255,255])

    mask = cv2.inRange(hsv, lowerRange, upperRange)
    cv2.imshow("Frame",frame)

    result = cv2.bitwise_and(frame,frame,mask=mask)
    cv2.imshow("Result",result)


    if cv2.waitKey(1) == 27:
        break

device.release()
cv2.destroyAllWindows()
