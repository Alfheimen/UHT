import cv2
import numpy as np
import time

timeList = []
counter = 0
timeTotal = 0

device = cv2.VideoCapture(0)

while True:
    start = time.time()
    ret, frame = device.read()
    
    cv2.imshow("Frame",frame)
    
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

