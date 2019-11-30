# Source code: https://www.pyimagesearch.com/2015/09/21/opencv-track-object-movement/
# Last modified by Kristian 18.09.19
        # Static coordinate system added
        # Angle calculation added
        # Code cleaned and simplified
                # Argument parser removed
                # Video file option removed, now always uses video stream
                # Moved a majority of code into functions
        #

from collections import deque
from imutils.video import VideoStream
import numpy as np
import cv2
import imutils
import time

# define the lower and upper boundaries of the "blue"
# disk in the HSV color space
blueLower = (110,50,50)
blueUpper = (130,255,255)
 
# initialize the list of tracked points, the frame counter,
# and the coordinate deltas
pts = deque(maxlen=32)
counter = 0
(dX, dY) = (0, 0)
direction = ""
aPoint = np.array([0, 0])
bVector = np.array([0, 0])
start = 0
end = 0
velocity = 0
displaceDX = 0
displaceDY = 0
timeArray = []

def resizeBlurHSV(frame):
        # resize the frame, blur it, and convert it to the HSV
	# color space
	
        frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        return hsv, frame

def createMask(hsv):
        # construct a mask for the color "blue", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        
        mask = cv2.inRange(hsv, blueLower, blueUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        return mask

def findCountour(mask):
        # find contours in the mask and initialize the current
        # (x, y) center of the disk
        
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        return cnts

def findLCountour(cnts):
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        
        objectContour = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(objectContour)
        return objectContour, radius, x, y

def computeCentroid(objectContour):
        # compute the centroid of the largest contour in the mask
        
        m = cv2.moments(objectContour)
        objectCenter = (int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"]))
        return objectCenter

def drawCircles(frame, x, y, radius, objectCenter, pts):
        # draw the circle and centroid on the frame,
        # then update the list of tracked points
        
        cv2.circle(frame, (int(x), int(y)), int(radius),
                (0, 255, 255), 2)
        cv2.circle(frame, objectCenter, 5, (0, 0, 255), -1)
        pts.appendleft(objectCenter)

def calculateAngle(dX, dY):
        bVector[0] = dX                                 # Prepare to make construct a displacement vector out of dX and Dy.
        bVector[1] = dY
        length2 = np.sqrt(np.square(bVector[0]) + np.square(bVector[1]))        # Create displacement vector.
        angle = np.arccos(dX / length2)                                         # Cosine of angle = adjacent/hypotenuse <=> angle = arccos(dX/length2).
        angle = angle * (180/np.pi)                                             # Convert angle from radians to degrees.
        print ("Angle: ", angle)

##def calculateDisplacement():
##        start = time.time()
##        displaceDX = pts[i][0] - pts[i-1][0]            # Calculate displacement in x direction.
##        displaceDY = pts[i][1] - pts[i-1][1]            # Calculate displacement in y direction.
##        end = time.time()
##
##def calculateVelocity(i):
##        timeArray.append(end-start)
##        print("Appended to timeArray: ", timeArray[i])
##        if(i > 2):
##                print("i == 2")
##                velocity = (np.sqrt(np.square(displaceDX) + np.square(displaceDY))) / timeArray[i] - timeArray[i-1]
##
##        elif(velocity <= 0):
##                print("Current i is: ", i)
##                print("Velocity is less than 0: 0")
##                print("Displacement: ", np.sqrt(np.square(displaceDX) + np.square(displaceDY)))
##        else:
##                print("Velocity else: ", velocity)

def calculateDirection():
        (dirX, dirY) = ("", "")

        trackdX = pts[-10][0] - pts[i][0]
        trackdY = pts[-10][1] - pts[i][1]

        # ensure there is significant movement in the
        # x-direction
        if np.abs(trackdX) > 20:
                dirX = "East" if np.sign(trackdX) == 1 else "West"

        # ensure there is significant movement in the
        # y-direction
        if np.abs(trackdY) > 20:
                dirY = "North" if np.sign(trackdY) == 1 else "South"

        # handle when both directions are non-empty
        if dirX != "" and dirY != "":
                direction = "{}-{}".format(dirY, dirX)

        # otherwise, only one direction is non-empty
        else:
                direction = dirX if dirX != "" else dirY
        return direction

# grab reference to the webcam
vs = VideoStream(src=0).start()
 
# allow the camera or video file to warm up
time.sleep(2.0)
# keep looping
while True:
        # grab the current frame
        frame = vs.read()
 
        # handle the frame from the VideoStream
        #frame = frame
 
        hsv, frame = resizeBlurHSV(frame)
        mask = createMask(hsv)
        cnts = findCountour(mask)
        
        # only proceed if at least one contour was found
        if len(cnts) > 0:
                objectContour, radius, x, y = findLCountour(cnts)
                objectCenter = computeCentroid(objectContour)
 
                # only proceed if the radius meets a minimum size
                if radius > 10:
                        drawCircles(frame, x, y, radius, objectCenter, pts)

        # loop over the set of tracked points
        for i in np.arange(1, len(pts)):
                # if either of the tracked points are None, ignore
                # them
                if pts[i - 1] is None or pts[i] is None:
                        continue
 
                # check to see if enough points have been accumulated in
                # the buffer
                if counter >= 10 and i == 1 and pts[-10] is not None:
                        dX = 0 + pts[i][0]                              # Coordinate system starts at x = 0, i.e. left end of frame.
                        dY = 442 - pts[i][1]                            # Coordinate system starts top of frame (by default). Therefore, top is 442, anything below is 442 - something.

                        calculateAngle(dX, dY)
                        #calculateDisplacement()
                        direction = calculateDirection()
                                
                        # otherwise, compute the thickness of the line and
                        # draw the connecting lines
                        thickness = int(np.sqrt(32 / float(i + 1)) * 2.5)
                        cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

        #calculateVelocity(i)
        
        # show the movement deltas and the direction of movement on
        # the frame
        cv2.putText(frame, direction, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.65, (0, 0, 255), 3)
        cv2.putText(frame, "dX: {}, dY: {}".format(dX, dY),
                (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                0.35, (0, 0, 255), 1)
 
        # show the frame to our screen and increment the frame counter
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF
        counter += 1
 
        # if the 'q' key is pressed, stop the loop
        if key == ord("q"):
                break
 
# stop the camera video stream
vs.stop()
 
# close all windows
cv2.destroyAllWindows()
