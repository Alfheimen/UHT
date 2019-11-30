# Source code: https://www.pyimagesearch.com/2015/09/21/opencv-track-object-movement/
# Modified by Kristian 18.09.19
        # Static coordinate system added
        # Angle calculation added
        # Code cleaned and simplified
                # Argument parser removed
                # Video file option removed, now always uses video stream
                # Moved a majority of code into functions
        # Started velocity detection (doesn't work at all)

# Modified by Kristian 29.09.19
    # Displacement calculation in pixels added.
    # Velocity calculation in pixels per second added.
        # Needs testing to verify.
            # Testing indicates it works correctly, but difficult to truly tell.

# Modified by Kristian 16.10.19
    # Code has been slightly optimized
        # resizeBlurHSV function removed. Now only converts BGR to HSV.
        # createMask function removed; no longer erodes or dilates.
        
# Modified by Kristian 29.10.19
    # Added serial reading and writing to facilitate the interaction between RP4 and Arduino
        # Implemented as functions "fromSerial" and "toSerial"
            # toSerial protocol: send position, angle, velocity, in that order.
            # fromSerial protocol: to be decided

# Modified by Kristian 06.11.19
    # Fixed crash bug when puck goes untracked (i.e. when it goes offscreen, or isn't initially present)
    # Started implementing trajectory calculation as function "calculateTrajectory".
        # No idea if this really works atm. Not sure how to test this.
    # Program now outputs all the information directly to the frame, making it more readable for humans.
    
# Last modified by Kristian 13.11.19
    # Removed calls to functions angleCalculation and velocityCaculation. Currently not in-use.
    # Impemented function trajectoryCalculation; outputs the X-coordinate of the intersect vector with the X-axis where Y = 0.
        # Test with calculating by hand and seeing if program outputs same X-coordinate.
                
from collections    import deque
from imutils.video  import VideoStream, FPS
import numpy as np
import cv2
import imutils
import time
import serial
 
# Define the lower and upper boundaries of the "blue"
# disk in the HSV color space
blueLower = (110,50,50)
blueUpper = (130,255,255)

#bgrBlueLower = (23, 0, 0)
#bgrBlueUpper = (255, 16, 40)
 
# Initialize variables
pts         = deque(maxlen=32)
counter     = 0
(dX, dY)    = (0, 0)
direction   = ""
bVector     = np.array([0,0])
disX        = 0
disY        = 0
position    = []
timePos     = []
timeTotal   = 0
displacement = 0
velocity    = 0
fps         = 0
angle       = 0
x           = 0
oldX        = 0
lowerGoalBoundX = 62
upperGoalBoundX = 151
#ser = serial.Serial("/dev/ttyACM0", 9600)
#ser.flushInput()

def findCountour(mask):
    # Find contours in the mask and initialize the current
    # (x, y) center of the disk
    
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts    = imutils.grab_contours(cnts)
    center  = None
    return cnts

def findLCountour(cnts):
    # Find the largest contour in the mask, then use
    # it to compute the minimum enclosing circle
    
    objectContour = max(cnts, key=cv2.contourArea)
    ((x, y), radius) = cv2.minEnclosingCircle(objectContour)
    return objectContour, radius, x, y

def computeCentroid(objectContour):
    # Compute the centroid of the largest contour in the mask
    
    m = cv2.moments(objectContour)
    if(m["m00"] != 0):  # Prevents division by zero and program crashing.
        objectCenter = (int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"]))
        return objectCenter
    else: 
        return

def drawCircles(frame, x, y, radius, objectCenter, pts):
    # Draw the circle and centroid on the frame,
    # then update the list of tracked points
    
    cv2.circle(frame, (int(x), int(y)), int(radius),
        (0, 255, 255), 2)
    cv2.circle(frame, objectCenter, 5, (0, 0, 255), -1)
    pts.appendleft(objectCenter)

def calculateAngle(dX, dY):
    bVector[0]  = dX        # Prepare to construct a displacement vector out of dX and dY.
    bVector[1]  = dY
    length2     = np.sqrt(np.square(bVector[0]) + np.square(bVector[1]))        # Create displacement vector.
    angle       = np.arccos(dX / length2)                                       # Cosine of angle = adjacent/hypotenuse <=> angle = arccos(dX/length2).
    angle       = angle * (180/np.pi)                                           # Convert angle from radians to degrees.
    return angle

def drawTrailingLines():        
    # compute the thickness of the line and
    # draw the connecting lines
    
    thickness = int(np.sqrt(32 / float(i + 1)) * 2.5)
    cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)


def calculateDisplacement():
    # Using the list of tracked points, calculate its position delta
    displacement = position[len(position)-1] - position[len(position)-2]

    if(abs(displacement) > 10):
        return displacement
    else:
        return 0

def calculateVelocity():
    # Velocity = displacement / delta time
    velocity = displacement / timePos[len(timePos)-1] - timePos[len(timePos)-2]

    if(abs(velocity) > 10):
        return velocity
    else:
        return 0
    
def toSerial(position, angle, velocity):
    ser.write(position)
    ser.write(angle)
    ser.write(velocity)

def fromSerial():
	if(ser.inWaiting() > 0):
		line = ser.readline()
		print(line)
        
def calculateTrajectory():
    deltaX = (pts[0][0]-178) - (pts[1][0]-178)
    deltaY = pts[0][1] - pts[1][1]
    
    if(deltaX != 0):                            # Ensure no division by zero
        m = deltaY / deltaX                     # Find slope
        if(m != 0):                             # Ensure no division by zero
            x = (pts[0][1] - pts[1][1]) / m     # Calculate X-coordinate of intersection where Y = 0
            if(abs(x) > 5):                     # Ensure there is enough change (to prevent fluctuating values).
                return x
    
    # Math theory:
    # y-y1 = m(x-x1)
    # y=m(x-x1)+y1                          # Point-slope formula
    # y=m(x-pts[0][1]) + pts[1][1]          # Re-write for program deque
    # y=mx-mpts[0][1] + pts[1][1]           # Multiply m in
    
    # Goal will always be at Y = 0:
    # (y - pts[1][1] + mpts[0][1]) / m = x  # Solve for X where Y = 0
    # (0 - pts[1][1] + mpts[0][1]) / m = x
    # x = (pts[0][1] - pts[1][1]) / m
    # This is the X-coordinate where the vector intersects with X-axis
    
# Grab reference to the webcam
vs = VideoStream(src=0, resolution=(640, 480)).start()

# Allow the camera to warm up
time.sleep(2.0)

# Keep looping
while True:
    # Grab the current frame 
    start = time.time()
    frame = vs.read()

    # Convert to HSV color space; BGR very sensitive to light under testing.
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Create a mask with the defined boundaries for blue. inRange will go through each pixel and
    # check if it falls within this threshold. If it does, makes it white (all 255), if not,
    # makes it black (all 0)
    mask = cv2.inRange(hsv, blueLower, blueUpper)
    cv2.imshow("mask", mask)
    
    cnts = findCountour(mask)

    # Only proceed if at least one contour was found
    if (len(cnts) > 0):
        objectContour, radius, x, y = findLCountour(cnts)                   # In the array of countours, find the largest one.
        objectCenter                = computeCentroid(objectContour)

        # Only proceed if the radius meets a minimum size
        if (radius > 10):
            drawCircles(frame, x, y, radius, objectCenter, pts)

    # Loop over the set of tracked points
    for i in np.arange(1, len(pts)):
        
        # If either of the tracked points are None, ignore them
        if (pts[i - 1] is None or pts[i] is None): continue

        # Check to see if enough points have been accumulated in the buffer
        if (counter >= 10 and i == 1 and len(pts) > 9):
            # set coordinate system
            dX = pts[i][0] - 178
            dY = 480 - pts[i][1]

            #angle = calculateAngle(dX, dY)
            #print("Angle: ", angle)

        drawTrailingLines()
    
    # Keep track of object position for use in displacement and velocity calculation
    if (len(pts) > 2):
        oldX = x
        x = calculateTrajectory()
        
        if (x != None):         # Only update X if new X != None
            oldX = x
        
        positionX = pts[1 - len(pts)][0]
        positionY = pts[1 - len(pts)][1]
        position.append(np.sqrt(np.square(positionX) + np.square(positionY)))
        
        if ( lowerGoalBoundX < oldX < upperGoalBoundX ):   # Determine if x is within goal boundaries
            print("Moving to intercept!")

    # Show the movement deltas on the frame
    cv2.putText(frame, "dx: {}, dy: {}".format(dX, dY),
    (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
    0.35, (0, 0, 255), 1)
        
    # Show displacement on the frame
    cv2.putText(frame, "Displacement: {}".format(displacement),
    (10, frame.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX,
    0.35, (0, 0, 255), 1)
    
    cv2.putText(frame, "x interesect: {}".format(oldX),
    (10, frame.shape[0] - 50), cv2.FONT_HERSHEY_SIMPLEX,
    0.35, (0, 0, 255), 1)
    
    # # Show velocity on the frame
    # cv2.putText(frame, "Velocity: {}".format(velocity),
    # (10, frame.shape[0] - 50), cv2.FONT_HERSHEY_SIMPLEX,
    # 0.35, (0, 0, 255), 1)
    
    # # Show angle on the frame
    # cv2.putText(frame, "Angle: {}".format(angle),
    # (10, frame.shape[0] - 70), cv2.FONT_HERSHEY_SIMPLEX,
    # 0.35, (0, 0, 255), 1)
    
    # Show FPS on the frame
    cv2.putText(frame, "FPS: {}".format(fps),
    (10, frame.shape[0] - 90), cv2.FONT_HERSHEY_SIMPLEX,
    0.35, (0, 0, 255), 1)
                
    # Show the frame to our screen and increment the frame counter
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

    end = time.time()
    timePos.append(end-start)
    timeTotal += end-start

    # Ensure enough positions have been stored before caclulating displacement and velocity (because y2-y1, x2-x1)
    if(len(position) > 2):
        displacement = calculateDisplacement()
        # velocity = calculateVelocity()
        
    counter += 1

    if(counter > 30):
        fps = counter / timeTotal
        counter = 0
        timeTotal = 0

    # If the 'q' key is pressed, stop the loop
    if key == ord("q"):
            break
        
# Stop the video stream
vs.stop()
 
# Close all windows
cv2.destroyAllWindows()
