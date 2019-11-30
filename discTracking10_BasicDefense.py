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

# Modified by Kristian 06.11.19
    # Fixed crash bug when puck goes untracked (i.e. when it goes offscreen, or isn't initially present)
    # Started implementing trajectory calculation as function "calculateTrajectory".
        # No idea if this really works atm. Not sure how to test this.
    # Program now outputs all the information directly to the frame, making it more readable for humans.
    
# Modified by Kristian 13.11.19
    # Removed calls to functions angleCalculation and velocityCaculation. Currently not in-use.
    # Impemented function trajectoryCalculation; outputs the X-coordinate of the intersect vector with the X-axis where Y = 0.
        # Test with calculating by hand and seeing if program outputs same X-coordinate.

# Last modified by Kristian 20.11.19
    # Fixed trajectory calculation to fit new coordinate system.
    # Added calculateTrajetoryByHand() function with pre-determined points for testing.
        # Can now check result of this function against standard function,as well as
        # doing it by hand, to ensure program outputs correct coordinate.
    # Cleaned up program, improved readability.
                
from collections    import deque
from imutils.video  import VideoStream, FPS
import numpy as np
import cv2
import imutils
import time
import serial
 
# Define the lower and upper boundaries of the "blue"
# disk in the HSV color space
blueLower = (77,91,107)
blueUpper = (151,255,232)

#bgrBlueLower = (23, 0, 0)
#bgrBlueUpper = (255, 16, 40)
 
# Initialize variables
pts             = deque(maxlen=32)      # Keeps track of object points
counter         = 0                     # Frame counter (used in FPS)
(dX, dY)        = (0, 0)                # Initialize coordinate system
direction       = ""                    # Direction of object
bVector         = np.array([0,0])       # Vector used in angle calculation
position        = []                    # Position vector used in velocity calculation
timePos         = []                    # Array to keep track of time
timeTotal       = 0                     # Total frame computation time (used in FPS)
displacement    = 0                     # Displacement of object
velocity        = 0                     # Velocity of object
fps             = 0                     # FPS of program
angle           = 0                     # Angle of object                   
yIntersect      = 0                     # Computer goal intersect coordinate
oldY            = 0                     # Old of the above
lowerGoalBoundY = 104                   # Lower boundary y coordinate of computer goal
upperGoalBoundY = 224                   # Upper boundary y coordinate of computer goal
ser = serial.Serial("/dev/ttyACM0", 115200)
ser.flushInput()
ser.timeout = 0
toArduino = 0

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
    # Compute the thickness of the line and draw the connecting lines
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
    
def toSerial(toArduinoEncoded):
    print("Sending to Arduino...")
    ser.flushInput()
    ser.write(toArduinoEncoded)

def fromSerial():
	if(ser.inWaiting() > 0):
		line = ser.readline()
		print(line)
        
def calculateTrajectory():
    # Assign the newest and second newest points to more readable variables
    # Ensure one coordinate is float type to make slope division float (in case of slope < 1)
    x1 = pts[0][0] - 47
    x1 = float(x1)
    x2 = pts[1][0] - 47
    y1 = 421 - pts[0][1]
    y2 = 421 - pts[1][1]
    
    # Calculate the deltas to be used when finding the slope
    deltaX = x2 - x1
    deltaY = y2 - y1
    print("x1: ", x1,"y1: ", y1)
    print("x2: ", x2,"y2: ", y2)
    print("Delta x: ", deltaX)
    print("Delta Y: ", deltaY)
    
    if(deltaX != 0):                          # Ensure no division by zero
        slope = deltaY / deltaX               # Calculate the slope of the line              
        print("slope: ", slope)
        if(slope != 0):                       # Ensure no division by zero
            yIntersect = -slope * x1 + y1
            print("yIntersect: ",yIntersect)
            if(abs(yIntersect) > 5):          # Only return the point if it's large enough
                return yIntersect
                
def calculateTrajectoryByHand():
    # Tracjetory calculation with pre-determined points for testing
    x1 = 92
    x1 = float(x1)
    y1 = 126
    x2 = 28
    y2 = 154
    
    deltaXh = x2 - x1
    deltaYh = y2 - y1
    slope = deltaYh / deltaXh
    intersect = -slope * x1 + y1
    
    print("Delta x: ", deltaXh)
    print("Delta Y: ", deltaYh)
    print("Slope: ", slope)
    print("Intersect: ", intersect)
    
# Grab reference to the webcam
vs = VideoStream(src=0, resolution=(640, 480)).start()

# Allow the camera to warm up
time.sleep(2.0)

string1 = "calib"
string1Encode = string1.encode()
ser.write(string1Encode)

# Keep looping
while True:
    # Grab the current frame 
    start = time.time()
    frame = vs.read()
    
    # Flip frame to better align with coordinate system
    frame = cv2.flip(frame, 1)      

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
        # In the array of countours, find the largest one.
        objectContour, radius, x, y = findLCountour(cnts)                   
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
            # Set coordinate system
            dX =  pts[i][0] - 47
            dY =  421 - pts[i][1]

        drawTrailingLines()
    
    # Only calculate trajectory if enough points have been gathered, and there is a big
    # enough difference between the two points
    if (len(pts) > 2 and abs(pts[0][0] - pts[1][0]) > 2 and abs(pts[0][1] - pts[1][1]) > 2):
        oldY = y
        yIntersect = calculateTrajectory()
        
        # Only update y if new y != None
        if (yIntersect != None):         
            oldY = yIntersect
        
        # Determine if y is within goal boundaries
        if(lowerGoalBoundY < oldY < upperGoalBoundY and pts[0][0] - pts[1][0] < 0):   
            print("Moving to intercept")
            oldY = str(oldY)
            toArduino = "def2," + oldY
            toArduinoEncoded = toArduino.encode()
            print("Encoded.")
            toSerial(toArduinoEncoded)
            print("Successfully sent!")

    # Show the movement deltas on the frame
    cv2.putText(frame, "dx: {}, dy: {}".format(dX, dY),
    (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
    0.35, (0, 0, 255), 1)
    
    # Show the y intersect point on the frame
    cv2.putText(frame, "y interesect: {}".format(oldY),
    (10, frame.shape[0] - 50), cv2.FONT_HERSHEY_SIMPLEX,
    0.35, (0, 0, 255), 1)
    
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
