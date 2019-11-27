#!/usr/bin/python

# Standard imports
import cv2
import numpy as np;

# Read image
#im = cv2.imread("blob.jpg", cv2.IMREAD_GRAYSCALE)

# Start video capture
capture = cv2.VideoCapture(0)

# Setup SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()

# Change thresholds
params.minThreshold = 10
params.maxThreshold = 200


# Filter by Area.
params.filterByArea = True
params.minArea = 50

# Filter by Circularity
params.filterByCircularity = True
params.minCircularity = 0.1

# Filter by Convexity
params.filterByConvexity = True
params.minConvexity = 0.3
    
# Filter by Inertia
params.filterByInertia = True
params.minInertiaRatio = 0.01

# Create a detector with the parameters
ver = (cv2.__version__).split('.')
if int(ver[0]) < 3 :
	detector = cv2.SimpleBlobDetector(params)
else : 
	detector = cv2.SimpleBlobDetector_create(params)

# Range for lower red
lower_redl = np.array([0, 120, 70])
upper_redl = np.array([10, 255, 255])

# Range for upper red
lower_redu = np.array([170, 120, 70])
upper_redu = np.array([180, 255, 255])

while(True):
    
    # Draw frame
    ret, im = capture.read()
    
    # Converting from BGR to HSV color space
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    
    # Compute mask
    mask1 = cv2.inRange(hsv, lower_redl, upper_redl)
    mask2 = cv2.inRange(hsv, lower_redu, upper_redu)
    
    # Bitwise NOR for two ranges
    mask2 = cv2.bitwise_not(mask1 + mask2)
    
    # Bitwise AND 
    #res = cv2.bitwise_and(im, im, mask=mask2)
    

    # Detect blobs.

    keypoints = detector.detect(mask2)

    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
    # the size of the circle corresponds to the size of blob

    im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Show blobs
    cv2.imshow("Keypoints", im_with_keypoints)
    if cv2.waitKey(1) == 27:
        break
	
capture.release()
cv2.destroyAllWindows()

