import argparse
import imutils
import cv2
import sys
import numpy as np

image = cv2.imread("pre-game board.png")


''' aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters_create()

# Detect the markers.
corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(resized_down,aruco_dict,parameters=parameters)

out = cv2.aruco.drawDetectedMarkers(resized_down, corners, ids)

print(corners)

cv2.imshow("arUco detection",out)
cv2.waitKey(0)
cv2.destroyAllWindows() '''

"""Detecting the chess board"""

# Setup SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()

# Change thresholds
params.minThreshold = 50;
params.maxThreshold = 15000;

# Filter by Area.
params.filterByArea = True
params.minArea = 30
params.maxArea = 800 

# Filter by Circularity
params.filterByCircularity = True
params.minCircularity = 0.7
params.maxCircularity = 1

#Filter by Color
params.filterByColor = False

# Filter by Convexity
params.filterByConvexity = False

# Filter by Inertia
params.filterByInertia = False

# Create a detector with the parameters
ver = (cv2.__version__).split('.')
if int(ver[0]) < 3 :
	detector = cv2.SimpleBlobDetector(params)
else :
	detector = cv2.SimpleBlobDetector_create(params)


img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)



# Detect blobs.
keypoints = detector.detect(img_gray)

cordinate = []

number = 0
for keypoint in keypoints:
    pixelX = int(keypoint.pt[0])
    pixelY = int(keypoint.pt[1])
    mXcv = pixelX * 0.000826086957
    mYcv = pixelY * 0.000826086957
    mXcv = float("{:.4f}".format(mXcv))
    mYcv = float("{:.4f}".format(mYcv))
    mXurx = 0.89517 - mYcv
    mYurx = 0.22763 - mXcv
    cordTup = (mXurx,mYurx)
    cordinate.append(cordTup)
    number += 1
    print (number, cordTup)

imageCircle = img_gray.copy()
circle_center1 = (405, 194)
radius = 10
cv2.circle(imageCircle, circle_center1, radius, (255, 255, 255), 3, cv2.LINE_AA) #white
cv2.imshow("image Circle", imageCircle)
cv2.waitKey(0)

# Draw detected blobs as red circles.
# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
im_with_keypoints = cv2.drawKeypoints(img_gray, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

# Show keypoints
cv2.imshow("Keypoints", im_with_keypoints)
cv2.waitKey(0)


cv2.destroyAllWindows()
