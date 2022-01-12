"""For marker detction"""
# import the necessary packages
import argparse
import imutils
import cv2
import sys
import numpy as np

image = cv2.imread("opencv_frame_0.png")


h,w,c = image.shape
ratio = int(h) / int(w)

# let's downscale the image using new  width and height
down_width = 600
down_height = int(down_width * ratio)
down_points = (down_width, down_height)
resized_down = cv2.resize(image, down_points, interpolation=cv2.INTER_LINEAR)

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


h,w = img_gray.shape
ratio = int(h) / int(w)

# let's upscale the image using new  width and height
up_width = 600
up_height = int(up_width * ratio)
up_points = (up_width, up_height)
resized_up = cv2.resize(img_gray, up_points, interpolation=cv2.INTER_LINEAR)
cv2.waitKey(0)


h,w,c = image.shape


# Detect blobs.
keypoints = detector.detect(resized_up)

cordinate = []


for keypoint in keypoints:
    pixelX = int(keypoint.pt[0])
    pixelY = int(keypoint.pt[1])
    mXcv = pixelX * 0.000264583333
    mYcv = pixelY * 0.000264583333
    mXcv = float("{:.4f}".format(mXcv))
    mYcv = float("{:.4f}".format(mYcv))
    mXurx = 0.8828 - mYcv
    mYurx = 0.0450 - mXcv
    cordTup = (mXcv,mYcv)
    cordinate.append(cordTup)
    print (cordTup)
#print (cordinate)

imageCircle = resized_up.copy()
circle_center = (29, 31)
radius = 10
cv2.circle(imageCircle, circle_center, radius, (255, 255, 255), 3, cv2.LINE_AA)
cv2.imshow("image Circle", imageCircle)
cv2.waitKey(0)

# Draw detected blobs as red circles.
# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
im_with_keypoints = cv2.drawKeypoints(resized_up, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

# Show keypoints
cv2.imshow("Keypoints", im_with_keypoints)
cv2.waitKey(0)

steps = {
	1 : cordinate[21],
	2 : cordinate [22],
	3 : cordinate [23],
    4 : cordinate [24],
    5 : cordinate [25],
    6 : cordinate [19],
    7 : cordinate [17],
    8 : cordinate [11],
    9 : cordinate [10],
    10 : cordinate [3],
    11 : cordinate [2],
    12 : cordinate [0],
    13 : cordinate [1],
    14 : cordinate [4],
    15 : cordinate [9],
    16 : cordinate [12],
    17 : cordinate [18],
    18 : cordinate [20],
    19 : cordinate [26],
    20 : cordinate [27],
    21 : cordinate [28],
    22 : cordinate [29],
    23 : cordinate [30],
    24 : cordinate [67],
    25 : cordinate [68],
    26 : cordinate [70],
    27 : cordinate [45],
    28 : cordinate [44],
    29 : cordinate [43],
    30 : cordinate [42],
    31 : cordinate [41],
    32 : cordinate [47],
    33 : cordinate [49],
    34 : cordinate [55],
    35 : cordinate [57],
    36 : cordinate [63],
    37 : cordinate [66],
    38 : cordinate [65],
    39 : cordinate [64],
    40 : cordinate [62],
    41 : cordinate [56],
    42 : cordinate [54],
    43 : cordinate [48],
    44 : cordinate [46],
    45 : cordinate [40],
    46 : cordinate [39],
    47 : cordinate [38],
    48 : cordinate [37],
    49 : cordinate [36],
    50 : cordinate [69],
    51 : cordinate [72],
    52 : cordinate [31],
    53 : cordinate [32],
    54 : cordinate [33],
    55 : cordinate [34],
    56 : cordinate [35]
}

print(steps[42])

cv2.destroyAllWindows()
