import keyboard
import cv2
import urx
# from IPython import embed
import logging
import traceback
import argparse
import imutils
import sys
import numpy as np
# from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper

'''
def grab_piece(x, y):
    rob.movel(x, y, above_piece, rx, ry, rz)
    rob.movel(x, y, at_piece, rx, ry, rz)
    gripper.close_gripper()
    rob.movel(x, y, above_piece, rx, ry, rz)

def place_piece(x, y):
    rob.movel(x, y, above_step, rx, ry, rz)
    rob.movel(x, y, at_step, rx, ry, rz)
    gripper.open_gripper()
    rob.movel(x, y, above_step, rx, ry, rz)
'''

while True:
    keyboard.wait("space")
    """rob.movel(pos_dice_grab, 0.5, 0.3)
        gripper.close_gripper()
        rob.movel(pos_dice_drop, 0.5, 0.3)
        gripper.open_gripper()"""

    cam = cv2.VideoCapture(4)
    # rob.movel(pos_dice_pic, 0.5, 0.3)
    while True:
        ret, frame = cam.read()
        if not ret:
            print("failed to grab frame")
            break
        cv2.imshow("test", frame)

        k = cv2.waitKey(1)
        if k % 256 == 32:
            # SPACE pressed
            img_name = "dice_number.png"
            cv2.imwrite(img_name, frame)
            print("{} written!".format(img_name))
            break

    """Detecting the dice"""
    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    params.minThreshold = 50;
    params.maxThreshold = 15000;

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 50
    params.maxArea = 400

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.8
    params.maxCircularity = 1

    # Filter by Color
    params.filterByColor = False

    # Filter by Convexity
    params.filterByConvexity = False

    # Filter by Inertia
    params.filterByInertia = False

    # Create a detector with the parameters
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3:
        detector = cv2.SimpleBlobDetector(params)
    else:
        detector = cv2.SimpleBlobDetector_create(params)

    image = cv2.imread("dice_number.png")
    img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Detect blobs.
    keypoints = detector.detect(img_gray)

    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(img_gray, keypoints, np.array([]), (0, 0, 255),
                                          cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Show keypoints
    cv2.imshow("Keypoints", im_with_keypoints)

    print(len(keypoints))
    cv2.waitKey(0)

    # Move the robot to pos_board
    #rob.movel[pos_board,0.3,0.3]

    # Make the robot to take the picture of chess board
    cam = cv2.VideoCapture(4)
    cv2.namedWindow("board")
    while True:
        ret, frame = cam.read()
        if not ret:
            print("failed to grab frame")
            break
        cv2.imshow("test", frame)

        k = cv2.waitKey(1)
        if k % 256 == 32:
            # SPACE pressed
            img_name = "board.png"
            cv2.imwrite(img_name, frame)
            print("{} written!".format(img_name))
            break
    image1 = cv2.imread("board.png")

    # Detect chess pieces
    """aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(resized_down,aruco_dict,parameters=parameters)
    out = cv2.aruco.drawDetectedMarkers(image1, corners, ids)
    
    markerCentres = []
    markerCenter_urx = []

    for marker in corners:
        centreX = (marker[0][0][0] + marker[0][2][0]) / 2
        centreY = (marker[0][0][1] + marker[0][2][1]) / 2
        markerCoord = []
        markerCoord.append(centreX)
        markerCoord.append(centreY)
        markerCentres.append(markerCoord)

    print(markerCentres)

    cv2.imshow("out",out)
    cv2.waitKey(0)"""

    # Grab_the_pieces
    """for markerTup in markerCenters:
        pixelX = int(markerTup[0])
        pixelY = int(markerTup[1])
        mXcv = pixelX * 0.000264583333
        mYcv = pixelY * 0.000264583333
        mXcv = float("{:.4f}".format(mXcv))
        mYcv = float("{:.4f}".format(mYcv))
        mXurx = 0.71140 - mYcv
        mYurx = 0.58306 - mXcv
        markerTup[0] = mXurx
        markerTup[1] = mYurx
        markerCenter_urx.append(markerTup)"""

    # To detect whether the dice number is odd or even
    """
    numAtStart = 0
    for markerCenter in markerCenter_urx:
        for start in startingPts:
            if markerCenter[0] == start[0] and markerCenter[1] == start[1]:
                numAtStart += 1
    
    if dice_number % 2 == 1:
        if numAtStart == 4:
            break
        else:
            for step in steps:
                if markerCenter_urx[0][0] == steps[step][0] and markerCenter_urx[0][1] == steps[step][1]:
                    next_step_num = step + dice_number
                    for nextStep in steps:
                        if next_step_num == nextStep:
                            grab_piece(markerCenter_urx[0][0], markerCenter_urx[0][1])
                            place_piece(steps[nextStep][0], steps[nextSTep][1])
    else:
        if numAtStart == 4:
            next_step_num(e)
        else:
            rng between (0, 1)
            if rng == 0:
                move one new piece out
            else:
                move the furthest piece(improve later)"""
