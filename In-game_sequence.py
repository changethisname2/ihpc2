"""Press space to initiate the robot
Make the robot move to the container, shake container(pos_roll), take photo (pos_dice) and detect number on dice
Robot move to pos_board, take photo detect its own pieces
Write conditional statement, if odd and all pieces start, break. Else, move. If decide between moving piece out of start or continue moving another piece, do 50/50, write algorithm later, after moving piece, it will sleep
everytime it is its turn, make robot take photo of ludo board"""
import keyboard
import cv2
import urx
#from IPython import embed
import logging
import traceback
import argparse
import imutils
import sys
import numpy as np
while True:
    if keyboard.read_key() == "r":
        pass
        #Make robot move to dice(pos_roll)
        """pose_dice = (x,y,z,rx,ry,rz)
        rob.movel[pose_dice,0.2,0.3]"""

        # Roll the dice

        #Make robot move to camera position (pos_dice)
        cam = cv2.VideoCapture(4)
        cv2.namedWindow("dice")
        ret, frame = cam.read()
        if not ret:
            print("failed to grab frame")
            break
        cv2.imshow("dice", frame)
        img_dice = "dice_number.png"
        cv2.imwrite(img_dice, frame)
        print("{} written!".format(img_dice))
        cam.release()
        image = cv2.imread("dice_number.png")
        # Detecting the dice number
        """ #Setup SimpleBlobDetector parameters.
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

        img_dice = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        keypoints = detector.detect(img_dice)
        print(len(keypoints))"""

        #Move the robot to pos_board
        """pose_board = (x',y',z',rx',ry',rz')
        rob.movel[pose_board,0.2,0.3]"""

        #Make the robot to take the picture of chess board
        cam = cv2.VideoCapture(4)
        cv2.namedWindow("board")
        ret, frame = cam.read()
        if not ret:
            print("failed to grab frame")
            break
        cv2.imshow("board", frame)
        img_board = "board.png"
        cv2.imwrite(img_board, frame)
        print("{} written!".format(img_board))
        cam.release()
        image1 = cv2.imread("board.png")

        #Detect chess pieces
        """aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(resized_down,aruco_dict,parameters=parameters)
        
        out = cv2.aruco.drawDetectedMarkers(image1, corners, ids)
        
        

        markerCentres = []
        
        count = 0
        
        for marker in corners:
            centreX = (marker[0][0][0] + marker[0][2][0]) / 2
            centreY = (marker[0][0][1] + marker[0][2][1]) / 2
            markerList = []
            markerList.append(centreX)
            markerList.append(centreY)
            markerList.append(ids([count]))
            markerCentres.append(markerList)
            count += 1
        
        print(markerCentres)
        
        cv2.imshow("out",out)
        cv2.waitKey(0)"""

        #Grab_the_pieces
        """for markerTup in markerCenters:
            pixelX = int(markerTup[0])
            pixelY = int(markerTup[1])
            mXcv = pixelX * 0.000264583333
            mYcv = pixelY * 0.000264583333
            mXcv = float("{:.4f}".format(mXcv))
            mYcv = float("{:.4f}".format(mYcv))
            mXurx = 0.8828 - mYcv
            mYurx = 0.0450 - mXcv
            markerTup[0] = mXurx
            markerTup[1] = mYurx
            markerCenter_urx = []
            markerCenter_urx.append(markerTup)"""

        # To detect whether the dice number is odd or even
        """if len(keypoints) % 2 == 1:
             if all pieces at start:
                 break
             else:
                 move the furthest piece(improve later)
        else:
            if all pieces at start:
                move the first one
            else:
                rng between (0, 1)
                if rng == 0:
                    move one new piece out
                else:
                    move the furthest piece(improve later)"""
