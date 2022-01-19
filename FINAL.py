import keyboard
import cv2
import urx
from IPython import embed
import logging
import traceback
import argparse
import imutils
import sys
import numpy as np
from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper

'''Definitions'''
a, v = 0.5, 0.3
pos_dice_drop = [0.707, 0.219, 0.030, -2.205, 2.205, -0.043]
pos_dice_grab = [0.719, 0.219, -0.200, -2.205, 2.205, -0.043]
pos_dice_pic = [0.653, 0.187, -0.143, -2.189, 2.191, -0.011]
pos_board = [0.647, -0.078, -0.029, -2.125, 2.190, -0.108]
pos_boardj = [0.137, -1.985, -1.438, -1.176, 1.563, 0.166]

def grab_piece(x, y):
    above_piece, rxb, ryb, rzb = -0.1502, -2.1890, 2.1920, -0.0111
    at_piece, rx, ry, rz = -0.2025, -2.1890, 2.1910, -0.0110
    rob.movel([x, y, above_piece, rxb, ryb, rzb], a, v)
    rob.movel([x, y, at_piece, rx, ry, rz], a, v)
    gripper.close_gripper()
    rob.movel([x, y, above_piece, rx, ry, rz], a, v)

def place_piece(x, y):
    above_piece, rxb, ryb, rzb = -0.1502, -2.1890, 2.1920, -0.0111
    at_piece, rx, ry, rz = -0.2025, -2.1890, 2.1910, -0.0110
    rob.movel([x, y, above_piece, rxb, ryb, rzb], a, v)
    rob.movel([x, y, at_piece, rx, ry, rz],a,v)
    gripper.open_gripper()
    rob.movel([x, y, above_piece, rxb, ryb, rzb], a, v)

def move_piece(markerCenter_urx, steps, dice_num):
    for step in steps:
        if markerCenter_urx[0][0] == steps[step][0] and markerCenter_urx[0][1] == steps[step][1]: 
            marker_pos = step
    diff = (57 - marker_pos)
    if diff < 6:
        if diff < dice_num:
            key = 57 - (dice_num - diff)
            grab_piece(markerCenter_urx[0][0], markerCenter_urx[0][1])
            place_piece(steps[key][0], steps[key][1])
    else:
        for step in steps:
            if markerCenter_urx[0][0] == steps[step][0] and markerCenter_urx[0][1] == step[step][1]:
                next_step_num = step + dice_num
                for nextStep in steps:
                    if next_step_num == nextStep:
                        grab_piece(markerCenter_urx[0][0], markerCenter_urx[0][1])
                        place_piece(steps[nextStep][0], steps[nextStep][1])

if __name__ == "__main__":
    try:
        print("Connecting to Robot...")
        logging.basicConfig(level=logging.WARN)
        while True:
            try:
                print("...")
                rob = urx.Robot("192.168.1.6")
                rob.set_tcp((0, 0, 0.335, 0, 0, 0))
                rob.set_payload(0.5, (0, 0, 0))
                gripper = Robotiq_Two_Finger_Gripper(rob)
                break
            except KeyboardInterrupt:
                break
            except:
                try:
                    rob.close()
                except:
                    pass

        image = cv2.imread("pre-game board.png")

        # Setup SimpleBlobDetector parameters.()
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

        coordinate = []
        number = 1

        for keypoint in keypoints:
            pixelX = int(keypoint.pt[0])
            pixelY = int(keypoint.pt[1])
            mXcv = pixelX * 0.00062295
            mYcv = pixelY * 0.00062295
            mXcv = float("{:.4f}".format(mXcv))
            mYcv = float("{:.4f}".format(mYcv))
            mXurx = 0.8880 - mYcv
            mYurx = 0.1575 - mXcv
            cordTup = (mXurx,mYurx)
            coordinate.append(cordTup)
        endPts = (0.73487,-0.06897)

        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(img_gray, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # Show keypoints
        #cv2.imshow("Keypoints", im_with_keypoints)
        cv2.waitKey(0)

        cv2.destroyAllWindows()
        
        steps = {
            1 : coordinate [57],
            2 : coordinate [7],
            3 : coordinate [8],
            4 : coordinate [27],
            5 : coordinate [9],
            6 : coordinate [2],
            7 : coordinate [0],
            8 : coordinate [23],
            9 : coordinate [22],
            10 : coordinate [40],
            11 : coordinate [39],
            12 : coordinate [19],
            13 : coordinate [55],
            14 : coordinate [41],
            15 : coordinate [42],
            16 : coordinate [43],
            17 : coordinate [1],
            18 : coordinate [3],
            19 : coordinate [10],
            20 : coordinate [26],
            21 : coordinate [11],
            22 : coordinate [4],
            23 : coordinate [5],
            24 : coordinate [56],
            25 : coordinate [58],
            26 : coordinate [49],
            27 : coordinate [48],
            28 : coordinate [35],
            29 : coordinate [46],
            30 : coordinate [45],
            31 : coordinate [44],
            32 : coordinate [59],
            33 : coordinate [60],
            34 : coordinate [53],
            35 : coordinate [54],
            36 : coordinate [37],
            37 : coordinate [38],
            38 : coordinate [18],
            39 : coordinate [17],
            40 : coordinate [16],
            41 : coordinate [36],
            42 : coordinate [52],
            43 : coordinate [51],
            44 : coordinate [50],
            45 : coordinate [34],
            46 : coordinate [33],
            47 : coordinate [15],
            48 : coordinate [14],
            49 : coordinate [47],
            50 : coordinate [32],
            51 : coordinate [28],
            52 : coordinate [29],
            53 : coordinate [12],
            54 : coordinate [13],
            55 : coordinate [30],
            56 : coordinate [31],
            57 : endPts
        }
    
        #print(steps)
        startingPts = []
        startingPts.append(coordinate[24])
        startingPts.append(coordinate[25])
        startingPts.append(coordinate[21])
        startingPts.append(coordinate[20])

        """Rolling dice"""
        rob.movej(pos_boardj, a, v)
        rob.movel(pos_dice_drop, a, v)
        rob.movel(pos_dice_grab, a, v)
        gripper.close_gripper()
        rob.movel(pos_dice_drop, a, v)
        gripper.open_gripper()

        """Taking picture of dice"""
        cam = cv2.VideoCapture(4)
        rob.movel(pos_dice_pic, a, v)
        while True:
            ret, frame = cam.read()
            if not ret:
                print("failed to grab frame")
                break
            cv2.imshow("test", frame)

            k = cv2.waitKey(1)
            if k%256 == 32:
                # SPACE pressed
                img_name = "dice_number.png"
                cv2.imwrite(img_name, frame)
                print("{} written!".format(img_name))
                break
        cam.release()
        cv2.destroyAllWindows()


        """Detecting the dice"""
        # Setup SimpleBlobDetector parameters.
        dice_params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        dice_params.minThreshold = 50;
        dice_params.maxThreshold = 15000;

        # Filter by Area.
        dice_params.filterByArea = True
        dice_params.minArea = 50
        dice_params.maxArea = 400

        # Filter by Circularity
        dice_params.filterByCircularity = True
        dice_params.minCircularity = 0.8
        dice_params.maxCircularity = 1

        # Filter by Color
        dice_params.filterByColor = False

        # Filter by Convexity
        dice_params.filterByConvexity = False

        # Filter by Inertia
        dice_params.filterByInertia = False

        # Create a detector with the parameters
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3 :
            detector = cv2.SimpleBlobDetector(dice_params)
        else :
            detector = cv2.SimpleBlobDetector_create(dice_params)

        image = cv2.imread("dice_number.png")
        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) 


        # Detect blobs.
        keypoints = detector.detect(img_gray)

        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(img_gray, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # Show keypoints
        cv2.imshow("Keypoints", im_with_keypoints)
        dice_num = len(keypoints)
        print(dice_num)
        cv2.waitKey(0)

        
        cam = cv2.VideoCapture(4)
        rob.movel(pos_board, a, v)
        while True:
            ret, frame = cam.read()
            if not ret:
                print("failed to grab frame")
                break
            cv2.imshow("test", frame)

            k = cv2.waitKey(1)
            if k%256 == 32:
                # SPACE pressed
                img_name = "in-game_board.png"
                cv2.imwrite(img_name, frame)
                print("{} written!".format(img_name))
                break
        cam.release()
        cv2.destroyAllWindows()

        image = cv2.imread("in-game_board.png")

        # Detect chess pieces
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict, parameters=parameters)
        out = cv2.aruco.drawDetectedMarkers(image, corners, ids)

        markerCenters = []
        markerCenter_urx = []
        for marker in corners:
            centerX = (marker[0][0][0] + marker[0][2][0]) / 2
            centerY = (marker[0][0][1] + marker[0][2][1]) / 2
            markerCoord = []
            markerCoord.append(centerX)
            markerCoord.append(centerY)
            markerCenters.append(markerCoord)
        print(markerCenters)
        cv2.imshow("out",out)
        cv2.waitKey(0)

        # Grab the pieces
        for markerTup in markerCenters:
            pixelX = int(markerTup[0])
            pixelY = int(markerTup[1])
            mXcv = pixelX * 0.00062295
            mYcv = pixelY * 0.00062295
            mXcv = float("{:.4f}".format(mXcv))
            mYcv = float("{:.4f}".format(mYcv))
            mXurx = 0.8880 - mYcv
            mYurx = 0.1575 - mXcv
            markerTup[0] = mXurx
            markerTup[1] = mYurx
            markerCenter_urx.append(markerTup)

        print(markerCenter_urx)

        place_piece(markerCenter_urx[0][0], markerCenter_urx[0][1])

        
    except:
        traceback.print_exc()
    finally:
        rob.close()

cam.release()        
cv2.destroyAllWindows()
