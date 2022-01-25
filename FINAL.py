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
pos_board = [0.685, -0.047, -0.086, -2.106, 2.029, -0.308]
pos_boardj = [0.234, -2.031, -2.085, -0.312, 1.503, 0.184]

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
        
        steps = {
            1 : (0.7232, -0.1647),
            2 : (0.7230, -0.1447),
            3 : (0.7230, -0.1279),
            4 : (0.7230, -0.1074),
            5 : (0.7230, -0.0904),
            6 : (0.7019, -0.0717),
            7 : (0.6853, -0.0718),
            8 : (0.6674, -0.0717),
            9 : (0.6475, -0.0717),
            10 : (0.6284, -0.0741),
            11 : (0.6104, -0.0741),
            12 : (0.6104, -0.0556),
            13 : (0.6104, -0.0383),
            14 : (0.6271, -0.0370),
            15 : (0.6473, -0.0360),
            16 : (0.6658, -0.0361),
            17 : (0.6845, -0.0360),
            18 : (0.7025, -0.-350),
            19 : (0.7197, -0.0160),
            20 : (0.7214, 0.0050),
            21 : (0.7196, 0.0239),
            22 : (0.7196, 0.0397),
            23 : (0.7186, 0.0594),
            24 : (0.7187, 0.0786),
            25 : (0.7374, 0.0786),
            26 : (0.7554, 0.0786),
            27 : (0.7555, 0.0604),
            28 : (0.7575, 0.0420),
            29 : (0.7574, 0.0230),
            30 : (0.7582, 0.0045),
            31 : (0.7582, -0.0144),
            32 : (0.7783, -0.0325),
            33 : (0.7967, -0.0325),
            34 : (0.8143, -0.0320),
            35 : (0.8341, -0.0308),
            36 : (0.8534, -0.0302),
            37 : (0.8710, -0.0303),
            38 : (0.8710, -0.0482),
            39 : (0.8709, -0.0669),
            40 : (0.8524, -0.0674),
            41 : (0.8331, -0.0687),
            42 : (0.8170, -0.0687),
            43 : (0.7986, -0.0687),
            44 : (0.7793, -0.0690),
            45 : (0.7607, -0.0890),
            46 : (0.7606, -0.1207),
            47 : (0.7606, -0.1260),
            48 : (0.7607, -0.1439),
            49 : (0.7606, -0.1624),
            50 : (0.7606,- 0.1818),
            51 : (0.7427, -0.1818),
            52 : (0.7439, -0.1647),
            53 : (0.7442, -0.1458),
            54 : (0.7445, -0.1250),
            55 : (0.7418, -0.1065),
            56 : (0.7420, -0.0891),
            57 : (0.73487,-0.06897)
        }
    
        #print(steps)
        startingPts = []
        startingPts.append((0.6792, -0.1566))
        startingPts.append((0.6763, -0.1195))
        startingPts.append((0.6420, -0.1213))
        startingPts.append((0.6410, -0.1196))

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
        gripper.open_gripper()
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
            mXcv = pixelX * 0.0008261
            mYcv = pixelY * 0.0008261
            mXcv = float("{:.4f}".format(mXcv))
            mYcv = float("{:.4f}".format(mYcv))
            mXurx = 0.8906 - mYcv
            mYurx = 0.2395 - mXcv
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
