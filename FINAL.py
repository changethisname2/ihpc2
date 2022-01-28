import keyboard
import cv2
import urx
from IPython import embed
import random
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
pos_board = [0.707, -0.053, 0.035, -2.042, 2.027, -0.376]
pos_boardj = [0.234, -1.847, -1.965, -0.558, 1.478, 0.208]

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
    gripper.gripper_action(150)
    rob.movel([x, y, above_piece, rxb, ryb, rzb], a, v)

def move_piece(markerCenter_urx, steps, dice_num):
    for step in steps:
        if ((markerCenter_urx[0][0] >= (steps[step][0] - 0.01)) and (markerCenter_urx[0][0] <= (steps[step][0] + 0.01))) and ((markerCenter_urx[0][1] >= (steps[step][1] - 0.01)) and (markerCenter_urx[0][1] <= (steps[step][1] + 0.01))): 
            marker_pos = step
    diff = (57 - marker_pos)
    if diff < 6:
        if diff < dice_num:
            key = 57 - (dice_num - diff)
            grab_piece(markerCenter_urx[0][0], markerCenter_urx[0][1])
            place_piece(steps[key][0], steps[key][1])
        else:
            for step in steps:
                if ((markerCenter_urx[0][0] >= (steps[step][0] - 0.01)) and (markerCenter_urx[0][0] <= (steps[step][0] + 0.01))) and ((markerCenter_urx[0][1] >= (steps[step][1] - 0.01)) and (markerCenter_urx[0][1] <= (steps[step][1] + 0.01))):
                    next_step_num = step + dice_num
                    for nextStep in steps:
                        if next_step_num == nextStep:
                            grab_piece(markerCenter_urx[0][0], markerCenter_urx[0][1])
                            place_piece(steps[nextStep][0], steps[nextStep][1])
    else:
        for step in steps:
            if ((markerCenter_urx[0][0] >= (steps[step][0] - 0.01)) and (markerCenter_urx[0][0] <= (steps[step][0] + 0.01))) and ((markerCenter_urx[0][1] >= (steps[step][1] - 0.01)) and (markerCenter_urx[0][1] <= (steps[step][1] + 0.01))):
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
                rob = urx.Robot("192.168.1.5")
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
            1 : (0.715, -0.16),
            2 : (0.716, -0.145),
            3 : (0.715, -0.125),
            4 : (0.715, -0.105),
            5 : (0.715, -0.09),
            6 : (0.695, -0.07),
            7 : (0.677, -0.07),
            8 : (0.661, -0.07),
            9 : (0.641, -0.07),
            10 : (0.623, -0.07),
            11 : (0.604, -0.07),
            12 : (0.604, -0.054),
            13 : (0.604, -0.035),
            14 : (0.623, -0.035),
            15 : (0.641, -0.035),
            16 : (0.6595, -0.034),
            17 : (0.678, -0.034),
            18 : (0.696, -0.034),
            19 : (0.7148, -0.0145),
            20 : (0.7148, 0.0050),
            21 : (0.7148, 0.0239),
            22 : (0.7148, 0.0415),
            23 : (0.7148, 0.0595),
            24 : (0.7148, 0.0786),
            25 : (0.7329, 0.0786),
            26 : (0.7524, 0.0786),
            27 : (0.7524, 0.0604),
            28 : (0.7524, 0.0420),
            29 : (0.7524, 0.0229),
            30 : (0.7524, 0.0045),
            31 : (0.7529, -0.0144),
            32 : (0.7718, -0.0305),
            33 : (0.7902, -0.0305),
            34 : (0.8083, -0.0305),
            35 : (0.8271, -0.0305),
            36 : (0.8464, -0.0305),
            37 : (0.8660, -0.0305),
            38 : (0.8660, -0.0472),
            39 : (0.8660, -0.0669),
            40 : (0.8479, -0.0674),
            41 : (0.8266, -0.0659),
            42 : (0.8090, -0.0687),
            43 : (0.7902, -0.0687),
            44 : (0.7718, -0.0690),
            45 : (0.7539, -0.090),
            46 : (0.7544, -0.105),
            47 : (0.7549, -0.125),
            48 : (0.7549, -0.1439),
            49 : (0.7554, -0.1624),
            50 : (0.7559, -0.1818),
            51 : (0.7377, -0.1818),
            52 : (0.7377, -0.1624),
            53 : (0.7377, -0.1439),
            54 : (0.7372, -0.125),
            55 : (0.7367, -0.107),
            56 : (0.7362, -0.0885),
            57 : (0.7343, -0.06897)
        }
   
        startingPts = []
        startingPts.append((0.67, -0.16))
        startingPts.append((0.67, -0.12))
        startingPts.append((0.63, -0.16))
        startingPts.append((0.63, -0.12))

        """Rolling dice"""
        rob.movej(pos_boardj, a, v)
        rob.movel(pos_dice_drop, a, v)
        rob.movel(pos_dice_grab, a, v)
        gripper.close_gripper()
        rob.movel(pos_dice_drop, a, v)
        gripper.gripper_action(150)

        """Taking picture of dice"""
        cam = cv2.VideoCapture('/dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_435_Intel_R__RealSense_TM__Depth_Camera_435_935523026947-video-index0')
        rob.movel(pos_dice_pic, a, v)
        while True:
            ret, frame = cam.read()
            if not ret:
                print("failed to grab frame")
                break
                
            k = cv2.waitKey(1)
            if k%256 == 32:
                # SPACE pressed
                img_name = "dice_number.png"
                cv2.imwrite(img_name, frame)
                #print("{} written!".format(img_name))
                break
        cam.release()
        cv2.destroyAllWindows()

        """Detecting the dice"""
        # Setup SimpleBlobDetector parameters.
        dice_params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        dice_params.minThreshold = 50
        dice_params.maxThreshold = 15000

        # Filter by Area
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

        # Detect blobs
        keypoints = detector.detect(img_gray)

        # Draw detected blobs as red circles
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(img_gray, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # Show keypoints
        dice_num = len(keypoints)
   
        cam = cv2.VideoCapture('/dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_435_Intel_R__RealSense_TM__Depth_Camera_435_935523026947-video-index0')
        gripper.open_gripper()
        rob.movel(pos_board, a, v)
        while True:
            ret, frame = cam.read()
            if not ret:
                print("failed to grab frame")
                break
            
            k = cv2.waitKey(1)
            if k%256 == 32:
                # SPACE pressed
                img_name = "in-game_board.png"
                cv2.imwrite(img_name, frame)
                #print("{} written!".format(img_name))
                break
        cam.release()
        cv2.destroyAllWindows()

        gripper.gripper_action(100)

        image = cv2.imread("in-game_board.png")

        # Detect chess pieces
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict, parameters=parameters)
        out = cv2.aruco.drawDetectedMarkers(image, corners, ids)

        markerCenters = []
        markerCenter_urxmix = []
        markerCenter_urx = []
        count = 0
        for marker in corners:
            centerX = (marker[0][0][0] + marker[0][2][0]) / 2
            centerY = (marker[0][0][1] + marker[0][2][1]) / 2
            markerCoord = []
            markerCoord.append(centerX)
            markerCoord.append(centerY)
            markerCoord.append(ids[count])
            count += 1
            markerCenters.append(markerCoord)

        # Grab the pieces
        for markerTup in markerCenters:
            pixelX = int(markerTup[0])
            pixelY = int(markerTup[1])
            mXcv = pixelX * 0.0007308
            mYcv = pixelY * 0.0007308
            mXcv = float("{:.4f}".format(mXcv))
            mYcv = float("{:.4f}".format(mYcv))
            mXurx = 0.8911 - mYcv
            mYurx = 0.2214 - mXcv
            mXurx = float("{:.2f}".format(mXurx))
            mYurx = float("{:.2f}".format(mYurx))   
            markerTup[0] = mXurx
            markerTup[1] = mYurx
            markerCenter_urxmix.append(markerTup)        
        
        count = 0 
        for turn in range(4):
            for marker in markerCenter_urxmix:
                if marker[2][0] == count:
                    markerCenter_urx.append(marker)
            count += 1

        numAtStart = 0
        for markerCenter in markerCenter_urx:
            for start in startingPts:
                if ((markerCenter[0] >= (start[0] - 0.01)) and (markerCenter[0] <= (start[0] + 0.01))) and ((markerCenter[1] >= (start[1] - 0.01)) and (markerCenter[1] <= (start[1] + 0.01))):
                    numAtStart += 1

        if dice_num % 2 == 1:
            if len(markerCenter_urx) - numAtStart == 0:
                pass
            else:
                move_piece(markerCenter_urx, steps, dice_num)

        else:
            if len(markerCenter_urx) - numAtStart == 0:
                next_step_num_e = dice_num
                for nextStep in steps:
                    if next_step_num_e == nextStep:
                        grab_piece(markerCenter_urx[0][0], markerCenter_urx[0][1])
                        place_piece(steps[nextStep][0], steps[nextStep][1])
            elif len(markerCenter_urx) - numAtStart == 1:
                if len(markerCenter_urx) > 1:
                    decision = random.randint(0, 1)
                    if decision == 0:
                        next_step_num_e = dice_num
                        for nextStep in steps:
                            if next_step_num_e == nextStep:
                                grab_piece(markerCenter_urx[1][0], markerCenter_urx[1][1])
                                place_piece(steps[nextStep][0], steps[nextStep][1])
                    else:
                        move_piece(markerCenter_urx, steps, dice_num)
                else:
                    move_piece(markerCenter_urx, steps, dice_num)
            elif len(markerCenter_urx) - numAtStart == 2:
                move_piece(markerCenter_urx, steps, dice_num)

    except:
        traceback.print_exc()
    finally:
        rob.close()

cv2.destroyAllWindows()
