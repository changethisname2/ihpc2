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

a, v = 0.5, 0.3
pos_dice_drop = [0.707, 0.219, 0.030, -2.205, 2.205, -0.043]
pos_dice_grab = [0.719, 0.219, -0.200, -2.205, 2.205, -0.043]
pos_dice_pic = [0.653, 0.187, -0.163, -2.189, 2.191, -0.011]
pos_board = [0.707, -0.053, 0.035, -2.042, 2.027, -0.376]
pos_boardj = [0.234, -1.847, -1.965, -0.558, 1.478, 0.208]

def grab_piece(x, y):
    above_piece, rxb, ryb, rzb = -0.1502, -2.1890, 2.1920, -0.0111
    at_piece, rx, ry, rz = -0.2025, -2.1890, 2.1910, -0.0110
    rob.movel([x, y, above_piece, rxb, ryb, rzb],0.2,0.3)
    rob.movel([x, y, at_piece, rx, ry, rz],0.2,0.3)
    gripper.close_gripper()
    rob.movel([x, y, above_piece, rx, ry, rz],0.2,0.3)

def place_piece(x, y):
    above_piece, rxb, ryb, rzb = -0.1502, -2.1890, 2.1920, -0.0111
    at_piece, rx, ry, rz = -0.2025, -2.1890, 2.1910, -0.0110
    rob.movel([x, y, above_piece, rxb, ryb, rzb],0.2,0.3)
    rob.movel([x, y, at_piece, rx, ry, rz],0.2,0.3)
    gripper.gripper_action(150)
    rob.movel([x, y, above_piece, rxb, ryb, rzb],0.2,0.3)

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
        params.minThreshold = 50
        params.maxThreshold = 15000

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
            mXcv = pixelX * 0.0007308
            mYcv = pixelY * 0.0007308
            mXcv = float("{:.4f}".format(mXcv))
            mYcv = float("{:.4f}".format(mYcv))
            mXurx = 0.8911 - mYcv
            mYurx = 0.2214 - mXcv
            cordTup = (mXurx,mYurx)
            coordinate.append(cordTup)
        endPts = (0.73487,-0.06897)

        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(img_gray, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # Show keypoints
        cv2.imshow("Keypoints", im_with_keypoints)
        cv2.waitKey(0)

        cv2.destroyAllWindows()
        
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

        """
        rob.movel(pos_board, a, v)
        for step in steps:
            next_step_num_e = 8
            for nextStep in steps:
                if next_step_num_e == nextStep:
                    gripper.open_gripper()
                    #grab_piece(startingPts[0][0],startingPts[0][1])
                    place_piece(steps[26][0],steps[26][1])
        """
    except:
        traceback.print_exc()
    finally:
        rob.close()

cv2.destroyAllWindows()
