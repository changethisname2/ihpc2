# import keyboard
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

pos_dice_drop = [0.707, 0.219, 0.030, -2.205, 2.205, -0.043]
pos_dice_grab = [0.719, 0.219, -0.200, -2.205, 2.205, -0.043]
pos_board = [0.726, -0.078, 0.085, -2.125, 2.190, -0.108]

a, v = 0.5, 0.3
def grab_piece(x, y):
    above_piece, rxb, ryb, rzb = -0.1502, -2.1890, 2.1920, -0.0111
    at_piece, rx, ry, rz = -0.2025, -2.1890,2.1910,-0.0110
    gripper.open_gripper()
    rob.movel([x, y, above_piece, rxb, ryb, rzb], a, v)
    rob.movel([x, y, at_piece, rx, ry, rz], a, v)
    gripper.close_gripper()
    rob.movel([x, y, above_piece, rx, ry, rz], a, v)

def place_piece(x, y):
    above_piece, rxb, ryb, rzb = -0.1502, -2.1890, 2.1920, -0.0111
    at_piece, rx, ry, rz = -0.2025, -2.1890,2.1910,-0.0110
    rob.movel([x, y, above_piece, rxb, ryb, rzb], a, v)
    rob.movel([x, y, at_piece, rx, ry, rz], a, v)
    gripper.open_gripper()
    rob.movel([x, y, above_piece, rxb, ryb, rzb], a, v)

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
            mXcv = pixelX * 0.000826086957
            mYcv = pixelY * 0.000826086957
            mXcv = float("{:.4f}".format(mXcv))
            mYcv = float("{:.4f}".format(mYcv))
            mXurx = 0.89517 - mYcv
            mYurx = 0.22763 - mXcv
            cordTup = (mXurx,mYurx)
            coordinate.append(cordTup)
            print (number, cordTup)
            number += 1

        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(img_gray, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # Show keypoints
        cv2.imshow("Keypoints", im_with_keypoints)
        cv2.waitKey(0)

        cv2.destroyAllWindows()
        '''
        steps = {
            1 : coordinate [63],
            2 : coordinate [38],
            3 : coordinate [16],
            4 : coordinate [17],
            5 : coordinate [18],
            6 : coordinate [15],
            7 : coordinate [13],
            8 : coordinate [58],
            9 : coordinate [57],
            10 : coordinate [32],
            11 : coordinate [69],
            12 : coordinate [55],
            13 : coordinate [68],
            14 : coordinate [31],
            15 : coordinate [33],
            16 : coordinate [34],
            17 : coordinate [35],
            18 : coordinate [14],
            19 : coordinate [36],
            20 : coordinate [37],
            21 : coordinate [71],
            22 : coordinate [72],
            23 : coordinate [62],
            24 : coordinate [61],
            25 : coordinate [65],
            26 : coordinate [66],
            27 : coordinate [67],
            28 : coordinate [44],
            29 : coordinate [43],
            30 : coordinate [42],
            31 : coordinate [9],
            32 : coordinate [23],
            33 : coordinate [24],
            34 : coordinate [25],
            35 : coordinate [27],
            36 : coordinate [29],
            37 : coordinate [53],
            38 : coordinate [52],
            39 : coordinate [54],
            40 : coordinate [31],
            41 : coordinate [28],
            42 : coordinate [26],
            43 : coordinate [51],
            44 : coordinate [50],
            45 : coordinate [45],
            46 : coordinate [46],
            47 : coordinate [22],
            48 : coordinate [48],
            49 : coordinate [47],
            50 : coordinate [49],
            51 : coordinate [41],
            52 : coordinate [21],
            53 : coordinate [40],
            54 : coordinate [20],
            55 : coordinate [39],
            56 : coordinate [19]
        }
        '''
        #print(steps)
        '''
        startingPts = []
        startingPts.append(coordinate[59])
        startingPts.append(coordinate[60])
        startingPts.append(coordinate[56])
        startingPts.append(coordinate[70])
        '''
        for step in steps:
            next_step_num_e = 8
            for nextStep in steps:
                if next_step_num_e == nextStep:
                    gripper.open_gripper()
                    grab_piece(0.73487,-0.10697)
                    place_piece(0.6, -0.1)           
    except:
        traceback.print_exc()
    finally:
        rob.close()

cv2.destroyAllWindows()
