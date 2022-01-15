"""Import keyboard"""
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
def grab_piece(x, y):
    above_piece, rxb, ryb, rzb = -0.1502, -2.1890, 2.1920, -0.0111
    at_piece, rx, ry, rz = -0.2025, -2.1890, 2.1910, -0.0110
    rob.movel([x, y, above_piece, rxb, ryb, rzb], a, v)
    rob.movel([x, y, at_piece, rx, ry, rz], a, v)
    gripper.close_gripper()
    rob.movel([x, y, above_piece, rx, ry, rz], a, v)
def place_piece(x, y):
    rob.movel([x, y, above_piece, rxb, ryb, rzb], a, v)
    rob.movel([x, y, at_piece, rx, ry, rz],a,v)
    gripper.open_gripper()
    rob.movel([x, y, above_piece, rxb, ryb, rzb], a, v)

if __name__ == "__main__":
    try:
        """Connecting to Robot"""
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

        """Rolling dice"""
        pos_boardj = [0.137, -1.985, -1.438, -1.176, 1.563, 0.166]
        pos_dice_drop = [0.707, 0.219, 0.030, -2.205, 2.205, -0.043]
        pos_dice_grab = [0.719, 0.219, -0.200, -2.205, 2.205, -0.043]
        pos_dice_pic = [0.653, 0.187, -0.163, -2.189, 2.191, -0.011]
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

        """Detecting the dice"""
        # Setup SimpleBlobDetector parameters.
        dice_params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        dice_params.minThreshold = 50
        dice_params.maxThreshold = 15000

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

        # Detect blobs
        keypoints = detector.detect(img_gray)

        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(img_gray, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # Show keypoints
        cv2.imshow("Keypoints", im_with_keypoints)
        dice_num = len(keypoints)
        print(dice_num)
        cv2.waitKey(0)
	
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
                    next_step_num_o = step + dice_num
                    for nextStep in steps:
                        if next_step_num_o == nextStep:
                            grab_piece(markerCenter_urx[0][0], markerCenter_urx[0][1])
                            place_piece(steps[nextStep][0], steps[nextSTep][1])
    else:
        if numAtStart == 4:
            next_step_num_e == dice_num
            for nextStep in steps:
                if next_step_num_e == nextStep:
                    grab_piece(markerCenter_urx[0][0],markerCenter[0][1])
                    place_piece(steps[nextStep][0],steps[nextStep][1])
        elif numAtStart == 3:
            rng between (0, 1)
            if rng == 0:
               next_step_num_e == dice_num
               for nextStep in steps:
                   if next_step_num_e == nextStep:
                      grab_piece(markerCenter_urx[1][0],markerCenter_urx[1][1])
                      place_piece(steps[nextStep][0],steps[nextStep][1])
            else:
               for step in steps:
                   if markerCenter_urx[0][0] == steps[step][0] and markerCenter_urx[0][1] == step[step][1]:
                    next_step_num_e = step + dice_num
                    for nextStep in steps:
                        if next_step_num_e == nextStep:
                            grab_piece(markerCenter_urx[0][0], markerCenter_urx[0][1])
                            place_piece(steps[nextStep][0], steps[nextStep][1])
        else:
            if markerCenter_urx[0][0] != end_point[0] and markerCenter_urx[0][1] != end_point[1]:
               for step in steps:
                   if markerCenter_urx[0][0] == steps[step][0] and markerCenter_urx[0][1] == step[step][1]:
                   next_step_num_e = step + dice_num
                   for nextStep in steps:
                       if next_step_num_e == nextStep:
                            grab_piece(markerCenter_urx[0][0], markerCenter_urx[0][1])
                            place_piece(steps[nextStep][0], steps[nextSTep][1])
            else:
                next_step_num_e == dice_num
                for nextStep in steps:
                if next_step_num_e == nextStep:
                    grab_piece(markerCenter_urx[2][0],markerCenter[2][1])
                    place_piece(steps[nextStep][0],steps[nextStep][1])
                
            else:
                move the furthest piece(improve later)"""
    except:
        traceback.print_exc()
    finally:
        rob.close()
      

cv2.destroyAllWindows()
