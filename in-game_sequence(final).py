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
        a, v = 0.5, 0.3
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

    except:
        traceback.print_exc()
    finally:
        rob.close()
      

cv2.destroyAllWindows()
