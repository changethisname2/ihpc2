"""import keyboard"""
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
        pos_dice_drop = [0.707, 0.219, 0.030, -2.205, 2.205, -0.043]
        pos_dice_grab = [0.719, 0.219, -0.200, -2.205, 2.205, -0.043]
        pos_dice_pic = [0.653, 0.187, -0.163, -2.189, 2.191, -0.011]
	a, v = 0.5, 0.3
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
            detector = cv2.SimpleBlobDetector(params)
        else :
            detector = cv2.SimpleBlobDetector_create(params)

        image = cv2.imread("dice_number.png")
        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


        ''' h,w = img_gray.shape
        ratio = int(h) / int(w)

        # let's upscale the image using new  width and height
        up_width = 600
        up_height = int(up_width * ratio)
        up_points = (up_width, up_height)
        resized_up = cv2.resize(img_gray, up_points, interpolation=cv2.INTER_LINEAR)
        cv2.waitKey(0)


        h,w,c = image.shape '''


        # Detect blobs.
        keypoints = detector.detect(img_gray)


        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(img_gray, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # Show keypoints
        cv2.imshow("Keypoints", im_with_keypoints)

        print(len(keypoints))
        cv2.waitKey(0)

    except:
        traceback.print_exc()
    finally:
        rob.close()
      
       #point4 = point3[:]

       #point4[2] += -0.08

       #rob.movel(point4, 0.4, 0.3)

'''
#a good position right above the board       
Transform:
<Orientation: 
array([[-0.09616118, -0.99501282,  0.02650485],
       [-0.99265482,  0.09390132, -0.07628206],
       [ 0.07341278, -0.03364554, -0.99673394]])>
<Vector: (0.58576, 0.35213, -0.07606)>

'''
cv2.destroyAllWindows()
