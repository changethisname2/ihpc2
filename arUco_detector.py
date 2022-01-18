# import the necessary packages
import argparse
import imutils
import cv2
import sys

image = cv2.imread("in-game_board.png")

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters_create()

# Detect the markers.
corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image,aruco_dict,parameters=parameters)

out = cv2.aruco.drawDetectedMarkers(image, corners, ids)

markerCentres = []

count = 0

for marker in corners:
    centreX = (marker[0][0][0] + marker[0][2][0]) / 2
    centreY = (marker[0][0][1] + marker[0][2][1]) / 2
    markerTup = (centreX, centreY, ids[count])
    markerCentres.append(markerTup)
    count += 1

print(markerCentres)

sampleID = 1

for marker in markerCentres:
    if marker[2] == sampleID:
        print(marker)


cv2.imshow("out",out)
cv2.waitKey(0)
cv2.destroyAllWindows()
