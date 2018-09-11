import cv2
import cv2.aruco as aruco
import numpy as np
import sys


mapImg = cv2.imread('catkin_ws_user/src/local_localization/images/fu_robotics_lab_map_no_aruco.jpg', cv2.IMREAD_GRAYSCALE )
# flip img about both axises (rotate by 180 degrees)
mapImg = cv2.flip(mapImg, -1)

dinA4 = np.full((30,22), 255, dtype=np.uint8)

# this dictionary contains information about all aruco markers
arucoDict = aruco.Dictionary_get(aruco.DICT_6X6_250)

# here the actual markers are saved into variables
# could be done in an array I guess.. except no. 25 is missing
aruco21 = aruco.drawMarker(arucoDict, 21, 20)
aruco22 = aruco.drawMarker(arucoDict, 22, 20)
aruco23 = aruco.drawMarker(arucoDict, 23, 20)
aruco24 = aruco.drawMarker(arucoDict, 24, 20)
aruco26 = aruco.drawMarker(arucoDict, 26, 20)
aruco27 = aruco.drawMarker(arucoDict, 27, 20)
aruco28 = aruco.drawMarker(arucoDict, 28, 20)
aruco29 = aruco.drawMarker(arucoDict, 29, 20)

# flip the aruco images according to how they are positioned on the field
aruco21 = cv2.flip(aruco21, 0)
aruco22 = cv2.flip(aruco22, 0)
aruco23 = cv2.flip(aruco23, 0)
aruco24 = cv2.flip(aruco24, 0)
aruco26 = cv2.flip(aruco26, 0)
aruco27 = cv2.flip(aruco27, 0)
aruco28 = cv2.flip(aruco28, 0)
aruco29 = cv2.flip(aruco29, 0)

# hardcoded positions of the aruco markers
# taken from: https://github.com/AutoModelCar/visual_odometry/blob/master/launch/SingleCameraLocalization.launch
# these are the centers of the aruco markers, so some calculations are still necessary to find out the actual corner positions
pos21 = (17,  12)
pos22 = (370, 553)
pos23 = (370, 12)
pos24 = (17,  553)
pos26 = (370, 362)
pos27 = (17,  212)
pos28 = (370, 212)
pos29 = (17,  362)

# place white areas on the map, simulating DIN-A4 sheets of paper
lowerX = int(pos21[0] - (dinA4.shape[0] / 2))
lowerY = int(pos21[1] - (dinA4.shape[1] / 2))
upperX = int(pos21[0] + (dinA4.shape[0] / 2))
upperY = int(pos21[1] + (dinA4.shape[1] / 2))
mapImg[lowerX:upperX, lowerY:upperY] = dinA4
lowerX = int(pos22[0] - (dinA4.shape[0] / 2))
lowerY = int(pos22[1] - (dinA4.shape[1] / 2))
upperX = int(pos22[0] + (dinA4.shape[0] / 2))
upperY = int(pos22[1] + (dinA4.shape[1] / 2))
mapImg[lowerX:upperX, lowerY:upperY] = dinA4
lowerX = int(pos23[0] - (dinA4.shape[0] / 2))
lowerY = int(pos23[1] - (dinA4.shape[1] / 2))
upperX = int(pos23[0] + (dinA4.shape[0] / 2))
upperY = int(pos23[1] + (dinA4.shape[1] / 2))
mapImg[lowerX : upperX, lowerY : upperY] = dinA4
lowerX = int(pos24[0] - (dinA4.shape[0] / 2))
lowerY = int(pos24[1] - (dinA4.shape[1] / 2))
upperX = int(pos24[0] + (dinA4.shape[0] / 2))
upperY = int(pos24[1] + (dinA4.shape[1] / 2))
mapImg[lowerX : upperX, lowerY : upperY] = dinA4
lowerX = int(pos26[0] - (dinA4.shape[0] / 2))
lowerY = int(pos26[1] - (dinA4.shape[1] / 2))
upperX = int(pos26[0] + (dinA4.shape[0] / 2))
upperY = int(pos26[1] + (dinA4.shape[1] / 2))
mapImg[lowerX : upperX, lowerY : upperY] = dinA4
lowerX = int(pos27[0] - (dinA4.shape[0] / 2))
lowerY = int(pos27[1] - (dinA4.shape[1] / 2))
upperX = int(pos27[0] + (dinA4.shape[0] / 2))
upperY = int(pos27[1] + (dinA4.shape[1] / 2))
mapImg[lowerX : upperX, lowerY : upperY] = dinA4
lowerX = int(pos28[0] - (dinA4.shape[0] / 2))
lowerY = int(pos28[1] - (dinA4.shape[1] / 2))
upperX = int(pos28[0] + (dinA4.shape[0] / 2))
upperY = int(pos28[1] + (dinA4.shape[1] / 2))
mapImg[lowerX : upperX, lowerY : upperY] = dinA4
lowerX = int(pos29[0] - (dinA4.shape[0] / 2))
lowerY = int(pos29[1] - (dinA4.shape[1] / 2))
upperX = int(pos29[0] + (dinA4.shape[0] / 2))
upperY = int(pos29[1] + (dinA4.shape[1] / 2))
mapImg[lowerX : upperX, lowerY : upperY] = dinA4

# draw the aruco markers on the simulated sheets of paper
lowerX = int(pos21[0] - (aruco21.shape[0] / 2))
lowerY = int(pos21[1] - (aruco21.shape[1] / 2))
upperX = int(pos21[0] + (aruco21.shape[0] / 2))
upperY = int(pos21[1] + (aruco21.shape[1] / 2))
mapImg[lowerX : upperX, lowerY : upperY] = aruco21
lowerX = int(pos22[0] - (aruco22.shape[0] / 2))
lowerY = int(pos22[1] - (aruco22.shape[1] / 2))
upperX = int(pos22[0] + (aruco22.shape[0] / 2))
upperY = int(pos22[1] + (aruco22.shape[1] / 2))
mapImg[lowerX : upperX, lowerY : upperY] = aruco22
lowerX = int(pos23[0] - (aruco23.shape[0] / 2))
lowerY = int(pos23[1] - (aruco23.shape[1] / 2))
upperX = int(pos23[0] + (aruco23.shape[0] / 2))
upperY = int(pos23[1] + (aruco23.shape[1] / 2))
mapImg[lowerX : upperX, lowerY : upperY] = aruco23
lowerX = int(pos24[0] - (aruco24.shape[0] / 2))
lowerY = int(pos24[1] - (aruco24.shape[1] / 2))
upperX = int(pos24[0] + (aruco24.shape[0] / 2))
upperY = int(pos24[1] + (aruco24.shape[1] / 2))
mapImg[lowerX : upperX, lowerY : upperY] = aruco24
lowerX = int(pos26[0] - (aruco26.shape[0] / 2))
lowerY = int(pos26[1] - (aruco26.shape[1] / 2))
upperX = int(pos26[0] + (aruco26.shape[0] / 2))
upperY = int(pos26[1] + (aruco26.shape[1] / 2))
mapImg[lowerX : upperX, lowerY : upperY] = aruco26
lowerX = int(pos27[0] - (aruco27.shape[0] / 2))
lowerY = int(pos27[1] - (aruco27.shape[1] / 2))
upperX = int(pos27[0] + (aruco27.shape[0] / 2))
upperY = int(pos27[1] + (aruco27.shape[1] / 2))
mapImg[lowerX : upperX, lowerY : upperY] = aruco27
lowerX = int(pos28[0] - (aruco28.shape[0] / 2))
lowerY = int(pos28[1] - (aruco28.shape[1] / 2))
upperX = int(pos28[0] + (aruco28.shape[0] / 2))
upperY = int(pos28[1] + (aruco28.shape[1] / 2))
mapImg[lowerX : upperX, lowerY : upperY] = aruco28
lowerX = int(pos29[0] - (aruco29.shape[0] / 2))
lowerY = int(pos29[1] - (aruco29.shape[1] / 2))
upperX = int(pos29[0] + (aruco29.shape[0] / 2))
upperY = int(pos29[1] + (aruco29.shape[1] / 2))
mapImg[lowerX : upperX, lowerY : upperY] = aruco29

# flip (rotate) the image back to its original orientation
mapImg = cv2.flip(mapImg, -1)

cv2.imwrite('catkin_ws_user/src/local_localization/images/fu_robotics_lab_map.jpg', mapImg)
#cv2.namedWindow('image', cv2.WINDOW_NORMAL)
#cv2.imshow('image', mapImg)
#cv2.waitKey(0)
#cv2.destroyAllWindows()