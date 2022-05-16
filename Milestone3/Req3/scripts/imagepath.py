#!/usr/bin/env python3

import rospy
import numpy as np
import cv2

img = cv2.imread('/home/teleb/catkin_ws/src/milestone3Autonomous/scripts/path.png')
grayImage = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ret, bw_img = cv2.threshold(grayImage,30,255,cv2.THRESH_BINARY)
print(type(bw_img))
cv2.imshow("window", img)

h,w = bw_img.shape
print('hight: ', h)
print('width: ', w)
print(bw_img)


cv2.waitKey(0)
cv2.destroyAllWindows()

