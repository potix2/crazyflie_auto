#!/usr/bin/env python

import cv2
import numpy

# read png image and convert the image to HSV
image = cv2.imread("/home/potix2/catkin_ws/src/crazyflie_auto/test_data/target.png", cv2.IMREAD_COLOR)
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

lower_green = numpy.array([143 / 2 - 10, 63 * 255 / 100 - 30, 47 * 255 / 100 - 30], numpy.uint8)
upper_green = numpy.array([143 / 2 + 10, 63 * 255 / 100 + 30, 47 * 255 / 100 + 30], numpy.uint8)
mask = cv2.inRange(hsv, lower_green, upper_green)

cv2.imshow('original', image)
cv2.imshow('mask', mask)
#cv2.imwrite("hsv_mask.png", mask)
cv2.waitKey(0)
cv2.destroyAllWindows()
