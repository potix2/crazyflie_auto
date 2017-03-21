#!/usr/bin/env python

import cv2
import numpy

# read png image and convert the image to HSV
image = cv2.imread("/home/potix2/catkin_ws/src/crazyflie_auto/test_data/blue4.png", cv2.IMREAD_COLOR)
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

class HSVRangeFinder(object):
    def __init__(self, h, s, v):
        self.h = h
        self.s = s
        self.v = v

    def range(l, u):
        lh = max(self.h / 2 - l * hoffset, 0)
        ls = max(self.s * 255 / 100 - l * soffset, 0)
        lv = max(self.v * 255 / 100 - v * voffset, 0)
        uh = min(self.h / 2 + l * hoffset, 180)
        us = min(self.s * 255 / 100 + l * soffset, 0)
        uv = min(self.v * 255 / 100 + v * voffset, 0)
        lower = numpy.array([lh, ls, lv], numpy.uint8)
        upper = numpy.array([uh, us, uv], numpy.uint8)
        return (lower, upper)

# detect yellow
hoffset = 5
soffset = 10
voffset = 10
# duck
# h = 55
# s = 22
# v = 98

# blue light
h = 216
s = 67
v = 99

for l in range(0, 10):
    for u in range(0, 10):
        lh = max(h / 2 - l * hoffset, 0)
        ls = max(s * 255 / 100 - l * soffset, 0)
        lv = max(v * 255 / 100 - l * voffset, 0)
        uh = min(h / 2 + u * hoffset, 180)
        us = min(s * 255 / 100 + u * soffset, 255)
        uv = min(v * 255 / 100 + u * voffset, 255)
        lower = numpy.array([lh, ls, lv], numpy.uint8)
        upper = numpy.array([uh, us, uv], numpy.uint8)
        print("lower=%s, upper=%s" % (lower, upper))

        mask = cv2.inRange(hsv, lower, upper)
        cv2.imshow('mask', mask)
        # cv2.imwrite("/home/potix2/catkin_ws/src/crazyflie_auto/test_mask/%d_%d.png" % (l,u), mask)
        cv2.waitKey(0)


# cv2.imshow('original', image)
cv2.destroyAllWindows()
