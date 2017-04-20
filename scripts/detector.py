import sys
import cv2
import numpy as np
import traceback


class Detector(object):

    def __init__(self):
        self.countNotFound = 0
        self.counter = 0
        self.target_found = True

    def detect(self, image):
        target_u = 0
        target_v = 0

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        h_margin = 10
        s_margin = 30
        v_margin = 30

        # green origami
        lower = np.array([143 / 2 - h_margin, 63 * 255 / 100 - s_margin, 47 * 255 / 100 - v_margin], np.uint8)
        upper = np.array([143 / 2 + h_margin, 63 * 255 / 100 + s_margin, 47 * 255 / 100 + v_margin], np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        # dilate and erode with kernel size 11x11
        cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((11,11)))

        # find all of the contours in the mask image
        cImage, contours, heirarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contourLength  = len(contours)

        # Check for at least one target found
        if contourLength < 1:
            return None
        else:
            # target found
            ## Loop through all of the contours, and get their areas
            try:
                area = np.array([0.0]*len(contours))
                for i in range(contourLength):
                   area[i] = cv2.contourArea(contours[i])

                #### Target #### the largest "pink" object
                indicies = np.argpartition(-area, 1)[:1]
                tu = 0
                tv = 0
                for i in indicies:
                    target_image = contours[i]

                    # Using moments find the center of the object and draw a red outline around the object
                    target_m = cv2.moments(target_image)
                    target_u = int(target_m['m10']/target_m['m00'])
                    target_v = int(target_m['m01']/target_m['m00'])
                    points = cv2.minAreaRect(target_image)
                    box = cv2.boxPoints(points)
                    box = np.int0(box)
                    cv2.drawContours(image, [box], 0, (0, 0, 255), 2)
                    tu += target_u
                    tv += target_v
                target_u = int(tu / 2)
                target_v = int(tv / 2)

                self.countNotFound = 0
                self.counter += 1
                self.target_found = True

                # show image with target outlined with a red rectangle
                if self.counter % 5 == 0:
                    cv2.imwrite("/home/potix2/sampling/%d.png" % self.counter, image)

                return (target_u, target_v)
            except:
                self.countNotFound += 1
                if self.countNotFound > 10:
                    self.target_found = False
                return None


