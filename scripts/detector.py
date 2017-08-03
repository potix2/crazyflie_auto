import sys
import cv2
import numpy as np
import traceback
import os.path
import time


class CaptureCamera(object):
    def __init__(self, outdir, w, h, fps):
        self._width=w
        self._height=h
        self._fps=fps
        self._path=os.path.join(outdir, '{}.avi'.format(time.strftime('%Y%m%d-%H%M%S')))

    @property
    def width(self):
        return self._width

    @property
    def height(self):
        return self._height

    @property
    def fps(self):
        return self._fps

    @property
    def path(self):
        return self._path


class Detector(object):

    def __init__(self, cc=None):
        self.countNotFound = 0
        self.counter = 0
        self.target_found = True
        self.target = None
        self.capture_callbacks = []
        self._text_start_positions = [
                (2,8), (210,8), (420,8),
                (2,18), (210,18), (420,18)
                ]


        self.add_capture_callback(self._capture_time)
        self.add_capture_callback(self._capture_position)

        self.capture_camera = cc
        if cc is not None:
            size = (self.capture_camera.width, self.capture_camera.height)
            self.video_writer = cv2.VideoWriter(
                    self.capture_camera.path, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), self.capture_camera.fps, size)

    def _capture_time(self):
        return 'Time: %s' % time.strftime('%Y-%m-%d %H:%M:%S')

    def _capture_position(self):
        if self.target is not None:
            return 'Target Position: (%d, %d)' % (self.target[0], self.target[1])
        else:
            return 'Target Position: (-, -)'

    def add_capture_callback(self, callback):
        self.capture_callbacks.append((callback, self._text_start_positions[len(self.capture_callbacks)]))

    def set_mask_range_callback(self, callback):
        self._mask_range_callback = callback

    def _capture(self, image):
        # show image with target outlined with a red rectangle
        if self.capture_camera is not None:
            for (callback, start_position) in self.capture_callbacks:
                text = callback()
                cv2.putText(image, text, start_position, cv2.FONT_HERSHEY_SIMPLEX, 0.3, (196,196,196))
            self.video_writer.write(image)

    def detect(self, image):
        target_u = 0
        target_v = 0

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # green origami
        lower, upper = self._mask_range_callback()
        mask = cv2.inRange(hsv, lower, upper)

        # dilate and erode with kernel size 11x11
        cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((11,11)))

        # find all of the contours in the mask image
        cImage, contours, heirarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contourLength  = len(contours)

        # Check for at least one target found
        if contourLength < 1:
            self._capture(image)
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

                self.target = (target_u, target_v)
                self._capture(image)
                return self.target
            except:
                self.countNotFound += 1
                if self.countNotFound > 10:
                    self.target_found = False
                return None


def _uvc_mask_range():
    lower = np.array([0, 0, 200], np.uint8)
    upper = np.array([360, 255, 255], np.uint8)
    return (lower, upper)

if __name__ == '__main__':
    cam = cv2.VideoCapture(0)
    size = (int(cam.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    print("size=({}, {})".format(size[0], size[1]))
    path = '/home/potix2/sampling'
    cc = CaptureCamera(path, size[0], size[1], fps=30)
    detector = Detector(cc)
    detector.set_mask_range_callback(_uvc_mask_range)
    while True:
        ret, img = cam.read()
        if img is not None:
            ret = detector.detect(img)
            if ret is None:
                continue
            u, v = ret
    cv2.destroyAllWindows()
