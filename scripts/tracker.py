#!/usr/bin/env python
"""
Script to retrieve sensor images from kinect
"""
import sys
import rospy
import roslib
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tf
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

class CrazyflieTracker:

    def __init__(self):
        rospy.init_node('crazyflie_tracker')
        self.bridge = CvBridge()

        # publish transform rate at 50Hz
        self.rate = rospy.Rate(50.0)

        # subscribe to kinect image messages
        rospy.wait_for_message("/kinect2/qhd/camera_info", CameraInfo)

        # Subscribe to Kinect v2 sd camera_info to get image frame height and width
        rospy.Subscriber('/kinect2/qhd/camera_info', CameraInfo, self.camera_data, queue_size=1)
        rospy.Subscriber("/kinect2/qhd/image_color_rect", Image, self.image_callback, queue_size=1)

        self.rate.sleep()

    def camera_data(self, data):
        # set values on the parameter server
        rospy.set_param('camera_link', data.header.frame_id)  # kinect2_ir_optical_frame
        rospy.set_param('camera_height', data.height)         # sd height is 424 / qhd height is 540
        rospy.set_param('camera_width', data.width)           # sd width is 512 / qhd width is 960

        # set values for local variables
        self.cam_height = data.height
        self.cam_width = data.width

    def image_callback(self,data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)


        h_margin = rospy.get_param("/crazyflie_tracker/h_margin")
        s_margin = rospy.get_param("/crazyflie_tracker/s_margin")
        v_margin = rospy.get_param("/crazyflie_tracker/v_margin")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_green = np.array([143 / 2 - h_margin, 63 * 255 / 100 - s_margin, 47 * 255 / 100 - v_margin], np.uint8)
        upper_green = np.array([143 / 2 + h_margin, 63 * 255 / 100 + s_margin, 47 * 255 / 100 + v_margin], np.uint8)
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # dilate and erode with kernel size 11x11
        cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((11,11)))

        # find all of the contours in the mask image
        cImage, contours, heirarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        self.contourLength  = len(contours)

        # Check for at least one target found
        if self.contourLength < 1:
           print "No target found"

        else:                       # target found

           ## Loop through all of the contours, and get their areas
           area = [0.0]*len(contours)
           for i in range(self.contourLength):
              area[i] = cv2.contourArea(contours[i])

           #### Target #### the largest "pink" object
           target_image = contours[area.index(max(area))]

           # Using moments find the center of the object and draw a red outline around the object
           target_m = cv2.moments(target_image)
           #self.target_u = int(target_m['m10']/target_m['m00'])
           #self.target_v = int(target_m['m01']/target_m['m00'])
           points = cv2.minAreaRect(target_image)
           box = cv2.boxPoints(points)
           box = np.int0(box)
           cv2.drawContours(image, [box], 0, (0, 0, 255), 2)
           #rospy.loginfo("Center of target is x at %d and y at %d", int(self.target_u), int(self.target_v))

           #self.target_found = True               # set flag for depth_callback processing

           # show image with target outlined with a red rectangle
           cv2.imshow ("Target", image)
           cv2.waitKey(3)

def main(args):
    tracker = CrazyflieTracker()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
