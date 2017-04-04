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
        self.pub_tf = tf.TransformBroadcaster()
        self.bridge = CvBridge()

        # publish transform rate at 50Hz
        self.rate = rospy.Rate(50.0)

        self.target_u = 0
        self.target_v = 0
        self.target_d = 0

        self.last_d = 0


        #self.r = 0
        self.y = 0
        self.r = -1.5708
        #self.y = -3.1415
        self.p = 0

        # subscribe to kinect image messages
        rospy.wait_for_message("/camera/rgb/camera_info", CameraInfo)
        rospy.wait_for_message("/camera/depth/camera_info", CameraInfo)

        # Subscribe to Realsense R200 camera_info to get image frame height and width
        rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.camera_data, queue_size=1)
        rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.image_callback, queue_size=1)
        rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.depth_camera_data, queue_size=1)
        rospy.Subscriber("/camera/depth/image", Image, self.depth_callback, queue_size=1)

        self.rate.sleep()


    def camera_data(self, data):
        # set values on the parameter server
        rospy.set_param('camera_link', data.header.frame_id)  # camera_rgb_optical_frame
        rospy.set_param('camera_height', data.height)         # sd height is 424 / qhd height is 540
        rospy.set_param('camera_width', data.width)           # sd width is 512 / qhd width is 960

        # set values for local variables
        self.camera_link = data.header.frame_id
        self.cam_height = data.height
        self.cam_width = data.width
        # rospy.loginfo("camera: width=%d, height=%d", int(self.cam_width), int(self.cam_height))


    def depth_camera_data(self, data):
        # set values on the parameter server
        rospy.set_param('camera_depth_height', data.height)         # sd height is 424 / qhd height is 540
        rospy.set_param('camera_depth_width', data.width)           # sd width is 512 / qhd width is 960

        # set values for local variables
        self.cam_depth_height = data.height
        self.cam_depth_width = data.width
        # rospy.loginfo("camera depth: width=%d, height=%d", int(self.cam_depth_width), int(self.cam_depth_height))


    def image_callback(self,data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)


        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # duck
        # lower=[ 17  36 229], upper=[ 72 146 255]

        # blue light
        # lower=[ 88 130 212], upper=[128 210 255]
        lower = np.array([88, 130, 229], np.uint8)
        upper = np.array([128, 210, 255], np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        # dilate and erode with kernel size 11x11
        cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((11,11)))

        # find all of the contours in the mask image
        cImage, contours, heirarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        self.contourLength  = len(contours)

        # Check for at least one target found
        if self.contourLength < 1:
           print "No target found"

        else:
            # target found
            ## Loop through all of the contours, and get their areas
            area = np.array([0.0]*len(contours))
            for i in range(self.contourLength):
               area[i] = cv2.contourArea(contours[i])

            #### Target #### the largest "pink" object
            # target_image = contours[area.index(max(area))]
            indicies = np.argpartition(-area, 1)[:2]
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
                # rospy.loginfo("target is x at %d and y at %d", int(target_u), int(target_v))
                tu += target_u
                tv += target_v
            self.target_u = int(tu / 2)
            self.target_v = int(tv / 2)
            # rospy.loginfo("Center of target is x at %d and y at %d", int(self.target_u), int(self.target_v))

            self.target_found = True               # set flag for depth_callback processing

            # show image with target outlined with a red rectangle
            cv2.imshow ("Target", image)

            # TODO: save image when pressed 's' key
            cv2.waitKey(3)


    def depth_callback(self, msg):

        # create OpenCV depth image using default passthrough encoding
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            print(e)

        # using box (u, v) position, find depth value of Crazyflie point
        target_depth = depth_image[int(self.target_v * self.cam_depth_height / self.cam_height), int(self.target_u * self.cam_depth_width / self.cam_width)]

        if np.isnan(target_depth) or target_depth == 0:
            self.target_d = self.last_d
        else:
            self.last_d = target_depth
        # rospy.loginfo("Depth: x at %d  y at %d  z at %f", int(self.target_u), int(self.target_v), self.target_d)

        # publish Crazyflie tf transform
        self.update_cf_transform(self.target_u, self.target_v, self.target_d)

    def update_cf_transform(self, x, y, z):

        # send position as transform from the parent "camera_rgb_optical_frame" to the
        # child "crazyflie/base_link" (described by crazyflie.urdf.xacro)"
        self.pub_tf.sendTransform((x,
                                   y,
                                   z),
                                tf.transformations.quaternion_from_euler(self.r, self.p, self.y),
                                rospy.Time.now(),
                                "crazyflie/base_link", self.camera_link)
#        self.pub_tf.sendTransform((0.6 - y / 640.0,
#                                   - z,
#                                   0),
#                                tf.transformations.quaternion_from_euler(self.r, self.p, self.y),
#                                rospy.Time.now(),
#                                "crazyflie/base_link", self.camera_link)
#        self.pub_tf.sendTransform(( x / 300.0,
#                                    y / 500.0,
#                                    z),
#                                tf.transformations.quaternion_from_euler(self.r, self.p, self.y),
#                                rospy.Time.now(),
#                                "crazyflie/base_link", self.camera_link)
        rospy.loginfo("Send CF transform %f %f %f", x, y, z)

def main(args):
    tracker = CrazyflieTracker()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
