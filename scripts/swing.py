#!/usr/bin/env python

import rospy
import tf
from pid import PID
from geometry_msgs.msg import PoseStamped, Twist
from std_srvs.srv import Empty
from std_msgs.msg import String
import time

class CF_Controller():

   """Controller to fly a Crazyflie in a space with Kinect v2 position feedback"""
   """Adapted to Python from Wolfgang Hoenig's controller.cpp in package crazyflie_controller"""

   def __init__(self):

      self.ff = 0.45                                   # adjustment for takeoff thrust
      self.acc_t = 0.0
      self.state = 0

      # tuple of Crazyflie flight states
      self._cf_state = 'hover'

      # Initialize the tf listener
      self.listener = tf.TransformListener()

      # initialize publisher for crazyflie command velocity (geometry_Twist)
      self.fly = Twist()                           # set the fly command
      self.velocity_pub = rospy.Publisher ('/crazyflie/cmd_vel', Twist, queue_size=1)
      self.state_pub = rospy.Publisher('/crazyflie/state', String, queue_size=1)

   # This process handles all of the flight command messages. 
   def iteration(self, event):

      try:
         # delta time is in a fraction of a second (0.02 sec for 50 Hz)
         dt = float(rospy.Time.to_sec(event.current_real)) - float(rospy.Time.to_sec(event.last_real))

      except (AttributeError, TypeError):
         dt = 0

      self.state_pub.publish(String(self._cf_state))

      # y => roll
      # x => pitch
      self.acc_t += dt
      rospy.loginfo('acc={}, dt={}'.format(self.acc_t, dt));
      if self.state == 1:
        self.fly.linear.x = 0.0
        self.fly.linear.y = 15.0
        self.fly.linear.z = 32767.0
        self.velocity_pub.publish(self.fly)
        if self.acc_t > 0.5:
          self.acc_t = 0
          self.state = 2
      elif self.state == 2:
        self.fly.linear.x = 0.0
        self.fly.linear.y = -15.0
        self.fly.linear.z = 32767.0
        self.velocity_pub.publish(self.fly)
        if self.acc_t > 0.5:
          self.acc_t = 0
          self.state = 1
      else:
        self.fly.linear.x = 0.0
        self.fly.linear.y = 0.0
        self.fly.linear.z = 32767.0
        self.velocity_pub.publish(self.fly)
        if self.acc_t > 1.0:
          self.acc_t = 0
          self.state = 1

if __name__ == '__main__':

   # initialize ROS node
   rospy.init_node('crazyflie_controller', anonymous=True)

   # set parameters on parameter server
   cf_frame = rospy.get_param("~frame", "crazyflie/base_link")
   frequency = rospy.get_param("frequency", 50.0)

   time.sleep(5)
   # start up the controller node and run until shutdown by interrupt
   try:
      controller = CF_Controller()
      rospy.Timer(rospy.Duration(1.0/frequency), controller.iteration)

      rospy.spin()                  # keep process alive

   except rospy.ROSInterruptException:
      rospy.loginfo("CF_controller node terminated.")

