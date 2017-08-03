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
      self.thrust = 0.0                                # initialize thrust to 0
      self.target = False                              # initialize target sighting to False

      # tuple of Crazyflie flight states
      self.flight_state = ['idle', 'takeoff', 'hover', 'flight', 'land']    
      self._cf_state = 'idle'                          # initial flight state
      self.takeoff_position = [0.0, 0.0, 0.0]          # inital takeoff and hover positions
      self.hover_position = [0.0, 0.0, 0.0]
      self.last_depth = 0.0                # variable to keep non-zero value for camera depth 

      # initialize services for land and takeoff # emergency is handled by crazyflie_server.cpp in crazyflie_ros/crazyflie_driver package
      s1 = rospy.Service("/crazyflie/land", Empty, self._Land)
      s2 = rospy.Service("/crazyflie/takeoff", Empty, self._Takeoff)

      # subscribe to target pose
      self.target_position = PoseStamped()
      rospy.Subscriber('target_pose', PoseStamped, self._update_target_pose, queue_size=1)

      # Initialize the tf listener
      self.listener = tf.TransformListener()

      # initialize publisher for crazyflie command velocity (geometry_Twist)
      self.fly = Twist()                           # set the fly command
      self.velocity_pub = rospy.Publisher ('/crazyflie/cmd_vel', Twist, queue_size=1)

      self.state_pub = rospy.Publisher('/crazyflie/state', String, queue_size=1)
      # get camera info from parameter server
      self.camera_height = 480 #rospy.get_param('camera_height')      # sd height is 424; qhd height is 540
      self.camera_width = 640 #rospy.get_param('camera_width')        # sd width is 512; qhd width is 960

      # create flight PID controllers in X, Y and Z 
      self.m_pidX = PID(rospy.get_param("~PIDs/X/kp"),
                        rospy.get_param("~PIDs/X/kd"),
                        rospy.get_param("~PIDs/X/ki"),
                        rospy.get_param("~PIDs/X/minOutput"),
                        rospy.get_param("~PIDs/X/maxOutput"),
                        rospy.get_param("~PIDs/X/integratorMin"),
                        rospy.get_param("~PIDs/X/integratorMax"))
      self.m_pidY = PID(rospy.get_param("~PIDs/Y/kp"),
                        rospy.get_param("~PIDs/Y/kd"),
                        rospy.get_param("~PIDs/Y/ki"),
                        rospy.get_param("~PIDs/Y/minOutput"),
                        rospy.get_param("~PIDs/Y/maxOutput"),
                        rospy.get_param("~PIDs/Y/integratorMin"),
                        rospy.get_param("~PIDs/Y/integratorMax"))
      self.m_pidZ = PID(rospy.get_param("~PIDs/Z/kp"),
                        rospy.get_param("~PIDs/Z/kd"),
                        rospy.get_param("~PIDs/Z/ki"),
                        rospy.get_param("~PIDs/Z/minOutput"),
                        rospy.get_param("~PIDs/Z/maxOutput"),
                        rospy.get_param("~PIDs/Z/integratorMin"),
                        rospy.get_param("~PIDs/Z/integratorMax"))
      self.m_pidYaw = PID(rospy.get_param("~PIDs/Yaw/kp"),
                          rospy.get_param("~PIDs/Yaw/kd"),
                          rospy.get_param("~PIDs/Yaw/ki"),
                          rospy.get_param("~PIDs/Yaw/minOutput"),
                          rospy.get_param("~PIDs/Yaw/maxOutput"),
                          rospy.get_param("~PIDs/Yaw/integratorMin"),
                          rospy.get_param("~PIDs/Yaw/integratorMax"))


   # This service processes the Takeoff request.
   def _Takeoff(self, req):
      rospy.loginfo("Takeoff requested!")
      self._cf_state = 'takeoff'
      self.thrust = 35000
      return ()
 
   # This service processes the Land request.
   def _Land(self, req):
      rospy.loginfo("Landing requested!")
      self._cf_state = 'land'
      return ()


   # This callback function puts the target PoseStamped message into a local variable.
   def _update_target_pose(self, msg):
      self.target_position = msg
      self.target = True

   # This function calls the reset function for all the PID controllers.
   def _pidReset(self):
      self.m_pidX.reset()
      self.m_pidY.reset()
      self.m_pidZ.reset()
      self.m_pidYaw.reset()


   # This process handles all of the flight command messages. 
   def iteration(self, event):

      try:
         # delta time is in a fraction of a second (0.02 sec for 50 Hz)
         dt = float(rospy.Time.to_sec(event.current_real)) - float(rospy.Time.to_sec(event.last_real))

      except (AttributeError, TypeError):
         dt = 0

      self.state_pub.publish(String(self._cf_state))

      ##### Idle ########
      if self._cf_state == 'idle':
         # set command velocity message values to zero; thrust value to zero
         self.fly.linear.x = 0.0
         self.fly.linear.y = 0.0
         self.fly.linear.z = 0.0
         self.thrust = 0.0
         self._cf_state = 'takeoff'

      ##### Hover ########
      # use x, y, and z PID controllers to keep Crazyflie's position the same as the hover position
      #
      elif self._cf_state == 'hover':

         self.fly.linear.x = 0.0             # set cmd_vel x and y to 0
         self.fly.linear.y = 0.0
         self.fly.linear.z = 60000.0

      ##### Takeoff ########
      # increase the thrust (z) value in the command velocity message until takeoff is achieved
      #
      elif self._cf_state == 'takeoff':

         self.fly.linear.x = 0.0             # set cmd_vel x and y to 0
         self.fly.linear.y = 0.0
         self.fly.linear.z = 35000.0
         self._cf_state = 'hover'
         self.thrust = 35000.0

      ##### Land ########
      elif self._cf_state == 'land':

         # reduce pitch and roll to zero maintain thrust at 30000 for Crazyflie landing
         self.fly.linear.x = 0.0
         self.fly.linear.y = 0.0
         self.fly.linear.z = 30000.0
         self.thrust = 30000.0

         # log achievement and change state to idle
         rospy.loginfo("Landing achieved!")
         self._cf_state = 'idle'


      # publish command velocity message to Crazyflie
      self.fly.linear.x = 0.0
      self.fly.linear.y = 0.0
      self.fly.linear.z = 32767.0
      self.velocity_pub.publish(self.fly)

      # log flight state
      # rospy.loginfo("CF_state is %s", str(self._cf_state))


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

