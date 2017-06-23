#!/usr/bin/env python
# license removed for brevity
""" A controller for head-in parking

Publihed Topics:
    cmu_cadillac/ackermann_cmd (ackermann_msgs/AckermannDriveStamped)
        Ackermann command. It contains the vehicle's desired speed and steering
        angle.

Subscribed Topics:
	ground_truth/odom(nav_msgs/Odometry)
		Odometry information.

"""		

import math
import numpy
import threading
import time

from math import pi

import rospy
import tf

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from controller_manager_msgs.srv import ListControllers
from std_msgs.msg import String

class parking_ctrl(object):

	def __init__(self):
		"""Initialize this parking_ctrl."""
		rospy.init_node("parking_controller")
		self.x = 0	#current x
		self.y = 0	#current y
		self.x0 = -22	#initial x
		self.y0 = 31 #initial y	
		self.xd = -15.5 #final x
		self.obs1 = [-18,36]
		self.obs2 = [-18,39.2]
		self.yd = (self.obs1[1]+self.obs2[1])/2 #final y
		#Centres of circles 1 and 2
		self.x_initial=0
		self.y_initial=0
		self.y1 = 0
		self.x2 = 0
		self.y2 = 0
		self.xc = 0 #meeting point of the 2 circles
		self.xp = {} #x path
		self.yp = {} #y path
		self.flag = 0 
		self.flag1 = 0
		self.flag2 = 0
		self.flag3 = 0
		self.correction_flag=1
		self.l = 2.807 #wheelbase length
		self.w = 1.627 #axle length
		self.ext = 1.08 #extension after wheelbase at front and rear 
		self.theta = {}
		self.psi=0 #steering angle req for 1st circle
		self.roll = 0
		self.pitch = 0
		self.yaw = 0
		self.rturn = 3.9395		#min turning radius wrt centre of rear wheel axle
		self.psimin = 0.6188	#min steering angle
		self.steer_ang = 0.0      # Steering angle
		self.steer_ang_vel = 0.0  # Steering angle velocity
		self.speed = 0.0
		self.accel = 0.0          # Acceleration
		self.jerk = 0.0
		self.pub = rospy.Publisher('cmu_cadillac/ackermann_cmd', AckermannDriveStamped, queue_size = 10)
		rospy.Subscriber('odom', Odometry, self.odom_cb, queue_size = 10)
		self.ack_cmd = AckermannDriveStamped()
		self.ack_cmd.header.seq = 0
		self.ack_cmd.header.frame_id = ''
		self.ack_cmd.header.stamp = rospy.Time.now()
		self.rate = rospy.Rate(10) # 1hz

	def odom_cb(self,odom):
		self.x = odom.pose.pose.position.x- math.cos(self.yaw)*self.l/2 #for offset wrt to rear wheel axle
		if self.flag == 0:
			self.x_initial=self.x
			self.y_initial=self.y
			self.flag=1
		self.y = odom.pose.pose.position.y- math.sin(self.yaw)*self.l/2
		q0 = odom.pose.pose.orientation.x
		q1 = odom.pose.pose.orientation.y
		q2 = odom.pose.pose.orientation.z
		q3 = odom.pose.pose.orientation.w
		# self.yaw = math.atan((2*(q0*q1 + q2*q3))/(1-2*(pow(q2,2)+pow(q3,2))))
		# self.xd = self.x0 - self.rturn
		# self.yd = self.y0 - self.rturn
		(self.roll, self.pitch, self.yaw) = tf.transformations.euler_from_quaternion([q0, q1, q2, q3])

		# print int(round(self.yaw))



	def spin(self):
		while not rospy.is_shutdown():
			if self.x_initial==0:
				continue
			wp = self.obs2[1]-self.obs1[1]
			rbr = self.rturn - (0.955)
			rfl = pow(pow((self.rturn+0.955),2)+pow(3.887,2),0.5)
			psifl = math.atan(3.887/(self.rturn+0.955))
			psicln = 3.1416-psifl-math.asin((self.obs2[1]+self.rturn-self.yd)/rfl)
			self.xd = self.obs2[0]-(rfl*math.cos(3.14-psicln-psifl))-0.7
			self.y0 = self.yd-self.rturn
			self.x0 = self.xd-self.rturn
			
			if self.correction_flag==1 and self.x_initial>self.x0:
				self.correction()
			elif self.x_initial<self.x0:
				self.x0 = self.x_initial
				self.park()
			else:
				self.park()
			# print self.yaw
			self.ack_cmd.drive.steering_angle = self.steer_ang
			self.ack_cmd.drive.steering_angle_velocity = self.steer_ang_vel 
			self.ack_cmd.drive.speed = self.speed 
			self.ack_cmd.drive.acceleration = self.accel
			self.ack_cmd.drive.jerk = self.jerk
			#print self.xc
			self.pub.publish(self.ack_cmd)
			if self.speed == 0:
				time.sleep(1)  #makes vehicle stop for a second to nullify inertia due to previous motion
			self.rate.sleep()
			#rospy.spin()

	def correction(self):

		if self.flag1==0:
			if self.yaw<=1.5708+math.acos((self.x0-self.x_initial+2*(self.rturn))/(2*(self.rturn))):
				self.turn_left()
			else:
				self.flag1=1
		else:
			if self.yaw>1.5708:
				self.turn_right1()
			else:
				self.stop()
				self.correction_flag=0

	def park(self):
		if self.flag2 == 0:
			if self.flag3==0:
				if self.y<self.y0:
					self.go_straight()
				else:
					self.stop()
					self.flag3=1
			else:
				if self.y>self.y0:
					self.reverse()
				else:
					self.stop()
					self.flag2=1
		else:
			if self.yaw>0:
				self.turn_right()
			elif self.x-1.08<self.obs1[0]:
				self.go_straight()
			else:
				self.stop()


	def turn_right(self):
		self.steer_ang = -self.psimin
		self.steer_ang_vel = 1
		self.speed = 0.5
		self.accel = 1
		self.jerk = 0

	def turn_right1(self):
		self.steer_ang = -self.psimin
		# self.steer_ang = -math.atan(self.l/(self.rturn+1))
		self.steer_ang_vel = 1
		self.speed = 0.5
		self.accel = 1
		self.jerk = 0

	def turn_left(self):
		# self.steer_ang = math.atan(self.l/(self.rturn+1))
		self.steer_ang = self.psimin
		self.steer_ang_vel = 1
		self.speed = 0.5
		self.accel = 1
		self.jerk = 0
	def go_straight(self):
		self.steer_ang = 0
		self.steer_ang_vel = 1
		self.speed = 1
		self.accel = 1
		self.jerk = 0

	def reverse(self):
		self.steer_ang = 0
		self.steer_ang_vel = 1
		self.speed = -0.2
		self.accel = 1
		self.jerk = 0

	def stop(self):
		self.steer_ang = 0
		self.steer_ang_vel = 1
		self.speed = 0
		self.accel = 1
		self.jerk = 0

if __name__ == '__main__':
	ctrl = parking_ctrl()
	try:
		ctrl.spin()
	except rospy.ROSInterruptException:
		pass
