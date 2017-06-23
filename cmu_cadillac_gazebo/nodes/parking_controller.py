#!/usr/bin/env python
# license removed for brevity
""" A controller for parallel parking

Publihed Topics:
    cmu_cadillac/ackermann_cmd (ackermann_msgs/AckermannDriveStamped)
        Ackermann command. It contains the vehicle's desired speed and steering
        angle.

Subscribed Topics:
	ground_truth/odom(nav_msgs/Odometry)
		Odometry information.

	sonar_scan(sensor_msgs/Range)
		Ultrasonic sensor to determine empty spot.
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
from sensor_msgs.msg import Range
from std_msgs.msg import String
from sensor_msgs.msg import Range
class parking_ctrl(object):

	def __init__(self):
		"""Initialize this parking_ctrl."""
		rospy.init_node("parking_controller")
		self.x = 0	#current x
		self.y = 0	#current y
		self.x0 = 9	#initial x
		self.y0 = -5 #initial y	
		self.xd = 0 #final x
		self.yd = -8.9 #final y
		#Centres of circles 1 and 2
		self.x1 = 0 
		self.y1 = 0
		self.x2 = 0
		self.y2 = 0
		self.xc = 0 #meeting point of the 2 circles
		self.xp = {} #x path
		self.yp = {} #y path
		self.distance = 0 #distance from obstacle
		self.flag = 0 
		self.flag1 = 0
		self.flag2 = 0
		self.flag4 = 0
		self.flag5 = 0
		self.l = 2.807 #wheelbase length
		self.w = 1.627 #axle length
		self.theta = {}
		self.psi=0 #steering angle req for 1st circle
		self.roll = 0
		self.pitch = 0
		self.yaw = 0
		self.rturn = 4.6473		#min turning radius wrt centre of rear wheel axle
		self.psimin=0.5434	#min avg steering angle
		self.steer_ang = 0.0      # Steering angle
		self.steer_ang_vel = 0.0  # Steering angle velocity
		self.speed = 0.0
		self.accel = 0.0          # Acceleration
		self.jerk = 0.0
		self.pub = rospy.Publisher('cmu_cadillac/ackermann_cmd', AckermannDriveStamped, queue_size = 10)
		rospy.Subscriber('odom', Odometry, self.odom_cb, queue_size = 10)
		rospy.Subscriber('sonar_scan', Range, self.sonar_cb, queue_size = 10)
		self.ack_cmd = AckermannDriveStamped()
		self.ack_cmd.header.seq = 0
		self.ack_cmd.header.frame_id = ''
		self.ack_cmd.header.stamp = rospy.Time.now()
		self.rate = rospy.Rate(10) # 1hz

	def odom_cb(self,odom):
		self.x = odom.pose.pose.position.x- math.cos(self.yaw)*self.l/2 #for offset wrt to rear wheel axle
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

	def sonar_cb(self,sonar_scan):
		self.distance = sonar_scan.range

	def spin(self):
		while not rospy.is_shutdown():
			self.park()
			# print self.yaw
			self.ack_cmd.drive.steering_angle = self.steer_ang
			self.ack_cmd.drive.steering_angle_velocity = self.steer_ang_vel 
			self.ack_cmd.drive.speed = self.speed 
			self.ack_cmd.drive.acceleration = self.accel
			self.ack_cmd.drive.jerk = self.jerk
			#print self.xc
			self.pub.publish(self.ack_cmd)
			self.rate.sleep()
			#rospy.spin()

	def park(self):
		#print self.x
		if self.flag4<2:
			if self.flag5 == 0:
				if self.distance>4:
					x1 = self.x + 1.4
					self.flag4 += 1
					self.flag5 = 1
					self.xd = x1+1+0.8
			else:
				if self.distance<4:
					self.flag4 += 1
					x2 = self.x
					#self.xd = ((x2+x1)/2) - 1.4
					
					print self.xd
		#print x1
		if self.flag == 0:
			if self.x<self.x0:
				if self.y>-0.3:
					self.go_straight()
			else:
				self.path_planner()
		else:
			self.path_planner()		


	def turn_right(self):
		self.steer_ang = -self.psi
		self.steer_ang_vel = 1
		self.speed = -1
		self.accel = 1
		self.jerk = 0

	def turn_left(self):
		self.steer_ang = self.psimin
		self.steer_ang_vel = 1
		self.speed = -1
		self.accel = 1
		self.jerk = 0
	def go_straight(self):
		self.steer_ang = 0
		self.steer_ang_vel = 1
		if self.y>-0.3:
			if self.x<self.x0-4:
				self.speed = 2
			else:
				self.speed = 0.1
		else:
			self.speed = 0.5
		self.accel = 1
		self.jerk = 0
	def stop(self):
		self.steer_ang = 0
		self.steer_ang_vel = 1
		self.speed = 0
		self.accel = 1
		self.jerk = 0

	def path_planner(self):	
		self.flag = 1
		# self.stop()
		# self.pub.publish(self.ack_cmd)
		# time.sleep(0.2)
		r1=-self.rturn - (pow((self.xd-self.x0),2) +pow((self.yd-self.y0),2))/(2*(self.yd-self.y0))
		
		self.alpha = math.asin((self.x0-self.xd)/(r1+self.rturn))
		self.xc = self.x0+(r1*math.cos(1.5707+self.alpha))
		self.psi = math.atan(self.l/r1)
		self.x1 = self.x0
		self.y1 = self.y0-r1
		self.x2 = self.xd
		self.y2 = self.yd+self.rturn
		i=0
		t=0
		while i<=self.alpha:
			self.xp[t]=self.x1+(r1*math.cos(1.5707+i));
			self.yp[t]=self.y1+(r1*math.sin(1.5707+i));
			self.theta[t]=i
			t=t+1
			i=i+0.02
		while i>=0:
			self.xp[t]=self.x2+(self.rturn*math.cos(4.71239+i));
			self.yp[t]=self.y2+(self.rturn*math.sin(4.71239+i));
			self.theta[t]=i
			t=t+1;
			i=i-0.02;

		self.path_follower()

	def path_follower(self):
		if self.flag2==0:
			if self.x>=self.xd:
				if self.flag1==0:
					if self.yaw<self.alpha:
						self.turn_right()
					else:
						self.flag1=1
				else:
					if self.yaw>0:
						self.turn_left()
					else:
						self.stop()
						self.flag2=1
			else:
				self.stop()
				self.flag2=1
		else:
			if self.x<=1.3:
				self.go_straight()
			else:
				self.stop()


if __name__ == '__main__':
	ctrl = parking_ctrl()
	try:
		ctrl.spin()
	except rospy.ROSInterruptException:
		pass
