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

	sonar_xxxxx_xxxxx_xxxxxx(sensor_msgs/Range)
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
		self.x0 = 0	#initial x
		self.y0 = 0 #initial y	
		self.xd = 0 #final x
		self.yd = 0 #final y
		#Centres of circles 1 and 2
		self.x1 = 0 
		self.y1 = 0
		self.x2 = 0
		self.y2 = 0
		self.xc = 0 #meeting point of the 2 circles
		self.xfr = [] #front right x path
		self.yfr = [] #front right y path
		self.xfl = [] #front left x path
		self.yfl = [] #front left y path
		self.right_limit1 = [4.5]
		self.right_limit2 = []
		self.left_limit = []
		self.perp_distance_right = 0 #distance from obstacle to the right side
		self.perp_distance_left = 0 #distance from obstacle to the left side
		# self.front_distance = 0 #front sonar
		# self.rear_distance = 0 #rear sonar
		self.flag = 0 
		self.flag1 = 0
		self.flag2 = 0
		self.flag3 = 0
		self.check_start_pos_flag = 1
		self.compute_space = 1
		self.check_collision_flag = 0
		self.shift_flag = 1
		self.shift_yd = 0
		self.correction_flag = 0
		self.flag5 = 0
		self.flag6 = 0
		self.flag7 = 0
		self.find_x1_obs = 1
		self.find_x2_obs = 0
		self.n = 0 #number of times yd is shifted
		self.l = 2.807 #wheelbase length
		self.w = 1.627 #axle length
		self.theta = []
		self.psi=0 #steering angle req for 1st circle
		self.roll = 0
		self.pitch = 0
		self.yaw = 0
		self.rturn = 3.9395		#min turning radius wrt centre of rear wheel axle
		self.psimin=0.6188	#min avg steering angle
		self.steer_ang = 0.0      # Steering angle
		self.steer_ang_vel = 0.0  # Steering angle velocity
		self.speed = 0.0
		self.accel = 0.0          # Acceleration
		self.jerk = 0.0
		self.pub = rospy.Publisher('cmu_cadillac/ackermann_cmd', AckermannDriveStamped, queue_size = 10)
		rospy.Subscriber('odom', Odometry, self.odom_cb, queue_size = 10)
		rospy.Subscriber('sonar_front_right_perpendicular', Range, self.sonar_front_right_perpendicular_cb, queue_size = 10)
		rospy.Subscriber('sonar_front_left_perpendicular', Range, self.sonar_front_left_perpendicular_cb, queue_size = 10)
		# rospy.Subscriber('sonar_front_right_parallel', Range, self.sonar_front_right_parallel_cb, queue_size = 10)
		# rospy.Subscriber('sonar_rear_right_parallel', Range, self.sonar_rear_right_parallel_cb, queue_size = 10)
		self.ack_cmd = AckermannDriveStamped()
		self.ack_cmd.header.seq = 0
		self.ack_cmd.header.frame_id = ''
		self.ack_cmd.header.stamp = rospy.Time.now()
		self.rate = rospy.Rate(10) # 1hz

	def odom_cb(self,odom):
		q0 = odom.pose.pose.orientation.x
		q1 = odom.pose.pose.orientation.y
		q2 = odom.pose.pose.orientation.z
		q3 = odom.pose.pose.orientation.w
		(self.roll, self.pitch, self.yaw) = tf.transformations.euler_from_quaternion([q0, q1, q2, q3])
		self.x = odom.pose.pose.position.x- math.cos(self.yaw)*self.l/2 #for offset wrt to rear wheel axle
		self.y = odom.pose.pose.position.y- math.sin(self.yaw)*self.l/2
		# self.yaw = math.atan((2*(q0*q1 + q2*q3))/(1-2*(pow(q2,2)+pow(q3,2))))
		# self.xd = self.x0 - self.rturn
		# self.yd = self.y0 - self.rturn
		

		# print int(round(self.yaw))

	def sonar_front_right_perpendicular_cb(self,sonar_front_right_perpendicular):
		self.perp_distance_right = sonar_front_right_perpendicular.range

	def sonar_front_left_perpendicular_cb(self,sonar_front_left_perpendicular):
		self.perp_distance_left = sonar_front_left_perpendicular.range


	# def sonar_front_right_parallel_cb(self,sonar_front_right_parallel):
	# 	self.front_distance = sonar_front_right_parallel.range

	# def sonar_rear_right_parallel_cb(self,sonar_rear_right_parallel):
	# 	self.rear_distance = sonar_rear_right_parallel.range

	def spin(self):
		while not rospy.is_shutdown():
			if self.perp_distance_right==0 or self.perp_distance_left==0:
				continue
			elif self.check_start_pos_flag == 1:
				if self.perp_distance_right<=4:
					self.free_space_flag = 0
					self.check_start_pos_flag =0
				elif self.perp_distance_right>4:
					self.free_space_flag = 1
					self.check_start_pos_flag =0

			if self.flag == 0:
				self.go_straight()
				self.left_limit.append(self.y+self.perp_distance_left+0.955)
			else:
				self.park()

			if self.compute_space == 1:
				if self.free_space_flag == 1:
					if self.find_x1_obs == 1:
						if self.perp_distance_right>=4:
							self.x1_obs = self.x + 3.887
							self.xd = self.x1_obs+1.08+0.25
							self.find_x1_obs = 0
							self.find_x2_obs = 1

					if self.find_x2_obs == 1:
						if self.perp_distance_right<=4:
							self.x2_obs = self.x + 3.887
							self.y2_obs = self.y - 0.955 - self.perp_distance_right + 0.1 
							self.yd = self.y2_obs-0.955
							self.find_x2_obs = 0
							dist = self.x2_obs - self.x1_obs
							print dist
							if dist >= 5.5: #min parking space req
								self.compute_space = 0
								self.check_collision_flag = 1
							else:
								self.compute_space = 1
								self.find_x1_obs = 1
								self.find_x2_obs = 0
								self.free_space_flag = 0
						elif (self.x+3.887-self.x1_obs)>9:
							self.x2_obs = self.x + 3.887
							self.y2_obs = self.y - 0.955 - self.perp_distance_right
							self.yd = self.y1_obs-0.955
							self.find_x2_obs = 0
							dist = self.x2_obs - self.x1_obs
							if dist >= 9: #safe parking space
								self.compute_space = 0
							else:
								self.compute_space = 1
								self.find_x1_obs = 1
								self.find_x2_obs = 0

				else:
					if self.find_x1_obs == 1:
						if self.perp_distance_right<=4:
							self.right_limit1.append(self.perp_distance_right)
							self.x1_obs = self.x + 3.887
							self.xd = self.x1_obs+1.08+0.25 #0.25 is the safe distance
						if self.perp_distance_right>4:
							self.y1_obs = self.y - 0.955 - min(self.right_limit1)  #min is used to consider widest part of the vehicle
							self.yd = self.y1_obs-0.955 
							self.find_x1_obs = 0
							self.find_x2_obs = 1

					if self.find_x2_obs == 1:
						if self.perp_distance_right<=4:
							self.x2_obs = self.x + 3.887
							self.y2_obs = self.y - 0.955 - self.perp_distance_right + 0.4 # +0.4 is the compensation for the difference between the sensed portion and the widest portion of the vehicle 
							self.find_x2_obs = 0
							dist = self.x2_obs - self.x1_obs
							print dist
							if dist >= 5.5: #min parking space req
								self.compute_space = 0
								self.check_collision_flag = 1
							else:
								self.compute_space = 1
								self.find_x1_obs = 1
								self.find_x2_obs = 0
								self.right_limit1 = []
						elif (self.x+3.887-self.x1_obs)>9:
							self.x2_obs = self.x + 3.887
							self.find_x2_obs = 0
							dist = self.x2_obs - self.x1_obs
							if dist >= 9: #safe parking space
								self.compute_space = 0
							else:
								self.compute_space = 1
								self.find_x1_obs = 1
								self.find_x2_obs = 0

			elif self.flag == 0:
				r1=-self.rturn - (pow((self.xd-self.x),2) +pow((self.yd-self.y),2))/(2*(self.yd-self.y))
				if r1>self.rturn:
					self.stop()
					self.x0 = self.x
					self.y0 = self.y
					if self.check_collision_flag == 1:
						self.check_collision()
					self.flag = 1



			# if self.compute_space == 1
			# 	if self.find_x1_obs == 1:
			# 		if self.perp_distance_right>4:
			# 			self.x1_obs = self.x + 3.887
			# 			self.find_x1_obs = 0
			# 	if self.find_x2_obs == 1:
			# 		if self.perp_distance_right<4:
			# 			self.x2_obs = self.x + 3.887
			# 			self.find_x2_obs = 0

			self.ack_cmd.drive.steering_angle = self.steer_ang
			self.ack_cmd.drive.steering_angle_velocity = self.steer_ang_vel 
			self.ack_cmd.drive.speed = self.speed 
			self.ack_cmd.drive.acceleration = self.accel
			self.ack_cmd.drive.jerk = self.jerk
			self.pub.publish(self.ack_cmd)
			if self.speed == 0:
				time.sleep(1)  #makes vehicle stop for a second to nullify inertia due to previous motion
			self.rate.sleep()
			#rospy.spin()

	def park(self):
		r1=-self.rturn - (pow((self.xd-self.x0),2) +pow((self.yd-self.y0),2))/(2*(self.yd-self.y0))
		
		self.alpha = math.asin((self.x0-self.xd)/(r1+self.rturn))
		self.xc = self.x0+(r1*math.cos(1.5707+self.alpha))
		self.psi = math.atan(self.l/r1)

		if self.flag1==0:
			if self.yaw<self.alpha:
				self.turn_right()
			else:
				self.flag1=1
		else:
			if self.flag7 == 0:
				if self.yaw>0:
					self.turn_left()
				else:
					self.flag7 = 1
			else:
				self.stop()
				if self.flag5==0:  
					if self.flag6==0:
						self.xd = self.x2_obs-3.887-0.3
						self.x0 = self.x
						self.flag5=1
					else:
						self.x0 = self.x
						self.xd = self.x1_obs+1.08+0.3
						self.flag5=1
				if self.correction_flag==1:					
					self.correction()

	def check_collision(self):
		while self.check_collision_flag==1:
			r1=-self.rturn - (pow((self.xd-self.x0),2) +pow((self.yd-self.y0),2))/(2*(self.yd-self.y0))
			rfrr = math.sqrt(pow((r1-0.955),2)+pow(3.887,2))
			rfrl = math.sqrt(pow((self.rturn+0.955),2)+pow(3.887,2))
			rflr = math.sqrt(pow((r1+0.955),2)+pow(3.887,2))
			psifrr = math.atan(3.887/(r1-0.955))
			psifrl = math.atan(3.887/(self.rturn+0.955))
			psiflr = math.atan(3.887/(r1+0.955))
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
				self.xfr.append(self.x1+(rfrr*math.cos(1.5707+i-psifrr)))
				self.yfr.append(self.y1+(rfrr*math.sin(1.5707+i-psifrr)))
				self.xfl.append(self.x1+(rflr*math.cos(1.5707+i-psiflr)))
				self.yfl.append(self.y1+(rflr*math.sin(1.5707+i-psiflr)))
				t=t+1
				i=i+0.02

			if max(self.yfl)>=(min(self.left_limit)-0.2):
				self.xfr = []
				self.yfr = []
				self.xfl = []
				self.yfl = []
				self.yd = self.yd+0.1
				self.n = self.n+1
				self.correction_flag = 1
				print 'collision'
				print self.yd
				continue
			
			while i>=0:
				self.xfr.append(self.x2+(rfrl*math.cos(4.71239+i+psifrl)))
				self.yfr.append(self.y2+(rfrl*math.sin(4.71239+i+psifrl)))
				if self.xfr[t]>=(self.x2_obs-0.4) and self.xfr[t]<=(self.x2_obs) and self.shift_flag==1:
					if self.yfr[t]<=self.y2_obs:
						print 'collision'
						self.shift_yd = self.y2_obs-self.yfr[t]+0.2
						print self.yd
						self.yd = self.yd + self.shift_yd 
						self.shift_flag = 0
						if self.shift_yd+(self.n*0.1)>=0.2:
							self.correction_flag = 1
						print self.shift_yd
						print self.yd
				t=t+1
				i=i-0.02	
				self.check_collision_flag = 0

	def correction(self):
		if self.flag6==0:
			if self.x<=((self.xd+self.x0)/2):
				self.turn_right_fwd()
			elif self.x<=self.xd:
				self.turn_left_fwd()
			else:
				self.stop()
				self.flag6=1
				self.flag5=0
		else:
			if self.x>=((self.xd+self.x0)/2):
				self.turn_right_bwd()
			elif self.x>=self.xd:
				self.turn_left_bwd()
			# if self.x>=self.xd:
			# 	self.reverse()
			else:
				self.stop()
				if self.y<=(self.yd-self.shift_yd-(self.n*0.1)):
					self.stop()
				else:
					self.flag6=0
					self.flag5=0
		# elif self.flag6==1:
		# 	if self.x>=((self.x0+self.xd)/2):
		# 		self.turn_right_bwd()
		# 	elif self.x>=self.xd:
		# 		self.turn_left_bwd()
		# 	else:
		# 		self.stop()
		# 		self.flag6=0
		# 		self.flag5=0
	# def correction(self):
	# 	if self.flag6==0:
	# 		self.correction_yd = self.yd
	# 		self.flag6 = 1

	# 	if self.correction_flag==0:
	# 		if self.front_distance>=0.25:
	# 			self.go_straight()
	# 		else:
	# 			self.stop()
	# 			self.x0 = self.x
	# 			if self.y-self.yd>0.2:
	# 				self.correction_flag = 1
	# 	else:
	# 		if self.x>=((self.xd+self.x0)/2):
	# 			self.turn_right()
	# 		else:
	# 			if self.x>self.xd:
	# 				self.turn_left()
	# 			else:
	# 				self.stop()
	# 				self.correction_flag = 0
	# 				self.flag6 = 0

		# elif self.correction_yd<self.y:
		# 			print self.xd
		# 			print self.yd
		# 			print self.x
		# 			print self.y
		# 	r1=-self.rturn - (pow((self.xd-self.x),2) +pow((self.correction_yd-self.y),2))/(2*(self.correction_yd-self.y))
		# 	self.alpha = math.asin((self.x-self.xd)/(r1+self.rturn))
		# 	self.psi = math.atan(self.l/r1)
		# 	if r1>=self.rturn:
		# 		if self.flag3==0:
		# 			if self.yaw<self.alpha:
		# 				self.turn_right()
		# 			else:
		# 				self.flag3=1
		# 		else:
		# 			if self.yaw>0:
		# 				self.turn_left()
		# 			else:
		# 				self.stop()
		# 				print self.xd
		# 				print self.yd
		# 				self.correction_flag = 0
		# 				self.flag6 = 0 			
		# 	else:
		# 		self.correction_yd = self.correction_yd+0.1



	def turn_right(self):
		self.steer_ang = -self.psi
		self.steer_ang_vel = 0.1
		self.speed = -0.6
		self.accel = 0.1
		self.jerk = 0

	def turn_left(self):
		self.steer_ang = self.psimin
		self.steer_ang_vel = 0.1
		self.speed = -0.4
		self.accel = 0.1
		self.jerk = 0
	def go_straight(self):
		self.steer_ang = 0
		self.steer_ang_vel = 0.1
		self.speed = 1
		self.accel = 0.1
		self.jerk = 0

	def go_straight_slow(self):
		self.steer_ang = 0
		self.steer_ang_vel = 0.1
		self.speed = 0.6
		self.accel = 0.1
		self.jerk = 0	

	def reverse(self):
		self.steer_ang = 0
		self.steer_ang_vel = 0.1
		self.speed = -0.3
		self.accel = 0.1
		self.jerk = 0

	def stop(self):
		self.steer_ang = 0
		self.steer_ang_vel = 1
		self.speed = 0
		self.accel = 1
		self.jerk = 0

	def turn_right_fwd(self):
		self.steer_ang = -self.psimin
		self.steer_ang_vel = 0.1
		self.speed = 0.2
		self.accel = 0.1
		self.jerk = 0

	def turn_left_fwd(self):
		self.steer_ang = self.psimin
		self.steer_ang_vel = 0.1
		self.speed = 0.2
		self.accel = 0.1
		self.jerk = 0

	def turn_right_bwd(self):
		self.steer_ang = -self.psimin
		self.steer_ang_vel = 0.1
		self.speed = -0.2
		self.accel = 0.1
		self.jerk = 0

	def turn_left_bwd(self):
		self.steer_ang = self.psimin
		self.steer_ang_vel = 0.1
		self.speed = -0.2
		self.accel = 0.1
		self.jerk = 0



if __name__ == '__main__':
	ctrl = parking_ctrl()
	try:
		ctrl.spin()
	except rospy.ROSInterruptException:
		pass
