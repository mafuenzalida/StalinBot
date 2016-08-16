#!/usr/bin/env python
#-*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from pid_c import PIDController
from tf.transformations import euler_from_quaternion
import numpy as np

'''
	Class that controls the movement of the turtlebot
'''
class MoveController():
	
	def __init__(self):
		rospy.Subscriber("/odom", Odometry, self.callback_Odom)
		#rospy.Subscriber("/turtlebot/odom", Odometry, self.callback_Odom)
		self.pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist)
		#self.pub = rospy.Publisher("/turtlebot/cmd_vel", Twist)
		self.max_vel_x = 0.2
		self.max_vel_z = math.radians(90)
		self.x = 0
		self.y = 0
		self.z = 0
		self.twist = Twist()
		self.pidz = PIDController(1.5)
		self.pidx = PIDController(1)
		self.angle = None
		self.have_odom = False
		self.actual_angle = 0
		self.vel_x = 0
		self.vel_z = 0
		self.x_i = 0
		self.y_i = 0

	def callback_Odom(self, msg):
		self.vel_x = msg.twist.twist.linear.x
		self.vel_z = msg.twist.twist.angular.z

		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y
		q = msg.pose.pose.orientation
		a = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

		if self.have_odom:
			a_diff = a - self.angle
			if a_diff > np.pi:
				a_diff -= 2*np.pi
			elif a_diff < -np.pi:
				a_diff += 2*np.pi
			self.actual_angle += a_diff

		self.angle = a
		self.have_odom = True

	def send_vel(self, vel_x, vel_z):
		if(vel_x > self.max_vel_x):
			self.twist.linear.x = self.max_vel_x
		else:
			self.twist.linear.x = vel_x

		if(vel_z > self.max_vel_z):
			self.twist.angular.z = self.max_vel_z
		else:
			self.twist.angular.z = vel_z

		acel_z = 1
		if(vel_z - acel_z > self.vel_z):
			self.twist.angular.z = self.vel_z + acel_z

		acel_x = 0.05
		if(vel_x - acel_x > self.vel_x):
			self.twist.linear.x = self.vel_x + acel_x
		
		self.pub.publish(self.twist)

	def turn(self, angle):
		neg = 1
		if angle < 0:
			neg = -1
		angle = math.radians(abs(angle))
		angle_i = self.actual_angle
		rate = rospy.Rate(10)
		while(True):
			a_diff = abs(self.actual_angle - angle_i)
			if (angle > 0 and a_diff >= angle) or (angle < 0 and a_diff <= angle):
				break
			angle_error = 0.2
			vel_z = self.pidz.update_PID(angle + angle_error - a_diff)
			vel_z = vel_z * neg
			self.send_vel(0,vel_z)
			rate.sleep()

		self.send_vel(0,0)
		rospy.sleep(1)
	
	def advance(self, distance):
		x_i = self.x
		y_i = self.y
		rate = rospy.Rate(10)
		while(True):
			distance_traveled = math.sqrt(math.pow((self.x - x_i), 2)+math.pow((self.y-y_i),2))
			if distance_traveled >= distance:
				break
			x_error = 0.05
			vel_x = self.pidx.update_PID(distance + x_error - distance_traveled)
			self.send_vel(vel_x, 0)
			rate.sleep()
		self.send_vel(0,0)

	def get_vel_by_distance_traveled(self, distance):
		distance_traveled = math.sqrt(math.pow((self.x - self.x_i), 2)+math.pow((self.y-self.y_i),2))
		if distance_traveled >= distance:
			return -1
		x_error = 0.05
		vel_x = self.pidx.update_PID(distance + x_error - distance_traveled)
		#print distance_traveled
		return vel_x

	def set_initial_pos(self):
		self.x_i = self.x
		self.y_i = self.y

	'''
	Method that rotates the robot from start_dir to final_dir.
	Simulator version where there is no error.
	'''
	def change_dir_simulator(self, start_dir, final_dir):
		if start_dir == 0:
			if final_dir == 1:
				self.turn(90)
			elif final_dir == 2:
				self.turn(180)
			elif final_dir == 3:
				self.turn(-90)
		elif start_dir == 1:
			if final_dir == 0:
				self.turn(-90)
			elif final_dir == 2:
				self.turn(90)
			elif final_dir == 3:
				self.turn(180)
		elif start_dir == 2:
			if final_dir == 0:
				self.turn(180)
			elif final_dir == 1:
				self.turn(-90)
			elif final_dir == 3:
				self.turn(90)
		elif start_dir == 3:
			if final_dir == 0:
				self.turn(90)
			elif final_dir == 1:
				self.turn(180)
			elif final_dir == 2:
				self.turn(-90)

	'''
	Same method as change_dir_simulator, but this version handles real robot errors.
	Vision is passed as an argument. This is the object that handles kinect in the robot
	used to align the robot in eventual error of the turn.
	'''			
	def change_dir(self, start_dir, final_dir, vision):
		rate = rospy.Rate(10)
		##Turn the theorical angle
		print "si"
		self.change_dir_simulator(start_dir,final_dir)
		##Try to align the robot the best possible
		##state = self.vision.get_state()
		state = 0
		'''
		Possible states:
		0 - Wall right in front
		1 - 2 Walls right at the sides
		2 - 1 Wall right at the left side
		3 - 1 Wall right at the right side
		4 - No walls near, front image is used
		'''
		if state == 0 or state == 1 or state == 4:
			depth_right = 0
			depth_left = 0
			while True:
				if state == 1 or state == 4:
					depth_right, depth_left = self.rob.vision.get_front_lines_depth()
				elif state == 2:
					depth_right, depth_left = self.rob.vision.get_sides_depth()
				vel_z = 0
				dif = abs(depth_right - depth_left)
				## Check if vel_z is needed
				min_dif_sides = 0
				if dif > min_dif_sides:
					##Set angular velocity and its orientation
					if(np.minimum(depth_right,depth_left) == depth_right):
						vel_z = self.pidz.update_PID(dif)
					else:
						vel_z = -self.pidz.update_PID(dif)
					self.send_vel(0,vel_z)
				else:
					self.send_vel(0,0)
					break
				rate.sleep()
		elif state == 2 or state == 3:
			##TODO: Ver valores para alinearse con una sola muralla...
			## 	Ver si se puede hacer siempre con muralla de fondo
			depth = 0
		

