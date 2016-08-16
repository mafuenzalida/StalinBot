#!/usr/bin/env python
#-*- coding: utf-8 -*-
import rospy
import math
from turtlebot import TurtleBot
from pid_c import PIDController

rob = TurtleBot()
pidx = PIDController(0.5)
pidz = PIDController(1)

def look_for_friend():
	global rob
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		cx, depth = rob.vision.friend_dist_finder()
		print "cx: ", cx, " depth: ", depth
		vel_x = 0
		vel_z = 0
		depth_final = 0.05

		if depth > 0:
			##SET VELOCIDAD LINEAL TODO: VER PID QUE FUNCIONE
			vel_x = pidx.update_PID(depth-depth_final)

		if cx >= 0:
			vel_z = -pidz.update_PID(cx)
		elif cx < 0:
			##TODO: REVISAR SI CX ES LA DIF DEL CENTRO PARA USAR EN PID
			vel_z = pidz.update_PID(abs(cx))

		#print vel_z

		print "velx: ", str(vel_x), " velz: ", str(vel_z)
		rob.move.send_vel(vel_x, vel_z)

		rate.sleep()


if __name__ == '__main__':
	try:
		look_for_friend()
	except rospy.ROSInterruptException:
		pass
