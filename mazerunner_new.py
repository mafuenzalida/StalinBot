#!/usr/bin/env python
#-*- coding: utf-8 -*-
import rospy
from turtlebot import TurtleBot
from pid_c import PIDController
import numpy as np
from maze import Maze

class MazeRunner():

	def __init__(self):
		self.rob = TurtleBot()
		self.pidx = PIDController(3)
		self.pidz = PIDController(1)
		self.min_depth_wall = 0.55

	'''
	This method makes the robot turn to see the 4 directions of a crossroad and saves this info
	METODO CAMBIADO, REEMPLAZAR EN ORIGINAL
	'''
	def check_crossroad(self):
		dirs = [4]
		for i in range(4):
			##Align
			##rob.vision.align_with_image()
			##Check
			center = self.rob.vision.get_depth_center()
			print center
			if center <= self.min_depth_wall:
				print "WALL"
				self.rob.speaker.speak("PARED")
				dirs[i] = 1
			else:
				print "NOTHING"
				self.rob.speaker.speak("NADA")
				dirs[i] = 0
			##Turn
			self.rob.move.turn(90)
		return Node(dirs[0],dirs[1],dirs[2],dirs[3],0,0,0)


	'''
	Method to go from crossroad to crossroad
	This version will advance until a wall is faced, later one will advance until the robot is in a new crossroad
	'''
	def advance_maze(self, distance):
		self.rob.move.set_initial_pos()
		rate = rospy.Rate(5)
		min_dif_depth = 0.01
		max_dif_turn = 1
		while not rospy.is_shutdown():
			depth_right, depth_left = self.rob.vision.get_sides_depth()
			print "dr: ", depth_right, " dl: ", depth_left
			vel_x = 0
			vel_z = 0
			dif = abs(depth_right - depth_left)
			## Check if vel_z is needed
			min_dif_sides = 0
			if dif > min_dif_sides and dif < max_dif_turn:
				##Set angular velocity and its orientation
				if(np.minimum(depth_right,depth_left) == depth_right):
					vel_z = self.pidz.update_PID(dif)
				else:
					vel_z = -self.pidz.update_PID(dif)
			else: #Use the wall in front
				depth_right, depth_left = self.rob.vision.get_center_sides_depth()
				dif = abs(depth_right - depth_left)
				if dif > min_dif_sides:
					if(np.minimum(depth_right,depth_left) == depth_right):
						vel_z = self.pidz.update_PID(dif)
					else:
						vel_z = -self.pidz.update_PID(dif)
				else:
					vel_z = 0

			vel_x = self.rob.move.get_vel_by_distance(distance)
			center_depth = self.rob.vision.get_depth_center()
			if center_depth <= self.min_depth_wall or vel_x == -1 :
				self.rob.move.send_vel(0,0)
				break
			else:
				##Closer side depth took in consideration
				#vel_x = self.pidx.update_PID(center_depth)
				if vel_z != 0:
					vel_x = self.pidx.update_PID(np.minimum(depth_right,depth_left))
				else:
					vel_x = self.pidx.update_PID(center_depth)

			self.rob.move.send_vel(vel_x, vel_z)
			rate.sleep()

	def follow_path_simulator(self):
		m = Maze()
		m.read_maze()
		path = m.astar_search()
		print path
		actual_dir = m.posrobots[0][2]
		final_dir = None
		i = 0
		while i < len(path):
			print "Moviendo en direccion: " + str(path[i])
			#Get right direction
			#self.rob.move.change_dir_simulator(actual_dir, path[i])
			actual_dir = path[i]
			#Go to next node
			j = 1
			dist = 0.8
			while(i + j < len(path) and path[i+j] == actual_dir):
				dist += 0.8
				j += 1
			print dist
			self.rob.move.advance(dist)
			i += j
		if final_dir != None:
			self.rob.move.change_dir_simulator(actual_dir, final_dir)
		print "Se llego a la meta"

	def follow_path(self):
		while not rospy.is_shutdown():
			m = Maze()
			m.read_maze()
			path = m.astar_search()
			actual_dir = m.posrobots[0][2]
			final_dir = None
			for i in path:
				#Get right direction by turning the robot
				self.rob.speaker.speak_direction("GIRANDO A DIRECCION ", i)
				self.rob.move.change_dir(actual_dir, i, self.rob.vision)
				actual_dir = i
				
				j = 1
				dist = 0.8
				while(i + j < len(path) and path[i+j] == actual_dir):
					dist += 0.8
					j += 1

				#Go to next node using walls depth info and odom info
				self.rob.speaker.speak_direction("AVANZANDO EN DIRECCION", i)
				self.advance_maze(j)

			#Get final direction in case there is one specified
			if final_dir != None:
				self.rob.move.change_dir(actual_dir,final_dir)
			self.rob.speaker.speak("A MI DESTINO LLEGADO HE")

	def programa(self):
		while not rospy.is_shutdown():
			print self.rob.vision.get_sides_depth(1)


if __name__ == '__main__':
	mr = MazeRunner()
	self.rob.vision.wait_image() ##AGREGAR EN ORIGINAL
	try:
		print "Comenzando programa..."
		#mr.programa()
		mr.follow_path_simulator()
		#mr.follow_path()
		#mr.check_crossroad()
		#mr.advance_maze()
	except rospy.ROSInterruptException:
		pass
