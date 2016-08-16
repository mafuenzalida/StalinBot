#!/usr/bin/env python
#-*- coding: utf-8 -*-
import rospy
from turtlebot import TurtleBot
from pid_c import PIDController
import numpy as np
from maze import Maze
from node import Node
import math
import Queue as Q

class MazeRunner():

	def __init__(self):
		self.rob = TurtleBot()
		self.pidx = PIDController(3)
		self.pidz = PIDController(1)
		self.min_depth_wall = 0.52
		self.min_check_depth_wall = 0.6
		self.m = None
		self.names = 0
		self.door_node = None
		self.door_dir = 0
		

	'''
	This method makes the robot turn to see the 4 directions of a crossroad and saves this info
	'''
	def check_crossroad(self,dir_actual, x=0,y=0):
		dirs = [0 for x in range(4)]
		for i in range(4):
			##Check
			wall = False
			center = self.rob.vision.get_depth_center()
			print center
			if center <= self.min_check_depth_wall or math.isnan(center):
				print "WALL"
				self.rob.speaker.speak("PARED")
				dirs[i] = 1
				self.align_with_wall()
				
			else:
				print "NOTHING"
				self.rob.speaker.speak("NADA")
				dirs[i] = 0
			##Turn
			self.rob.move.turn(90)
		self.names += 1
		if dir_actual == 0:
			return Node(dirs[0],dirs[1],dirs[2],dirs[3],self.names,x,y)
		elif dir_actual == 1:
			return Node(dirs[3],dirs[0],dirs[1],dirs[2],self.names,x,y)
		elif dir_actual == 2:
			return Node(dirs[2],dirs[3],dirs[0],dirs[1],self.names,x,y)
		else:
			return Node(dirs[1],dirs[2],dirs[3],dirs[0],self.names,x,y)


	'''
	# This method check the actual node that the robot is in
	# if first is False the robot doesnt inspect the previous node if is True it does 
	'''
	def inspect_node(self, dir_actual, x=0, y=0, first = False):

		dirs = [0 for x in range(4)]

		if first == True:
			for i in range(4):
				##Check
				center = self.rob.vision.get_depth_center()
				if center <= self.min_check_depth_wall or math.isnan(center):
					print "WALL"
					self.rob.speaker.speak("PARED")
					dirs[i] = 1
					self.align_with_wall()
					
				else:
					print "NOTHING"
					self.rob.speaker.speak("NADA")
					dirs[i] = 0
				##Turn
				self.rob.move.turn(90)
			center = self.rob.vision.get_depth_center()
			if center <= self.min_check_depth_wall or math.isnan(center):
				self.align_with_wall()
		else:
			center = self.rob.vision.get_depth_center()
			if center <= self.min_check_depth_wall or math.isnan(center):
				print "WALL"
				self.rob.speaker.speak("PARED")
				dirs[0] = 1
				self.align_with_wall()
					
			else:
				print "NOTHING"
				self.rob.speaker.speak("NADA")
				dirs[0] = 0

			self.rob.move.turn(90)

			center = self.rob.vision.get_depth_center()
			if center <= self.min_check_depth_wall or math.isnan(center):
				print "WALL"
				self.rob.speaker.speak("PARED")
				dirs[1] = 1
				self.align_with_wall()
					
			else:
				print "NOTHING"
				self.rob.speaker.speak("NADA")
				dirs[1] = 0

			self.rob.move.turn(180)

			dirs[2] = 1

			center = self.rob.vision.get_depth_center()
			if center <= self.min_check_depth_wall or math.isnan(center):
				print "WALL"
				self.rob.speaker.speak("PARED")
				dirs[3] = 1
				self.align_with_wall()
					
			else:
				print "NOTHING"
				self.rob.speaker.speak("NADA")
				dirs[3] = 0

			self.rob.move.turn(90)

		self.names += 1
		if dir_actual == 0:
			return Node(dirs[0],dirs[1],dirs[2],dirs[3],self.names,x,y)
		elif dir_actual == 1:
			return Node(dirs[3],dirs[0],dirs[1],dirs[2],self.names,x,y)
		elif dir_actual == 2:
			return Node(dirs[2],dirs[3],dirs[0],dirs[1],self.names,x,y)
		else:
			return Node(dirs[1],dirs[2],dirs[3],dirs[0],self.names,x,y)



	def check_wall(self):
		center = self.rob.vision.get_depth_center()
		if center <= 0.8:
			return True
		else:
			return False

	'''
	Method to go from crossroad to crossroad
	This version will advance until a wall is faced, later one will advance until the robot is in a new crossroad
	'''
	def advance_maze(self, distance):
		self.rob.move.set_initial_pos()
		rate = rospy.Rate(10)
		min_dif_depth = 0.01
		max_dif_turn = 0.5
		max_depth_side = 1.6
		while True:
			depth_right, depth_left = self.rob.vision.get_sides_depth()
			#print "dr: ", depth_right, " dl: ", depth_left
			vel_x = 0
			vel_z = 0
			dif = abs(depth_right - depth_left)
			sides = False
			## Check if vel_z is needed
			min_dif_sides = 0
			if dif > min_dif_sides and depth_right < max_depth_side and depth_left < max_depth_side:  #and dif < max_dif_turn:
				##Set angular velocity and its orientation
				if(np.minimum(depth_right,depth_left) == depth_right):
					vel_z = self.pidz.update_PID(dif)
				else:
					vel_z = -self.pidz.update_PID(dif)
				sides = True
			else:
				depth_right, depth_left = self.rob.vision.get_center_sides_depth()
				#print "drcentro: ", depth_right, " dlcentro: ", depth_left
				dif = abs(depth_right - depth_left)
				if dif >= 0.0005: #min_dif_sides:
					if(np.minimum(depth_right,depth_left) == depth_right):
						vel_z = -self.pidz.update_PID(dif*30)#20 ok
					else:
						vel_z = self.pidz.update_PID(dif*30)#20 ok
				else:
					vel_z = 0
			##TODO: TRATAR DE USAR MURALLA DEL FONDO PARA ALINEAR SI PAREDES DEL LADO NO FUNCIONAN

			vel_x = self.rob.move.get_vel_by_distance_traveled(distance)
			center_depth = self.rob.vision.get_depth_center()
			#print center_depth, " " , vel_x
			if center_depth <= self.min_depth_wall or vel_x == -1 :
				self.rob.move.send_vel(0,0)
				break
			else:
				##Closer side depth took in consideration
				#vel_x = self.pidx.update_PID(center_depth)
				aux1 = self.pidx.update_PID(np.minimum(depth_right,depth_left))
				aux2 = self.pidx.update_PID(center_depth - 0.5)
				vel_x = np.minimum(aux1,aux2)
				'''
				if sides == True:
					vel_x = self.pidx.update_PID(np.minimum(depth_right,depth_left))
				else:
					vel_x = self.pidx.update_PID(center_depth)
				'''
			#print "velx: ", vel_x, " vel_z: ", vel_z
			'''if sides:
				print "murallas"
			elif vel_z != 0:
				print "fondo"
			else:
				print "no giro" '''
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

	def follow_path(self, path = None, actual_dir = None):
		last_path = False
		if path == None:
			last_path = True
			path = self.m.astar_search()
		print path
		if actual_dir == None:
			actual_dir = self.m.posrobots[0][2]
		final_dir = None
		for i in path:
			#Get right direction by turning the robot
			self.rob.speaker.speak_direction("TURNING ", i)
			self.rob.move.change_dir_simulator(actual_dir, i)
			actual_dir = i
				
			j = 1
			dist = 0.8
			while(i + j < len(path) and path[i+j] == actual_dir):
				dist += 0.8
				j += 1

			#Go to next node using walls depth info and odom info
			self.rob.speaker.speak_direction("ADVANCING ", i)
			self.advance_maze(dist)
			if self.check_wall():
				while self.rob.vision.get_depth_center() > self.min_depth_wall:
					self.rob.move.send_vel(0.04,0)
				k = 0
				while k < 5:
					self.rob.move.send_vel(0,0)
					k += 1
				self.align_with_wall()

		if last_path == True:
			#Get final direction in case there is one specified
			if final_dir != None:
				self.rob.move.change_dir(actual_dir,final_dir)
			self.rob.speaker.speak("A MI DESTINO LLEGADO HE")
		else:
			print "Nodo final"
			return actual_dir

	'''
	Method to solve the maze with map info but not known position.
	Flow of the algorithm:
		1.Save info of actual node
		2.Check if there is only 1 match between actual info and the given map
		3.If 2 is false then go to 4 else go to 5
		4.Explore another node not visited and go to 1
		5.Execute follow_path() to get to the final point
	'''
	def solve_maze(self):
		actual_dir = 0
		actual_node = None
		last_node = None
		match = 0
		turn = 0
		x = 0
		y = 0
		visited = []
		
		while(match != 1):
			## Move to a new node
			if actual_node != None:
				path, last_node = self.m.new_node_search(actual_node)
				print path
				actual_dir = self.follow_path(path, actual_dir)
			## Add info of the actual node
			actual_node = self.check_crossroad(actual_dir)
			if last_node != None:
				last_node.add_neighbour(actual_node,actual_dir)
				actual_node.add_neighbour(last_node,(actual_dir+2)%4)
		
			print actual_node.neighbours
			## Check if the robot can figure where he is
			match, x, y,turn = self.m.compare_structures(self.m.m, actual_node)
	
		print "MATCH ENCONTRADO"
		self.rob.speaker.speak("I KNOW WHERE I AM")

		actual_dir = (actual_dir + turn) % 4
		print "actual_dir: ", actual_dir, " x: ", x, " y: ", y, "turn: ", turn
		self.m.change_initial(x,y,actual_dir)
		self.follow_path()

	def align_with_wall(self):
		##Align
		depth_right = 0
		depth_left = 0
		dif = 10
		while dif >= 0.001 and math.isnan(depth_right) == False and math.isnan(depth_left) == False:
			depth_right, depth_left = self.rob.vision.get_center_sides_depth()
			dif = abs(depth_right - depth_left)
			if(np.minimum(depth_right,depth_left) == depth_right):
				#vel_z = -self.pidz.update_PID(dif*30)#20 ok
				vel_z = -0.6
			else:
				#vel_z = self.pidz.update_PID(dif*30)#20 ok
				vel_z = 0.6
			self.rob.move.send_vel(0,vel_z)
		for k in range(10):
			self.rob.move.send_vel(0,0)
				
	def programa(self):
		while not rospy.is_shutdown():
			a = 2

	'''
	Method to solve the maze with known start position but uknown map.
	Flow of the algorithm:
		1.Save info of actual node
		2.Update map info
		2.Choose a possible direction to move
		3.If not at finish back to 1
	'''
	def map_maze(self):
		actual_dir = self.m.posrobots[0][2]
		actual_node = None
		last_node = None
		x = self.m.posrobots[0][0]
		y = self.m.posrobots[0][1]
		visited = []
		while(x != self.m.posgoals[0][0] or y != self.m.posgoals[0][1]):
			print "x: ",x," x_g: ",self.m.posgoals[0][0]," y: ",y," y_g: ",self.m.posgoals[0][1]
			## Move to a new node
			path = None
			if actual_node != None:
				path, last_node = self.m.new_node_search(actual_node, self.m.posgoals[0])
				print path
				actual_dir = self.follow_path(path, actual_dir)
			## Add info of the actual node
			actual_node = self.check_crossroad(actual_dir,x,y)
			if last_node != None:
				last_node.add_neighbour(actual_node,actual_dir)
				actual_node.add_neighbour(last_node,(actual_dir+2)%4)
		
			print actual_node.neighbours
			## Update the actual place of the robot
			if path != None:
				x, y = self.update_coordinates(x,y,path)
	
		print "FINAL ENCONTRADO"
		self.rob.speaker.speak("I MADE IT MASTER")

	def update_coordinates(self, x, y, path):
		for i in path:
			if i == 0:
				x = x+1
			elif i == 1:
				y = y-1
			elif i == 2:
				x = x-1
			else:
				y = y+1

		return x,y

	def hero_maze(self):
		## Find Friend looks for his friend and the door, and before it ends, it changes the start of the maze
		#self.find_friend()
		self.m.change_initial(1,3,0)
		print "Esperando que se abra la puerta"
		center = self.rob.vision.get_depth_center()
		while center <= self.min_check_depth_wall or math.isnan(center):
			center = self.rob.vision.get_depth_center()
		print "Ahora escapando"
		self.rob.speaker.speak("LETS ESCAPE NOW")
		self.map_maze()
		
		
	def find_friend(self):
		actual_dir = 0
		actual_node = None
		last_node = None
		match = 0
		turn = 0
		x = 0
		y = 0
		visited = []
		friend = False
		key = False

		## Search for the friend
		while(friend != True):
			## Move and check a new node
			actual_node, actual_dir = self.look_new_nodes(actual_node,actual_dir)

			## Check if the robot has found his friend
			if actual_node.get_friend() == True:
				print "si"
				friend = True

			## check if we can find a match
			if(match != 1):
				match, x, y,turn = self.m.compare_structures(self.m.m, actual_node)
				print("Ya se donde estoy")
				self.rob.speaker.speak("I KNOW WHERE I AM")

		print("SIGUE")
		## Check if we already found the key in the last path
		key = self.key_nodes(actual_node)

		## Search for the key
		while(key != True):
			## Move and check a new node
			actual_node, actual_dir = self.look_new_nodes(actual_node,actual_dir)

			if actual_node.get_key() == True:
				key = True

			## check if we can find a match
			if(match != 1):
				match, x, y,turn = self.m.compare_structures(self.m.m, actual_node)
				print("Ya se donde estoy")
				self.rob.speaker.speak("I KNOW WHERE I AM")

		## loop until the robot know where he is
		while(match != 1):
			## Move and check a new node
			actual_node, actual_dir = self.look_new_nodes(actual_node,actual_dir)
			match, x, y,turn = self.m.compare_structures(self.m.m, actual_node)
			print("Ya se donde estoy")
			self.rob.speaker.speak("I KNOW WHERE I AM")


		if(self.door_node != None):
			## Robot goes directly to the door
			if(actual_node != self.door_node):
				self.go_to_node(actual_node, self.door_node)
			## Look at the door
			self.rob.move.change_dir(actual_dir,self.door_dir,self.rob.vision)
		else:
			## Keep looking for the door
			while(self.door_node == None):
				## Move and check a new node
				actual_node, actual_dir = self.look_new_nodes(actual_node,actual_dir)
			##Look at the door
			self.rob.move.change_dir(actual_dir,self.door_dir,self.rob.vision)

		self.rob.speaker.speak("ABRETE SESAMO")
		match, x, y,turn = self.m.compare_structures(self.m.m, actual_node)
		actual_dir = (actual_dir - turn) % 4
		self.m.change_initial(x,y,actual_dir)

	def look_new_nodes(self, actual_node, actual_dir):
		last_node = None
		## Move to a new node
		if actual_node != None:
			##TODO: volver version de closest node search (o crear nuevo)
			path, last_node = self.m.new_node_search(actual_node)
			print path
			actual_dir = self.follow_path(path, actual_dir)
		## Add info of the actual node (including perception)
		actual_node = self.check_crossroad_image(actual_dir, True)
		if last_node != None:
			last_node.add_neighbour(actual_node,actual_dir)
			actual_node.add_neighbour(last_node,(actual_dir+2)%4)
	
		print actual_node.neighbours
		return actual_node, actual_dir

	def key_nodes(self, initial_node):
		frontier = Q.PriorityQueue()
		frontier.put(initial_node, 0)
		came_from = {}
		cost_so_far = {}
		came_from[initial_node.name] = None
		cost_so_far[initial_node.name] = 0

		while not frontier.empty():
			current = frontier.get()
			for next in current.neighbours:
				if next != None and next != 1:
					if next.get_key() == True:
						return True
					new_cost = cost_so_far[current.name] + 1
					if next.name not in cost_so_far or new_cost < cost_so_far[next.name]:
						cost_so_far[next.name] = new_cost
						priority = new_cost + self.heuristic(next.row, next.col)
						frontier.put(next,priority)
						came_from[next.name] = current
	
		return False


	def check_crossroad_image(self, dir_actual, first):
			friend = False
			key = False
			door = False
			dirs = [0 for x in range(4)]
			j = 0
			for i in range(4):
				##Check
				if(first == True or i != 2):
					wall = False
					center = self.rob.vision.get_depth_center()
					if center <= self.min_check_depth_wall or math.isnan(center):
						print "WALL"
						self.rob.speaker.speak("PARED")
						dirs[i] = 1
						self.align_with_wall()
						##Check wall image
						img = self.rob.vision.image_finder()
						print "IMAGEN: " , img
						if img == 0:
							friend = True
							self.rob.speaker.speak("COME MY FRIEND")
						elif img == 1:
							key = True
							self.rob.speaker.speak("I HAVE THE KEY")
						elif img == 2:
							door = True
							self.rob.speaker.speak("HOLD THE DOOR")
							j = i
						else:
							print "NADA EN PARED"
				
					else:
						print "NOTHING"
						self.rob.speaker.speak("NADA")
						dirs[i] = 0
				else:
					dirs[i] = 0
				##Turn
				self.rob.move.turn(90)
			self.names += 1
			node = None
			if dir_actual == 0:
				node = Node(dirs[0],dirs[1],dirs[2],dirs[3],self.names,0,0)
			elif dir_actual == 1:
				node = Node(dirs[3],dirs[0],dirs[1],dirs[2],self.names,0,0)
			elif dir_actual == 2:
				node = Node(dirs[2],dirs[3],dirs[0],dirs[1],self.names,0,0)
			else:
				node = Node(dirs[1],dirs[2],dirs[3],dirs[0],self.names,0,0)

			if friend == True:
				node.set_friend()
			if key == True:
				node.set_key()
			if door == True:
				self.door_node = node
				self.door_dir = ( dir_actual + j ) % 4

			return node

	def go_to_node(self, actual_node, final_node, actual_dir):
		path = []
		frontier = Q.PriorityQueue()
		frontier.put(self.initial_node, 0)
		came_from = {}
		cost_so_far = {}
		came_from[self.initial_node.name] = None
		cost_so_far[self.initial_node.name] = 0

		while not frontier.empty():
			current = frontier.get()
			if current == final_node:
				break
			for next in current.neighbours:
				if next != None:
					new_cost = cost_so_far[current.name] + 1
					if next.name not in cost_so_far or new_cost < cost_so_far[next.name]:
						cost_so_far[next.name] = new_cost
						priority = new_cost + self.heuristic(next.row, next.col)
						frontier.put(next,priority)
						came_from[next.name] = current
		
		##Construction of the path
		actual = final_node
		while came_from[actual.name] != None:
			path.append(came_from[actual.name].get_dir_neighbour(actual))
			actual = came_from[actual.name]
		path.reverse()
	
		self.follow_path(path,actual_dir)


if __name__ == '__main__':
	while not rospy.is_shutdown():
		mr = MazeRunner()
		mr.m = Maze()
		mr.m.read_maze()
		mr.rob.vision.wait_image()
		#rospy.sleep(5)
		try:
			print "Comenzando programa..."
			#mr.align_with_wall()
			#mr.map_maze()
			#mr.solve_maze()
			#mr.programa()
			#mr.follow_path_simulator()
			#mr.follow_path()
			#mr.check_crossroad()
			#mr.advance_maze()
			#mr.hero_maze()
			break
		except rospy.ROSInterruptException:
			pass
