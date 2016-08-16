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
		self.llave_roja_1 = False
		self.llave_roja_2 = False
		self.puerta_roja = False
		self.puerta_roja_node = None
		self.puerta_roja_dir = 0
		self.llave_amarilla_1 = False
		self.llave_amarilla_2 = False
		self.puerta_amarilla = False
		self.puerta_amarilla_node = None
		self.puerta_amarilla_dir = 0
		self.amigo_node = None
		self.amigo_dir = 0
		self.amigo_2_node = None
		self.amigo_2_dir = 0
		self.friend = False
		self.puerta_amarilla = False
		self.puerta_roja = False
		self.carga = False
		self.evil = False
		self.evil_node = None
		self.evil_dir = 0
		self.actual_dir = 0
		self.actual_node = None


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


	'''
	# This method check the actual node that the robot is in
	# if first is False the robot doesnt inspect the previous node if is True it does 
	'''
	def inspect_node(self, dir_actual, x=0, y=0, first = False):

		dirs = [0 for x in range(4)]
		node = None
		i = 0
		j = 0
		k = 0

		dirs[0] = self.check_wall()
		if self.friend == True:
			i = 0
		if self.puerta_amarilla == True or self.puerta_roja == True:
			j = 0
		if self.evil == True:
			k = 0
		self.rob.move.turn(90)

		dirs[1] = self.check_wall()
		if self.friend == True:
			i = 1
		if self.puerta_amarilla == True or self.puerta_roja == True:
			j = 1
		if self.evil == True:
			k = 1
		
		if first == True:
			self.rob.move.turn(90)
			dirs[2] = self.check_wall()
			if self.friend == True:
				i = 2
			if self.puerta_amarilla == True or self.puerta_roja == True:
				j = 2
			if self.evil == True:
				k = 2
			self.rob.move.turn(90)
		else:
			dirs[2] = 0
			self.rob.move.turn(180)

		dist[3] = self.check_wall()
		if self.friend == True:
			i = 3
		if self.puerta_amarilla == True or self.puerta_roja == True:
			j = 3
		if self.evil == True:
			k = 3
		self.rob.move.turn(180)

		center = self.rob.vision.get_depth_center()
		if center <= self.min_check_depth_wall or math.isnan(center):
			self.align_with_wall()

		self.names += 1

		if dir_actual == 0:
			node = Node(dirs[0],dirs[1],dirs[2],dirs[3],self.names,x,y)
		elif dir_actual == 1:
			node = Node(dirs[3],dirs[0],dirs[1],dirs[2],self.names,x,y)
		elif dir_actual == 2:
			node = Node(dirs[2],dirs[3],dirs[0],dirs[1],self.names,x,y)
		else:
			node = Node(dirs[1],dirs[2],dirs[3],dirs[0],self.names,x,y)

		if self.friend == True:
			if self.amigo_node == None:
				self.amigo_node = node
				self.amigo_dir = ( dir_actual + i ) % 4
			else:
				self.amigo_2_node = node
				self.amigo_2_dir = ( dir_actual + i ) % 4
			self.friend = False
		if self.puerta_amarilla == True:
			self.puerta_amarilla_node = node
			self.puerta_amarilla_dir = ( dir_actual + j ) % 4
			self.puerta_amarilla = False
		if self.puerta_roja == True:
			self.puerta_roja_node = node
			self.puerta_roja_dir = ( dir_actual + j ) % 4
			self.puerta_roja = False
		if self.evil == True:
			self.evil_node = node
			self.evil_dir = k
			self.evil = False

	def check_wall(self):
		center = self.rob.vision.get_depth_center()
		if center <= self.min_check_depth_wall or math.isnan(center):
			print "WALL"
			self.rob.speaker.speak("PARED")
			self.align_with_wall()
			##Look for the images
			self.look_for_images()
			return 1
		else:
			print "NOTHING"
			self.rob.speaker.speak("NADA")
			return 0

	def check_pared(self):
		center = self.rob.vision.get_depth_center()
		if center <= 0.8:
			return True
		else:
			return False

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
			if self.check_pared():
				while self.rob.vision.get_depth_center() > self.min_depth_wall:
					self.rob.move.send_vel(0.04,0)
				k = 0
				while k < 5:
					self.rob.move.send_vel(0,0)
					k += 1
				self.align_with_wall()

		update_coordinates(path)

		if last_path == True:
			#Get final direction in case there is one specified
			if final_dir != None:
				self.rob.move.change_dir(actual_dir,final_dir)
			self.rob.speaker.speak("A MI DESTINO LLEGADO HE")
		else:
			print "Nodo final"
			return actual_dir

	def update_coordinates(self, path):
		for i in path:
			if i == 0:
				self.x = self.x+1
			elif i == 1:
				self.y = self.y-1
			elif i == 2:
				self.x = self.x-1
			else:
				self.y = self.y+1

	def look_for_images():
		img = self.rob.vision.image_finder()
		if img == -1:
			print "No Hay Imagen"
		elif img == 0:
			print "AVERAGEMAN"
			self.friend = True
			if self.carga == False:
				self.rob.speaker.speak("COME WITH ME MY REAL FRIEND")
			else:
				self.rob.speaker.speak("DOPPELGANGER")
		elif img == 1:
			print "CLON 1"
			self.evil = True
			self.rob.speaker.speak("IMPOSTOR")
		elif img == 2:
			print "CLON 2"
			self.evil = True
			self.rob.speaker.speak("IMPOSTOR")
		elif img == 3:
			print "PUERTA ROJA"
			self.puerta_roja = True
			self.rob.speaker.speak("RED DOOR IS HERE")
		elif img == 4:
			print "LLAVE ROJA"
			if self.llave_roja_1 == False:
				self.llave_roja_1 = True
				self.rob.speaker.speak("FIRST RED KEY")
			else:
				self.llave_roja_2 = True
				self.rob.speaker.speak("SECOND RED KEY")
		elif img == 5:
			print "PUERTA AMARILLA"
			self.puerta_amarilla = True
			self.rob.speaker.speak("YELLOW DOOR IS HERE")
		elif img == 6:
			print "LLAVE AMARILLA"
			if self.llave_amarilla_1 == False:
				self.llave_amarilla_1 = True
				self.rob.speaker.speak("FIRST YELLOW KEY")
			else:
				self.llave_amarilla_2 = True
				self.rob.speaker.speak("SECOND YELLOW KEY")


	def find_friend(self, first = True):
		match = 0
		turn = 0
		x = 0
		y = 0

		## Search for the friend
		while(self.amigo_node == None):
			## Move and check a new node
			self.actual_node, self.actual_dir = self.look_new_nodes(self.actual_node,self.actual_dir)

			## check if we can find a match
			if(match != 1):
				match, x, y,turn = self.m.compare_structures(self.m.m, self.actual_node)
				print("Ya se donde estoy")
				self.rob.speaker.speak("I KNOW WHERE I AM")

		print("Ahora por las llaves amarillas")

		## Search for the key
		while(self.llave_roja_1 != True):
			## Move and check a new node
			self.actual_node, self.actual_dir = self.look_new_nodes(self.actual_node,self.actual_dir)

			## check if we can find a match
			if(match != 1):
				match, x, y,turn = self.m.compare_structures(self.m.m, self.actual_node)
				print("Ya se donde estoy")
				self.rob.speaker.speak("I KNOW WHERE I AM")

		## Search for the key
		while(self.llave_roja_2 != True):
			## Move and check a new node
			self.actual_node, self.actual_dir = self.look_new_nodes(self.actual_node,self.actual_dir)
			
			## check if we can find a match
			if(match != 1):
				match, x, y,turn = self.m.compare_structures(self.m.m, self.actual_node)
				print("Ya se donde estoy")
				self.rob.speaker.speak("I KNOW WHERE I AM")

		## loop until the robot know where he is
		while(match != 1):
			## Move and check a new node
			self.actual_node, self.actual_dir = self.look_new_nodes(self.actual_node,self.actual_dir)
			match, x, y,turn = self.m.compare_structures(self.m.m, self.actual_node)
			print("Ya se donde estoy")
			self.rob.speaker.speak("I KNOW WHERE I AM")

		print("Buscando puerta roja")

		if(self.puerta_roja_node != None):
			## Robot goes directly to the door
			if(self.actual_node != self.puerta_roja_node):
				self.go_to_node(self.actual_node, self.puerta_roja_node,self.actual_dir)
		else:
			## Keep looking for the door
			while(self.puerta_roja_node == None):
				## Move and check a new node
				self.actual_node, self.actual_dir = self.look_new_nodes(self.actual_node,self.actual_dir)
		
		##Look at the door
		self.rob.move.change_dir(self.actual_dir,self.puerta_roja_dir,self.rob.vision)
		self.actual_dir = self.puerta_roja_dir

		self.rob.speaker.speak("ABRETE SESAMO")
		self.actual_node = self.good_structure(self.actual_node)
		self.actual_dir = (self.actual_dir - turn) % 4

	def find_yellow():
		## Search for the key
		while(self.llave_amarilla_1 != True):
			## Move and check a new node
			self.actual_node, self.actual_dir = self.look_new_nodes(self.actual_node,self.actual_dir)

		## Search for the key
		while(self.llave_amarilla_2 != True):
			## Move and check a new node
			self.actual_node, self.actual_dir = self.look_new_nodes(self.actual_node,self.actual_dir)

		if(self.puerta_amarilla_node != None):
			## Robot goes directly to the door
			if(self.actual_node != self.puerta_amarilla_node):
				self.go_to_node(self.actual_node, self.puerta_amarilla_node,self.actual_dir)
		else:
			## Keep looking for the door
			while(self.puerta_amarilla_node == None):
				## Move and check a new node
				self.actual_node, self.actual_dir = self.look_new_nodes(self.actual_node,self.actual_dir)
		
		##Look at the door
		self.rob.move.change_dir(self.actual_dir,self.puerta_amarilla_dir,self.rob.vision)
		self.actual_dir = self.puerta_amarilla_dir

		self.rob.speaker.speak("ABRETE SESAMO")

	def go_to_node(self, self.actual_node, final_node, actual_dir):
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
	
		self.actual_dir = self.follow_path(path,actual_dir)

	def look_new_nodes(self, actual_node, actual_dir):
		last_node = None
		## Move to a new node
		if actual_node != None:
			##TODO: volver version de closest node search (o crear nuevo)
			path, last_node = self.m.new_node_search(actual_node)
			print path
			actual_dir = self.follow_path(path, actual_dir)
		## Add info of the actual node (including perception)
		if actual_node != None:
			actual_node = self.inspect_node(actual_dir)
		else:
			actual_node = self.inspect_node(actual_dir,0,0, True)

		if last_node != None:
			last_node.add_neighbour(actual_node,actual_dir)
			actual_node.add_neighbour(last_node,(actual_dir+2)%4)
	
		return actual_node, actual_dir

	def leave_maze(self):
		x_final = self.m.posgoals[0]
		y_final = self.m.posgoals[1]

		while(self.x != x_final and self.y != y_final):
			##TODO: volver version de closest node search (o crear nuevo)
			path, self.actual_node = self.m.new_node_search(self.actual_node,self.m.posgoals)
			new_dir = path.pop()
			print path
			self.actual_dir = self.follow_path(path, self.actual_dir)

			center = self.rob.vision.get_depth_center()
			if center <= self.min_check_depth_wall or math.isnan(center):
				self.actual_node.add_wall(new_dir)
				img = self.rob.vision.image_finder()
				if img == 7:
					#Doblar derecha
					self.actual_node.add_wall((self.actual_dir+1)%4)
					print "Doblar derecha"
				elif img == 8:
					#Doblar izquierda
					self.actual_node.add_wall((self.actual_dir-1)%4)
					print "Doblar izquierda"
				elif img == 9:
					#No doblar derecha
					self.actual_node.add_wall((self.actual_dir-1)%4)
					print "No doblar derecha"
				elif img == 10:
					#No doblar izquierda
					self.actual_node.add_wall((self.actual_dir+1)%4)
					print "No doblar izquierda"
			else:
				self.actual_dir = self.follow_path([new_dir], self.actual_dir)
				self.names++
				aux_node = Node(1,1,1,1,self.names,x,y)
				self.actual.add_neighbour(aux_node,new_dir)
				aux_node.add_neighbour(self.actual_node,(new_dir+2)%4)
				self.actual_node = aux_node


	def open_yellow(self):
		img = self.rob.vision.image_finder()
		while(img != 5):
			self.rob.move.turn(90)
			self.align_with_wall()
			img = self.rob.vision.image_finder()

		self.rob.speaker.speak("WE ARE FREEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE")
		self.advance_maze(0.8)


	def final_run(self):
		## Find Friend looks for the friend, 2 yellow keys and the yellow door
		self.find_friend()
		print "Esperando que se abra la puerta"
		center = self.rob.vision.get_depth_center()
		while center <= self.min_check_depth_wall or math.isnan(center):
			center = self.rob.vision.get_depth_center()
		print "Segunda parte"
		self.find_yellow()

		
		rospy.sleep(4)
		center = self.rob.vision.get_depth_center()
		if center <= self.min_check_depth_wall or math.isnan(center):
			self.rob.speaker.speak("ME HE EQUIVOCADO")
			self.go_to_node(self.actual_node, self.amigo_node,self.actual_dir)
			if amigo_2_node != None:
				self.go_to_node(self.actual_node, self.amigo_2_node,self.actual_dir)
			else:
				while amigo_2_node != None:
					self.actual_node, self.actual_dir = self.look_new_nodes(self.actual_node,self.actual_dir)
			self.rob.move.change_dir(self.actual_dir,self.amigo_2_dir,self.rob.vision)
			self.actual_dir = self.amigo_2_dir
			self.rob.speaker.speak("AHORA SI MI BUEN AMIGO")
			self.go_to_node(self.actual_node, self.puerta_roja_node,self.actual_dir)
			self.rob.move.change_dir(self.actual_dir,self.puerta_roja_dir,self.rob.vision)
			self.actual_dir = self.puerta_roja_dir
			self.actual_dir = self.follow_path([self.actual_dir],self.actual_dir)
			self.rob.speaker.speak("LOCK THE IMPOSTOR")
			self.go_to_node(self.actual_node, self.puerta_amarilla_node,self.actual_dir)
			self.rob.move.change_dir(self.actual_dir,self.puerta_amarilla_dir,self.rob.vision)
			self.actual_dir = self.puerta_amarilla_dir
			self.rob.speaker.speak("OPEN SESAME")

		print "SALIENDO DEL MAZE"
		self.leave_maze()
		##TERMINAMOS
		self.open_yellow()


if __name__ == '__main__':
	while not rospy.is_shutdown():
		mr = MazeRunner()
		mr.m = Maze()
		mr.m.read_maze()
		mr.rob.vision.wait_image()
		try:
			print "Comenzando programa..."
			mr.final_run()
			break
		except rospy.ROSInterruptException:
			pass
