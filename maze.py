#!/usr/bin/env python
#-*- coding: utf-8 -*-

from node import Node
import Queue as Q
import math

class Maze:
	def __init__(self):
		self.nrows = 0
		self.ncols = 0
		self.initial_node = None
		self.final_node = None
		self.nrobots = 0
		self.posrobots = []
		self.ngoals = 0
		self.posgoals = []
		self.max_d_search = 0
		self.matrix = None
		self.m = None

	'''
	All the info that describes the maze will be saved at the local variables and described by objects of the Node class.
	The next method reads the files and saves this info
	'''
	def read_maze(self):
		f = open("c.txt","r")
		'''
		FILE FORMAT
		3 2		nrows,ncols
		0 0 0 1 1 0	row,col,up,left,down,right
		1 0 1 1 0 0
		...
		START
		1		nrobots
		0 0 u		pos pos, dir
		GOAL
		4		ngoals
		2 0 u		pos pos, dir
		2 0 l
		...
		10		max_depth_search
		'''

		n = 0
	
		##rows,cols
		aux = f.readline().split(" ")
		self.nrows = int(aux[0])
		self.ncols = int(aux[1])

		self.matrix = [[None for x in range(self.ncols)] for y in range(self.nrows)]
		self.m = [[0 for x in range(self.ncols)] for y in range(self.nrows)]

		#Creation of Nodes
		for i in range(self.nrows*self.ncols):
			line = f.readline().split(" ")
			line = [int(j) for j in line]
			self.matrix[line[0]][line[1]] = Node(line[2],line[3],line[4],line[5],n,line[0],line[1])
			n += 1

		##Connection of neighbours
		for i in range(self.nrows):
			for j in range(self.ncols):
				self.m[i][j] = self.matrix[i][j].type
				temp_node = self.matrix[i][j]
				for k in range(4):
					n = False
					if temp_node.neighbours[k] == 1:
						n = True
					if k == 0 and n:
						temp_node.add_neighbour(self.matrix[i+1][j], 0)
					elif k == 1 and n:
						temp_node.add_neighbour(self.matrix[i][j-1], 1)
					elif k == 2 and n:
						temp_node.add_neighbour(self.matrix[i-1][j], 2)
					elif k == 3 and n:
						temp_node.add_neighbour(self.matrix[i][j+1], 3)
					else:
						temp_node.add_neighbour(None, k)
		##ignore START line
		print f.readline()
		##nrobots
		self.nrobots = int(f.readline())
		
		##Positions of robots (and direction)
		for i in range(self.nrobots):
			aux = f.readline().split(" ")
			aux2 = []
			aux2.append(int(aux[0]))
			aux2.append(int(aux[1]))
			if aux[2][0] == 'u':
				aux2.append(0)
			elif aux[2][0] == 'l':
				aux2.append(1)
			elif aux[2][0] == 'd':
				aux2.append(2)
			elif aux[2][0] == 'r':
				aux2.append(3)
			self.posrobots.append(aux2)

		#Save initial node (actual ver only has one robot)
		self.initial_node = self.matrix[self.posrobots[0][0]][self.posrobots[0][1]]

		##ignore GOAL line
		f.readline()
		##ngoals
		self.ngoals = int(f.readline())
		
		##Positions of goals (and direction)
		for i in range(self.ngoals):
			aux = f.readline().split(" ")
			aux2 = []
			aux2.append(int(aux[0]))
			aux2.append(int(aux[1]))
			if aux[2][0] == 'u':
				aux2.append(0)
			elif aux[2][0] == 'l':
				aux2.append(1)
			elif aux[2][0] == 'd':
				aux2.append(2)
			elif aux[2][0] == 'r':
				aux2.append(3)
			self.posgoals.append(aux2)

		##Save final node (actual ver only has one goal)
		self.final_node = self.matrix[self.posgoals[0][0]][self.posgoals[0][1]]
		
		##Max depth search
		self.max_d_search = int(f.readline())

	'''
	Search algorithm that finds a path from the start to the goal
	PATH FORMAT
	0 = UP
	1 = LEFT
	2 = DOWN
	3 = RIGHT
	'''
	def astar_search(self):
		path = []
		frontier = Q.PriorityQueue()
		frontier.put(self.initial_node, 0)
		came_from = {}
		cost_so_far = {}
		came_from[self.initial_node.name] = None
		cost_so_far[self.initial_node.name] = 0

		while not frontier.empty():
			current = frontier.get()
			if current == self.final_node:
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
		actual = self.final_node
		while came_from[actual.name] != None:
			path.append(came_from[actual.name].get_dir_neighbour(actual))
			actual = came_from[actual.name]
		path.reverse()
	
		return path

	def new_node_search(self, initial_node, goal = None):
		path = []
		frontier = Q.PriorityQueue()
		frontier.put(initial_node, 0)
		came_from = {}
		cost_so_far = {}
		came_from[initial_node.name] = None
		cost_so_far[initial_node.name] = 0

		non_full_visited = []

		while not frontier.empty():
			current = frontier.get()
			for i in current.neighbours:
				if i == 1:
					non_full_visited.append(current)
			for next in current.neighbours:
				if next != None and next != 1:
					new_cost = cost_so_far[current.name] + 1
					if next.name not in cost_so_far or new_cost < cost_so_far[next.name]:
						cost_so_far[next.name] = new_cost
						priority = new_cost
						frontier.put(next,priority)
						came_from[next.name] = current
		
		##Search for the closest node non full visited
		final_node = None
		min_cost = 100000
		for node in non_full_visited:
			aux = cost_so_far[node.name]
			if min_cost > aux:
				min_cost = aux
				final_node = node

		if goal != None:
			##Construction of the path
			actual = final_node
			closer = 100000
			k = 0
			for i in range(4):
				if actual.neighbours[i] == 1 and i == 0:
					dist = math.sqrt(math.pow(actual.row+1-goal[0],2) + math.pow(actual.col-goal[1],2))
					if closer > dist:
						closer = dist
						k = i
				if actual.neighbours[i] == 1 and i == 1:
					dist = math.sqrt(math.pow(actual.row-goal[0],2) + math.pow(actual.col-1-goal[1],2))
					if closer > dist:
						closer = dist
						k = i
				if actual.neighbours[i] == 1 and i == 2:
					dist = math.sqrt(math.pow(actual.row-1-goal[0],2) + math.pow(actual.col-goal[1],2))
					if closer > dist:
						closer = dist
						k = i
				if actual.neighbours[i] == 1 and i == 3:
					dist = math.sqrt(math.pow(actual.row-goal[0],2) + math.pow(actual.col+1-goal[1],2))
					if closer > dist:
						closer = dist
						k = i

			path.append(k)
		else:
			##Construction of the path
			actual = final_node
			for i in range(4):
				if actual.neighbours[i] == 1:
					path.append(i)
					break

		while came_from[actual.name] != None:
			path.append(came_from[actual.name].get_dir_neighbour(actual))
			actual = came_from[actual.name]
		path.reverse()
	
		return path, final_node


	def heuristic(self, row, col):
		return abs(row-self.posgoals[0][0]) + abs(col-self.posgoals[0][1])

	def change_initial(self, x, y, direction):
		self.initial_node = self.matrix[x][y]
		self.posrobots[0][0] = x
		self.posrobots[0][1] = y
		self.posrobots[0][2] = direction

	def compare_structures(self, m, actual_node):
		match = 0
		x = 0
		y = 0
		turn  = 0
		actual_dir = 0
		for k in range(1,5):
			#Rotate the matrix
			m = self.turn_matrix(m)
			#print m
			for i in range(len(m)):
				for j in range(len(m[0])):
					visited = []
					aux = self.find_match(m,actual_node,i,j,visited)
					if aux == True:
						match += 1
						x = i
						y = j
						turn = k

		for i in range(turn%4):
			aux = x
			if turn % 2 != 0:
				x = len(m) - y - 1
			else:
				x = len(m[0]) - y - 1
			y = aux

		return match, x, y, turn

	


	def find_match(self,m,node,x,y,visited):
		if node in visited:
			return True
		else:
			visited.append(node)
			if node != None and node != 1:
				if x < len(m) and x >= 0 and y < len(m[0]) and y >= 0:
					if m[x][y] == node.type:
						#print "x: ", x, " y: ", y, " m[x][y]: ", m[x][y], " node: ", node.type
						for i in range(4):
							if i == 0:
								if self.find_match(m,node.neighbours[i],x+1,y,visited) == False:
									return False
							elif i == 1:
								if self.find_match(m,node.neighbours[i],x,y-1,visited) == False:
									return False
							elif i == 2:
								if self.find_match(m,node.neighbours[i],x-1,y,visited) == False:
									return False
							else:
								if self.find_match(m,node.neighbours[i],x,y+1,visited) == False:
									return False
						return True
					else:
						return False
				else:
					return False
			else:
				return True
		
	def turn_matrix(self, m):
		m = list(zip(*m[::-1]))
		aux = [[0 for x in range(len(m[0]))] for y in range(len(m))]
		for i in range(len(m)):
			for j in range(len(m[0])):
				if m[i][j] == 0:
					aux[i][j] = 0
				elif m[i][j] == 1000:
					aux[i][j] = 100
				elif m[i][j] == 100:
					aux[i][j] = 10
				elif m[i][j] == 10:
					aux[i][j] = 1
				elif m[i][j] == 1:
					aux[i][j] = 1000
				elif m[i][j] == 1100:
					aux[i][j] = 110
				elif m[i][j] == 1010:
					aux[i][j] = 101
				elif m[i][j] == 1001:
					aux[i][j] = 1100
				elif m[i][j] == 110:
					aux[i][j] = 11
				elif m[i][j] == 101:
					aux[i][j] = 1010
				elif m[i][j] == 11:
					aux[i][j] = 1001
				elif m[i][j] == 111:
					aux[i][j] = 1011
				elif m[i][j] == 1011:
					aux[i][j] = 1101
				elif m[i][j] == 1101:
					aux[i][j] = 1110
				elif m[i][j] == 1110:
					aux[i][j] = 111
				elif m[i][j] == 1111:
					aux[i][j] = 1111
		return aux
