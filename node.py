#!/usr/bin/env python
#-*- coding: utf-8 -*-

class Node:
	
	def __init__(self,u,l,d,r,name,row,col):
		self.type = ""
		self.neighbours = [None for i in range(4)]
		if u == 0:
			self.neighbours[0] = 1
			self.type += "1"
		else:
			self.type += "0"
		if l == 0:
			self.neighbours[1] = 1
			self.type += "1"
		else:
			self.type += "0"
		if d == 0:
			self.neighbours[2] = 1
			self.type += "1"
		else:
			self.type += "0"
		if r == 0:
			self.neighbours[3] = 1
			self.type += "1"
		else:
			self.type += "0"

		self.type = int(self.type)
		self.name = name
		self.row = row
		self.col = col
		self.friend = False
		self.key = False
		

	def get_neighbour(self, direction):
		return self.neighbours[direction]

	def add_neighbour(self, node, direction):
		self.neighbours[direction] = node

	def add_wall(self, direction):
		self.neighbours[direction] = None
	
	def get_dir_neighbour(self, node):
		for i in range(4):
			if self.neighbours[i] == node:
				return i

	def set_friend(self):
		self.friend = True
	
	def get_friend(self):
		return self.friend

	def set_key(self):
		self.key = True
	
	def get_key(self):
		return self.key
		
