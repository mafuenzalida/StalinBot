from node import Node;
from maze import Maze;
import cv2
from cv2 import matchTemplate as cv2m


def compare_structures(m, actual_node):
	match = 0
	x = 0
	y = 0
	turn  = 0
	for k in range(1,5):
		#Rotate the matrix
		print "m1: ", m
		m = turn_matrix(m)
		print m
		for i in range(len(m)):
			for j in range(len(m[0])):
				if k == 3 and i == 2 and j == 2:
					print "asd"
				visited = []
				aux = find_match(m,actual_node,i,j,visited)
				if aux:
					match += 1
					x = i
					y = j
					turn = k

	print x, y
	for i in range(turn%4):
s
		#y = len(m) - 1 - x
		#x = y		
		x = len(m) - 1 - y
		y = x

	return match, x, y, turn


def find_match(m,node,x,y,visited):
	if node in visited:
		return True
	else:
		visited.append(node)
		if node != None and node != 1:
			if x < len(m) and x >= 0 and y < len(m[0]) and y >= 0:
				if m[x][y] == node.type:
					print "x: ", x, " y: ", y, " m[x][y]: ", m[x][y], " node: ", node.type
					for i in range(4):
						if i == 0:
							if find_match(m,node.neighbours[i],x+1,y,visited) == False:
								return False
						elif i == 1:
							if find_match(m,node.neighbours[i],x,y-1,visited) == False:
								return False
						elif i == 2:
							if find_match(m,node.neighbours[i],x-1,y,visited) == False:
								return False
						else:
							if find_match(m,node.neighbours[i],x,y+1,visited) == False:
								return False
					return True
				else:
					return False
			else:
				return False
		else:
			return True

def turn_matrix(m):
	m = list(zip(*m[::-1]))
	print "m2: ", m
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

m = Maze()
m.read_maze()

a = Node(1,0,1,1,0,0,0)
n2 = Node(0,1,0,0,1,0,0)
n3 = Node(0,0,1,0,0,0,0)
n4 = Node(0,1,1,0,0,0,0)

a.add_neighbour(n2,1)
n2.add_neighbour(a,3)
print a.neighbours
print n2.neighbours
#n2.add_neighbour(n3,2)
#n3.add_neighbour(n4,1)

print compare_structures(m.m, a)

print m.m
