# Python program for Dijkstra's single  
# source shortest path algorithm. The program is  
# for adjacency matrix representation of the graph 
  
# Library for INT_MAX 
import sys 
from helpers.geometry import *
import numpy as np
  
class Graph(): 
  
	def __init__(self, vertices): 
		self.V = vertices 
		self.graph = [[0 for column in range(vertices)]  
					  for row in range(vertices)]

		# self.distance_matrix[i][j]: shortest distance from i to j
		self.distance_matrix = []
		# self.st_path_matrix[i][j]: nodes on the path from i to j, not include the middle points
		self.st_path_matrix = []
  
	def printSolution(self, dist): 
		print("Vertex tDistance from Source")
		for node in range(self.V): 
			print(str(node),"\t",str(dist[node]))
  
	# A utility function to find the vertex with  
	# minimum distance value, from the set of vertices  
	# not yet included in shortest path tree 
	def minDistance(self, dist, sptSet): 
  
		# Initilaize minimum distance for next node 
		min = sys.maxsize 
  
		# Search not nearest vertex not in the  
		# shortest path tree 
		for v in range(self.V): 
			if dist[v] < min and sptSet[v] == False: 
				min = dist[v]
				min_index = v 
  
		return min_index
  
	# Funtion that implements Dijkstra's single source  
	# shortest path algorithm for a graph represented  
	# using adjacency matrix representation 
	def dijkstra(self, src): 
  
		dist = [sys.maxsize] * self.V 
		dist[src] = 0
		sptSet = [False] * self.V
		# parent array is the parent node of each node in the shortest path
		parent = [-1]*self.V

  
		for cout in range(self.V): 
  
			# Pick the minimum distance vertex from  
			# the set of vertices not yet processed.  
			# u is always equal to src in first iteration 
			u = self.minDistance(dist, sptSet)
  
			# Put the minimum distance vertex in the  
			# shotest path tree 
			sptSet[u] = True
  
			# Update dist value of the adjacent vertices  
			# of the picked vertex only if the current  
			# distance is greater than new distance and 
			# the vertex in not in the shotest path tree
			index = u
			for v in range(self.V): 
				if (self.graph[u][v] > 0) and sptSet[v] == False and (dist[v] > dist[u] + self.graph[u][v]): 
						dist[v] = dist[u] + self.graph[u][v]
						parent[v] = u
		# self.printSolution(dist)
		# print(dist)

		# based on the parent array, could get the shortest path to each node
		st_path = self.shortest_path(parent)
		return [dist, st_path]

	def shortest_path(self, parent):
		st_path = []
		for i in range(self.V):
			st_path.append([i])
			current_parent = parent[i]
			while(current_parent != -1):
				st_path[i].append(current_parent)
				current_parent = parent[current_parent]
			st_path[i].reverse()
		# print(st_path)
		return st_path

	def generate_distance_st_matrix(self):
		for i in range(self.V):
			temp = self.dijkstra(i)
			self.distance_matrix.append(temp[0])
			self.st_path_matrix.append(temp[1])




# Useful graph functions
# Author -- Shikhar Dev Gupta

# Returns a list of points from the end to the start
def backtrace(parent, start, end):
	path = [end];
	while path[-1] != start:
		path.append(parent[path[-1]]);
	path.reverse();
	return path;

# Breadth First Search on a graph with a given Source and Target
# Graph - list of lists, adjacency list
# Source - the source node to start the search
# Target - the target node to search for

# But there is no distance calculation among the vertexes
def bfs(graph, source, target):
	queue = [];
	visited = {};
	parent = {};
	
	for node in range(len(graph)):
		visited[node] = False;
		parent[node] = None;
	
	queue.append(source);
	while len(queue) != 0:
		current = queue.pop(0);
		if current == target:
			res = backtrace(parent, source, target);
			return res;
		for neighbor in graph[current]:
			if visited[neighbor] == False:
				visited[neighbor] = True;
				parent[neighbor] = current;
				queue.append(neighbor);
	return None;

def get_adjacency_matrix(nodes):
	# get the adjacency matrix
	num_node = len(nodes)
	adjacency_matrix = np.zeros((num_node, num_node))
	for i in range(num_node):
		adjacency_matrix[i][i] = 0
		for j, node in enumerate(nodes[i].adjacent):
			next_index = node.index
			adjacency_matrix[i][next_index] = nodes[i].distance[j]
			adjacency_matrix[next_index][i] = nodes[i].distance[j]
	return adjacency_matrix
   
