##############################################################################
#   Laboratorio Remoto de Robotica Movel - TCC
#   Arquivo de PathPlanning
##############################################################################
#   Author: Paulo Felipe - paulof (at) ufop.edu.br
#
##############################################################################
#  Contem os algoritmos de busca de caminho
#  - A* (Implementado)
##############################################################################
from math import sqrt
import json
import numpy as np
import heapq
import sys
from typing import Protocol, Dict, List, Iterator, Tuple, TypeVar, Optional
import collections

sys.setrecursionlimit(10**6)
# Fila
class Queue:
    def __init__(self):
        self.elements = collections.deque()
    
    def empty(self):
        return not self.elements
    
    def put(self, x):
        self.elements.append(x)
    
    def get(self):
        return self.elements.popleft()

    def size(self):
        return len(self.elements)

# Fila Prioritaria
class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return not self.elements
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]

# Grid
class SquareGrid(object):
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.walls = []
    
    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height
    
    def passable(self, id):
        return id not in self.walls

    def isWalkable(self, id):
        return (self.in_bounds(id) == True and self.passable(id) == True)
    
    def neighbors(self, id, diagonals = False):
        (x, y) = id
        if diagonals == False:
            neighbors = [(x+1, y), (x-1, y), (x, y-1), (x, y+1)] # E W N S
        else:
            neighbors = [(x+1, y), (x-1, y), (x, y-1), (x, y+1), (x+1, y+1), (x+1, y-1), (x-1,y+1), (x-1, y-1)] # E W N S
        # see "Ugly paths" section for an explanation:
        if (x + y) % 2 == 0: neighbors.reverse() # S N W E
        results = filter(self.in_bounds, neighbors)
        results = filter(self.passable, results)
        return results

# Grid com pesos
class GridWithWeights(SquareGrid):
    def __init__(self, width, height):
        super(GridWithWeights, self).__init__(width, height)
        self.weights = {}
    
    def cost(self, from_node, to_node):
        return self.weights.get(to_node, 1)

def numPyToSquareGrid(maze, algoritmo = 1):
    (width, height) = maze.shape
    walls = []
    it = np.nditer(maze, flags=['multi_index'])
    for x in it:
        if x != 0:
            walls.append(it.multi_index)
    if algoritmo == 1:
        grid = SquareGrid(width, height) 
    elif algoritmo == 2:
        grid = GridWithWeights(width, height)    
    grid.walls = walls
    return grid

def reconstruct_path(came_from,
                     start, goal):
    print("Reconstruindo caminho")
    current = goal
    path = []
    try:
        while current != start:
            path.append(current)
            current = came_from[current]
    except Exception as e:
        print("Nenhum caminhono encontrado")
    path.append(start)
    path.reverse()
    print("Caminho: ", path)
    return path

def getHeuristic(start, end, heuristic):
    (xStart, yStart) = start
    (xGoal, yGoal) = end
    deltaX = abs(xStart - xGoal)
    deltaY = abs(yStart - yGoal)
    if(heuristic == 1):
        return sqrt(deltaX**2 + deltaY**2)
    elif(heuristic == 2):
        return deltaX + deltaY
    else:
        return max(deltaX, deltaY)

#
#   Funcao do Astar
#
def astar(maze, start, goal, maxCounter, experimento, experimentoStats):
    graph = numPyToSquareGrid(maze,2)
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    counter = 0
    

    while not frontier.empty():
        current = frontier.get()
        experimentoStats.iterationsCounter = experimentoStats.iterationsCounter + 1
        if current == goal:
            break
        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            counter = counter + 1
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + getHeuristic(next, goal, experimento.parametros.heuristica)
                frontier.put(next, priority)
                came_from[next] = current
                
    
    experimentoStats.visitedNodes = len(came_from)
    experimentoStats.iterationsCounter = counter
    return reconstruct_path(came_from,start,goal)

#
#   Funcao do Breadth First Search
#
def breadthFirstSearch(maze, start, goal, maxCounter, experimento, experimentoStats):
    graph = numPyToSquareGrid(maze)
    frontier = Queue()
    frontier.put(start)
    came_from = {}
    came_from[start] = None
    counter = 0
    while not frontier.empty():
        current = frontier.get()
        
        experimentoStats.iterationsCounter = experimentoStats.iterationsCounter + 1
        if current == goal:
            break
        
        for next in graph.neighbors(current):
            counter = counter + 1
            if next not in came_from:
                frontier.put(next)
                came_from[next] = current
    
    experimentoStats.visitedNodes = len(came_from)
    experimentoStats.iterationsCounter = counter
    return reconstruct_path(came_from,start,goal)

#
#   Funcao do BFS - Best-First-Search
#
def bestFirstSearch(maze, start, goal, maxCounter, experimento, experimentoStats):
    graph = numPyToSquareGrid(maze,2)
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    counter = 0

    while not frontier.empty():
        current = frontier.get()
        
        if current == goal:
            break

        for next in graph.neighbors(current):
            counter = counter + 1
            if next not in came_from:
                priority = getHeuristic(next, goal, experimento.parametros.heuristica)
                frontier.put(next, priority)
                came_from[next] = current
    
    experimentoStats.visitedNodes = len(came_from)
    experimentoStats.iterationsCounter = counter
    return reconstruct_path(came_from,start,goal)

#
#   Funcao do Jump Point
#

def jumpPoint(maze, start, goal, maxCounter, experimento, experimentoStats):
    graph = numPyToSquareGrid(maze,2)
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    counter = 0
    while not frontier.empty():
        current = frontier.get()
        
        experimentoStats.iterationsCounter = experimentoStats.iterationsCounter + 1
        if current == goal:
            break
        for next in findSuccessors(graph, current, start, goal):
            new_cost = cost_so_far[current] + graph.cost(current, next)

            counter = counter + 1
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + getHeuristic(next, goal, experimento.parametros.heuristica)
                frontier.put(next, priority)
                came_from[next] = current
       
    experimentoStats.visitedNodes = len(came_from)
    experimentoStats.iterationsCounter = counter

    return reconstruct_path(came_from,start,goal)

# Clamp Function
def clamp(n, smallest, largest): return max(smallest, min(n, largest))

#
#   Find Successors JP Search
#
def findSuccessors(graph, current, start, goal):
    successors = []
    neighbours = graph.neighbors(current, True)
    notSuccessors = []
    for neighbour in neighbours:
        dx = clamp(neighbour[0] - current[0], -1, 1)
        dy = clamp(neighbour[1] - current[1], -1, 1)
        jumpPoint = jump(current, (dx, dy), graph, goal)
        if jumpPoint != False:
            successors.append(jumpPoint)
        else:
            notSuccessors.append(jumpPoint)
    return successors

#
#   Jump Point
#
def jump(current, direction, graph, goal):
    dx = direction[0]
    dy = direction[1]    
    jumpPosition = (current[0] + direction[0],current[1] + direction[1])

    if graph.isWalkable(jumpPosition) == False:
        return False
    
    if (jumpPosition[0] == goal[0] and jumpPosition[1] == goal[1]):
        return jumpPosition

    # Checa vizinhos forcados
    # Horizontal
    if (dx != 0 and dy == 0):
        #print("horizontal")

        # Regras de Prune
        pruneHorizontal1 = (current[0], current[1] + 1)
        pruneHorizontal2 = (current[0] + dx, current[1] + 1)
        pruneHorizontal3 = (current[0], current[1] - 1)
        pruneHorizontal4 = (current[0] + dx, current[1] -1)

        if (graph.isWalkable(pruneHorizontal1) == False and graph.isWalkable(pruneHorizontal2) == True):
            #print('vizinho forcado horizontal 1')
            return jumpPosition
        elif (graph.isWalkable(pruneHorizontal3) == False and graph.isWalkable(pruneHorizontal4) == True):
            #print('vizinho forcado horizontal 2')
            return jumpPosition

    # Vertical
    elif (dx == 0 and dy != 0):
        #print("vertical")
        if (graph.isWalkable((current[0] +1, current[1])) == False and graph.isWalkable((current[0] + 1, current[1] + dy)) == True):
            #print('vizinho forcado vertical 1')
            return jumpPosition
        elif (graph.isWalkable((current[0] -1, current[1])) == False and graph.isWalkable((current[0] -1, current[1] + dy)) == True):
            #print('vizinho forcado vertical 2')
            return jumpPosition
    # Diagonais
    elif (dx != 0 and dy != 0):
        #print("diagonal")
        if graph.isWalkable((current[0] + dx, current[1])) == False:
            #print('vizinho forcado diagonal 1')
            return jumpPosition
        elif graph.isWalkable((current[0], current[1] + dy)) == False:
            #print('vizinho forcado diagonal 2')
            return jumpPosition
        
        if(jump(jumpPosition, (dx,0), graph, goal) != False or jump(jumpPosition, (0,dy), graph, goal) != False):
            #print('vizinho forcado diagonal 3')
            return jumpPosition
    return jump(jumpPosition, direction, graph, goal)

#
#   Funcao de pegar a heuristica
#