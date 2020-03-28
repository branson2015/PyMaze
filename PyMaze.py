import pygame
import time
from pygame.locals import *
from functools import wraps
from enum import Enum
from random import randint

#TODO: come up with better decorator pattern to automate on_loop function call to make implementing algorithms more efficient
#make sure it doesn't override the OBJECT, just the INSTANCE.
#
#Implement the following solving algorithms and try and also convert them to be generation algorithms:
#BFS
#floyd-warshall
#dijkstra
#a*
#bellman-ford


class Color(Enum):
    white = (255,255,255)
    black = (0,0,0)
    purple= (255,0,255)
    green = (0,255,0)

class Direction(Enum):
    up    = 1
    right = 2
    down  = 3
    left  = 4

#wrapper around pygame for easier interfacing
class GraphViz:
    def __init__(self, windowSize):
        self.size = self.width, self.height = windowSize
        pygame.init()
        self._display_surf = pygame.display.set_mode(self.size, pygame.HWSURFACE | pygame.DOUBLEBUF)
        self._running = True

    def super_event(self, event):
        if event.type == pygame.QUIT:
            self._running = False
        else:
            self.on_event(event)
    
    def on_event(self, event):
        pass
    
    def on_loop(self):
        for event in pygame.event.get():
            self.super_event(event)
        self.render()

    def render(self):
        pygame.display.update()

    def on_cleanup(self):
        pygame.quit()

    def start(self):
        while(self._running):
            self.on_loop()
        self.on_cleanup()


#decorator to make incorporating graph algorithms into the class easier
def add_method(cls, fnName):
    def decorator(func):
        @wraps(func) 
        def wrapper(self, *args, **kwargs): 
            return func(self, *args, **kwargs)
        setattr(cls, fnName, wrapper)
        return func
    return decorator

#decorator to make generating mazes easier
def generate(cls):
    return add_method(cls, 'generate')

#decorator to make solving mazes easier
def solve(cls):
    return add_method(cls, 'solve')



#width x height board w/ drawing functions
class Board(GraphViz):
    def __init__(self, windowSize, numTiles, wallSize):
        super().__init__(windowSize)
        self._tileSize = (self.width/numTiles[0] - wallSize[0]/numTiles[0], self.height/numTiles[1] - wallSize[1]/numTiles[0])
        self._numTiles = numTiles
        self._innerSize = (self._tileSize[0] - wallSize[0], self._tileSize[1] - wallSize[1])
        self._wallSize = wallSize

    def genTile(self, pos, paths, color=Color.white, timeout=0):
        rect = pygame.Rect((self._wallSize[0] + pos[0]*self._tileSize[0], self._wallSize[1] + pos[1]*self._tileSize[1]), self._innerSize)
        rec2 = None
        self._display_surf.fill(color.value, rect=rect)
        for path in paths:
            if path == Direction.up:
                rec2 = rect.move(0, -self._wallSize[1]-1)
            elif path == Direction.right:
                rec2 = rect.move(self._wallSize[0]+1, 0)
            elif path == Direction.down:
                rec2 = rect.move(0, self._wallSize[1]+1)
            elif path == Direction.left:
                rec2 = rect.move(-self._wallSize[0]-1, 0)
            self._display_surf.fill(color.value, rect=rec2)
        time.sleep(timeout/1000)
    
    def getNeighbors(self, s):
        neighbors = []
        if (s/self._numTiles[0] - 1) >= 0:
            neighbors.append(s-self._numTiles[0])
        if (s%self._numTiles[0] + 1) < self._numTiles[0]:
            neighbors.append(s+1)
        if (s/self._numTiles[0] + 1) < self._numTiles[1]:
            neighbors.append(s+self._numTiles[0])
        if (s%self._numTiles[0] - 1) >= 0:
            neighbors.append(s-1)
        return neighbors

    def toXY(self, s):
        return (int(s%self._numTiles[0]), int(s/self._numTiles[0]))

    def toIndex(self, xy):
        return xy[0]*self._numTiles[0] + xy[1]

    def on_event(self, event):
        pass
    def generate(self):
        pass
    def solve(self):
        pass

#holds the vertices and edges
class Graph(object):
    def __init__(self, graph_dict=None):
        if graph_dict == None:
            graph_dict = {}
        self._graph_dict = graph_dict

    def vertices(self):
        return list(self._graph_dict.keys())

    def edges(self):
        return self._generate_edges()

    def edge(self, vertex):
        return self._graph_dict[vertex]

    def add_vertex(self, vertex):
        if vertex not in self._graph_dict:
            self._graph_dict[vertex] = []

    def add_edge(self, edge):
        edge = set(edge)
        (vertex1, vertex2) = tuple(edge)
        if vertex1 in self._graph_dict:
            self._graph_dict[vertex1].append(vertex2)
        else:
            self._graph_dict[vertex1] = [vertex2]
        if vertex2 in self._graph_dict:
            self._graph_dict[vertex2].append(vertex1)
        else:
            self._graph_dict[vertex2] = [vertex1]

    def _generate_edges(self):
        edges = []
        for vertex in self._graph_dict:
            for neighbour in self._graph_dict[vertex]:
                if {neighbour, vertex} not in edges:
                    edges.append({vertex, neighbour})
        return edges

#Maze class, brings together graph and Board
class Maze(Board, Graph):
    class State(Enum):
        init = 0
        generating = 1
        generated = 2
        solving = 3
        solved = 4
        
    def __init__(self, windowSize, numTiles, wallSize):
        Board.__init__(self, windowSize, numTiles, wallSize)
        Graph.__init__(self)
        self._state = self.State.init
        self._numVertices = numTiles[0]*numTiles[1]
        for v in range(self._numVertices):
            self.add_vertex(v)
    
    def vertexToDirs(self, v1):
        dirs = []
        for v2 in self._graph_dict[v1]:
            if (v1 - self._numTiles[0]) == v2:
                dirs.append(Direction.up)
            elif (v1 + 1) == v2:
                dirs.append(Direction.right)
            elif (v1 + self._numTiles[0]) == v2:
                dirs.append(Direction.down)
            elif (v1 - 1) == v2:
                dirs.append(Direction.left)
        return dirs

    def on_event(self, event):
        if(event.type == pygame.KEYDOWN):
            if event.key == pygame.K_SPACE:
                if self._state == self.State.init:
                    self.generate()
                    self._state = self.State.generated
                elif self._state == self.State.generating:
                    pass
                    #pause
                elif self._state == self.State.generated:
                    self.solve()
                elif self._state == self.State.solved:
                    pass


    
###--- algorithms section ---###


'''@generate(Maze)
def DFSGen(self):
    visited = [False]*self._numVertices
    stack = [] 

    stack.append(0)
    visited[0] = True  

    while(len(stack)):
        self.on_loop()
        s = stack.pop() 

        neighbors = self.getNeighbors(s)
        neighbors = [x for x in neighbors if not visited[x]]
        l = len(neighbors)
        
        if l == 0:
            self.genTile(self.toXY(s), self.vertexToDirs(s))
            continue
        
        stack.append(s)
        ns = neighbors[randint(0,l-1)]

        self.add_edge((s, ns))
        self.genTile(self.toXY(s), self.vertexToDirs(s))

        visited[ns] = True
        stack.append(ns)
'''

'''
@solve(Maze)
def DFS(self):
    start = 0
    end = self._numVertices-1
    path = None

    visited = set()
    stack = [(0, [0])]   

    while stack:
        self.on_loop()
        (v1,p) = stack.pop() 

        if v1 not in visited:  
            if v1 == end:
                path = p
                break
            visited.add(v1) 
            self.genTile(self.toXY(v1), self.vertexToDirs(v1), Color.purple)

        for v2 in self.edge(v1):  
            if v2 not in visited:  
                stack.append((v2, p+[v2]))  
                
    for v in path:
        self.genTile(self.toXY(v), self.vertexToDirs(v), Color.green)
'''

@generate(Maze)
def BFSGen(self):

    visited = [False]*self._numVertices
    queue = [0]
    visited[0] = True

    while queue:
        self.on_loop()
        s = queue.pop(0)

        neighbors = self.getNeighbors(s)
        neighbors = [x for x in neighbors if not visited[x]]
        l = len(neighbors)
        
        if l == 0:
            self.genTile(self.toXY(s), self.vertexToDirs(s))
            continue
        
        queue.append(s)
        ns = neighbors[randint(0,l-1)]

        self.add_edge((s, ns))
        self.genTile(self.toXY(s), self.vertexToDirs(s))

        visited[ns] = True
        queue.append(ns)

@solve(Maze)
def BFS(self):
    start = 0
    end = self._numVertices-1
    path = None

    visited = set()
    queue = [(0, [0])]   

    while queue:
        self.on_loop()
        (v1,p) = queue.pop(0) 

        if v1 not in visited:  
            if v1 == end:
                path = p
                break
            visited.add(v1) 
            self.genTile(self.toXY(v1), self.vertexToDirs(v1), Color.purple)

        for v2 in self.edge(v1):  
            if v2 not in visited:  
                queue.append((v2, p+[v2]))  
                
    for v in path:
        self.genTile(self.toXY(v), self.vertexToDirs(v), Color.green)




if __name__ == "__main__":
    app = Maze((800, 600), (130,100), (1,1)) #add in border?
    app.start()