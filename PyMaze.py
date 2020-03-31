import pygame
import time
from pygame.locals import *
from functools import wraps
from enum import Enum
from collections import defaultdict

#Implement the following solving algorithms and try and also convert them to be generation algorithms:
#DFS                - generating and solving done
#BFS                - generating and solving done
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
        self._running = True

        while(self._running):
            self.on_loop()


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

        self.genAlg = None
        self.solveAlg = None
    
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
                    self._display_surf.fill(Color.black.value)
                    self.genAlg(self)
                    self._state = self.State.generated
                elif self._state == self.State.generating:
                    pass
                    #pause
                elif self._state == self.State.generated:
                    self.solveAlg(self)
                    self._state = self.State.solved
                elif self._state == self.State.solved:
                    self._state = self.State.init
                    Graph.__init__(self)
                    self._running = False
    
    def selectAlg(self, prompt, mode):
        algs = self.genAlgs if mode == "generate" else self.solveAlgs

        prompt += " ("
        for i, alg in enumerate(algs):
            if i != 0:
                prompt += " | "
            prompt += alg
        prompt += ")\n"

        val = None
        while val == None:
            val = algs[input(prompt)]
        
        if mode == "generate":
            self.genAlg = val
        elif mode == "solve":
            self.solveAlg = val


#decorator to make incorporating graph algorithms into the class easier
def add_method(cls, attr):
    def decorator(func):
        @wraps(func) 
        def wrapper(self, *args, **kwargs): 
            return func(self, *args, **kwargs)
        exists = getattr(cls, attr, None)
        if exists is None:
            setattr(cls, attr, defaultdict(lambda: None))
        getattr(cls, attr)[func.__name__] = func
        return func
    return decorator

#decorator to make generating mazes easier
def generate():
    return add_method(Maze, 'genAlgs')

#decorator to make solving mazes easier
def solve():
    return add_method(Maze, 'solveAlgs')