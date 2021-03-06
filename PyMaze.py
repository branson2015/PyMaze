import pygame
import time
import math
from pygame.locals import *
from functools import wraps
from enum import Enum
from collections import defaultdict
from random import randint

class Color(Enum):
    white = (255,255,255)
    black = (0,0,0)
    purple= (255,0,255)
    red   = (255,0,0)
    blue  = (0,0,255)
    green = (0,255,0)
    

class Direction(Enum):
    up    = 1
    right = 2
    down  = 3
    left  = 4
Directions = set([x for x in Direction])

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

        pygame.event.post(pygame.event.Event(pygame.USEREVENT))
        while(self._running):
            self.on_loop()

        


#width x height board w/ drawing functions
class Board(GraphViz):
    def __init__(self, windowSize, numTiles, wallSize):
        if windowSize != None:
            super().__init__(windowSize)
            self.no_visuals = False
            self._tileSize = (self.width/numTiles[0] - wallSize[0]/numTiles[0], self.height/numTiles[1] - wallSize[1]/numTiles[0])
            self._innerSize = (self._tileSize[0] - wallSize[0], self._tileSize[1] - wallSize[1])
            self._wallSize = wallSize
        self._numTiles = numTiles

    def genTile(self, pos, color=Color.white.value, timeout=0, dirs=None):
        if self.no_visuals:
            return
        paths = self.vertexToDirs(pos) if dirs == None else dirs
        pos = self.toXY(pos)
        rect = pygame.Rect((self._wallSize[0] + pos[0]*self._tileSize[0], self._wallSize[1] + pos[1]*self._tileSize[1]), self._innerSize)
        rec2 = None
        self._display_surf.fill(color, rect=rect)
        for path in paths:
            if path == Direction.up:
                rec2 = rect.move(0, -self._wallSize[1]-1)
            elif path == Direction.right:
                rec2 = rect.move(self._wallSize[0]+1, 0)
            elif path == Direction.down:
                rec2 = rect.move(0, self._wallSize[1]+1)
            elif path == Direction.left:
                rec2 = rect.move(-self._wallSize[0]-1, 0)
            self._display_surf.fill(color, rect=rec2)
        self.on_loop()
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
        return xy[1]*self._numTiles[0] + xy[0]

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
        return self.generate_edges()
    
    def matrix(self):
        return self.generate_matrix()

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

    def generate_edges(self):
        edges = []
        for vertex in self._graph_dict:
            for neighbour in self._graph_dict[vertex]:
                if {neighbour, vertex} not in edges:
                    edges.append({vertex, neighbour})
        return edges
    
    def generate_matrix(self):
        matrix = [[math.inf]*len(self.vertices()) for i in range(len(self.vertices()))]
        for vertex in self._graph_dict:
            matrix[vertex][vertex] = 0
            for neighbour in self._graph_dict[vertex]:
                matrix[vertex][neighbour] = 1
                matrix[neighbour][vertex] = 1
        return matrix

#Maze class, brings together graph and Board
class Maze(Board, Graph):
    class State(Enum):
        init = 0
        generate = 1
        solve = 2
        busy = 3
        quit = 4
        
    def __init__(self, windowSize, numTiles, wallSize=(1,1)):
        Board.__init__(self, windowSize, numTiles, wallSize)
        Graph.__init__(self)
        self._state = self.State.init
        self._numVertices = numTiles[0]*numTiles[1]
        self._rows = numTiles[0]
        self._columns = numTiles[1]
        for v in range(self._numVertices):
            self.add_vertex(v)

        self.genAlg = None
        self.solveAlg = None
        self.cont = True
    
    def vertexToDirs(self, v1):
        dirs = set()
        if v1 not in self._graph_dict:
            return dirs
        for v2 in self._graph_dict[v1]:
            if (v1 - self._numTiles[0]) == v2:
                dirs.add(Direction.up)
            elif (v1 + 1) == v2:
                dirs.add(Direction.right)
            elif (v1 + self._numTiles[0]) == v2:
                dirs.add(Direction.down)
            elif (v1 - 1) == v2:
                dirs.add(Direction.left)
        return dirs

    def edgeToDir(self, v1, v2):
        if (v1 - self._numTiles[0]) == v2:
            return Direction.up
        elif (v1 + 1) == v2:
            return Direction.right
        elif (v1 + self._numTiles[0]) == v2:
            return Direction.down
        elif (v1 - 1) == v2:
            return Direction.left
        else: 
            return None

    def dirToIndex(self, v, dir):
        if dir == Direction.up:
            return v - self._numTiles[0] if v - self._numTiles[0] >= 0 else None
        elif dir == Direction.right:
            return v + 1 if v%self._numTiles[0] + 1 < self._numTiles[0] else None
        elif dir == Direction.down:
            return v + self._numTiles[0] if v + self._numTiles[0] < self._numVertices else None
        elif dir == Direction.left:
            return v - 1 if v%self._numTiles[0] - 1 >= 0 else None
            
    def on_event(self, event):

        if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
            if self._state == self.State.busy:
                return
            self.cont = True 
            pygame.event.post(pygame.event.Event(pygame.USEREVENT))
        elif event.type == pygame.USEREVENT and self.cont:
            self.cont = False
            if self._state == self.State.init:

                if self.selectAlg("Select Generation Algorithm, or q for quit:", "generate") == 'q':
                    pygame.event.post(pygame.event.Event(pygame.USEREVENT+1))
                    return
                for event in pygame.event.get():
                    pass #ignore events

                self._state = self.State.generate
            elif self._state == self.State.generate:
                self._display_surf.fill(Color.black.value)

                self._state = self.State.busy
                self.genAlg(self)

                if self.selectAlg("Select Solve Algorithm, or q for quit:", "solve") == 'q':
                    pygame.event.post(pygame.event.Event(pygame.USEREVENT+1))
                    return
                for event in pygame.event.get():
                    pass #ignore events

                self._state = self.State.solve
            elif self._state == self.State.solve:

                self._state = self.State.busy
                path = self.solveAlg(self)
                vn = 1
                for v in range(0, len(path)-1):
                    self.genTile(path[v], Color.green.value, dirs=[self.edgeToDir(path[v],path[vn])])
                    vn += 1
                self.genTile(path[-1], Color.green.value)

                pygame.event.post(pygame.event.Event(pygame.USEREVENT+1))
        elif event.type == pygame.USEREVENT+1:
                self._state = self.State.init
                Graph.__init__(self)
                self.cont = True
                self._running = False
    
    def selectAlg(self, prompt, mode):
        algs = self.genAlgs if mode == "generate" else self.solveAlgs

        prompt += " ("
        for i, alg in enumerate(algs):
            if alg[-1] != '+':
                if i != 0:
                    prompt += " | "
                prompt += alg
                if mode == 'generate':
                    prompt += '[+]'
        prompt += ")\n"

        val = None
        while val == None:
            val = input(prompt)
            if val == 'q':
                return 'q'
            val = algs[val]
        
        if mode == "generate":
            self.genAlg = val
        elif mode == "solve":
            self.solveAlg = val

        return None


def rngBreakWalls(self, count):
    for i in range(count):
        v = randint(0, self._numVertices-1)
        newEdges = list(Directions - self.vertexToDirs(v))
        e = None
        if len(newEdges) == 0:
            continue
        elif len(newEdges) == 1:
            e = newEdges[0]
        else:
            e = newEdges[randint(0, len(newEdges)-1)]
        v2 = self.dirToIndex(v, e)
        if v2:
            self.add_edge((v, v2))
            self.genTile(v)


#decorator to make incorporating graph algorithms into the class easier
def add_method(cls, attr):
    def decorator(func):
        @wraps(func) 
        def wrapper(self, *args, **kwargs): 
            return func(self, *args, **kwargs)
        exists = getattr(cls, attr, None)
        if exists is None:
            setattr(cls, attr, defaultdict(lambda: None))
        getattr(cls, attr)[func.__name__] = wrapper
        
        if attr == 'genAlgs':
            @wraps(func)
            def wrapper(self, *args, **kwargs):
                ret = func(self, *args, **kwargs)
                rngBreakWalls(self, int(self._numVertices/20))
                return ret
            getattr(cls, attr)[func.__name__ + '+'] = wrapper

    return decorator

#decorator to make generating mazes easier
def generate():
    return add_method(Maze, 'genAlgs')


#decorator to make solving mazes easier
def solve():
    return add_method(Maze, 'solveAlgs')
