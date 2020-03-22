import pygame
from pygame.locals import *
from functools import wraps
from enum import Enum
from random import random

class Color(Enum):
    white = (255,255,255)
    black = (0,0,0)

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
        self.on_render()

    def on_render(self):
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

    def genTile(self, pos, paths=[]):
        rect = pygame.Rect((self._wallSize[0] + pos[0]*self._tileSize[0], self._wallSize[1] + pos[1]*self._tileSize[1]), self._innerSize)
        rec2 = None
        self._display_surf.fill(Color.white.value, rect=rect)
        for path in paths:
            if path == Direction.up:
                rec2 = rect.move(0, -self._wallSize[1]-1)
            elif path == Direction.right:
                rec2 = rect.move(self._wallSize[0]+1, 0)
            elif path == Direction.down:
                rec2 = rect.move(0, self._wallSize[1]+1)
            elif path == Direction.left:
                rec2 = rect.move(-self._wallSize[0]-1, 0)
            self._display_surf.fill(Color.white.value, rect=rec2)

    def on_event(self, event):
        pass
    def generate(self):
        pass
    def solve(self):
        pass



#graph class, to hold nodes and edges
class Graph:
    def __init__(self, nodes = None, edges = None):
        self._nodes = nodes
        self._edges = edges

#Maze class, brings together graph and Board
class Maze(Board, Graph):
    class State(Enum):
        init = 0
        generated = 1
        solved = 2

    def __init__(self, windowSize, numTiles, wallSize):
        super().__init__(windowSize, numTiles, wallSize)
        self._state = self.State.init
    
    def on_event(self, event):
        if(event.type == pygame.KEYDOWN):
            if event.key == pygame.K_SPACE:
                if self._state == self.State.init:
                    self.generate()
                    self._state = self.State.generated
                elif self._state == self.State.generated:
                    self.solve()
                    self._state = self.State.solved
                elif self._state == self.State.solved:
                    pass


    
###--- algorithms section ---###


@generate(Maze)
def DFSGen(self):
    print("generating")

    self._nodes = self._numTiles[0]*self._numTiles[1]
    self._edges = []
    visited = [False for i in range(self._nodes)]  
    stack = [] 

    stack.append(0)  

    while(len(stack)):
        s = stack[-1]  
        stack.pop() 
        
        neighbors = filter(lambda s: visited[s], [s + self._numTiles[0], s+1, s - self._numTiles[0], s-1]) #todo, need to bound this somehow
        l = len(neighbors)
        
        if l > 0:
            stack.append(s)
        
        ns = neighbors[random()%l]

        #todo: knock down wall between ns and s

        visited[ns] = True
        stack.append(ns)

@solve(Maze)
def DFS(self):
    print("solving")

    visited = [False for i in range(self._nodes)]  
    stack = [] 

    stack.append(0)  

    while(len(stack)):
        s = stack[-1]  
        stack.pop() 

        if (not visited[s]):  
            visited[s] = True 

        for node in self._edges[s]:  
            if (not visited[node]):  
                stack.append(node)  
                








if __name__ == "__main__":
    app = Maze((800, 600), (13,10), (5,5)) #add in border?
    app.start()