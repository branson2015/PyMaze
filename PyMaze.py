import pygame
from pygame.locals import *
from functools import wraps
from enum import Enum

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

    def on_event(self, event):
        if event.type == pygame.QUIT:
            self._running = False
    
    def on_loop(self):
        pass

    def on_render(self):
        pass

    def on_cleanup(self):
        pygame.quit()

    def start(self):
        while(self._running):
            for event in pygame.event.get():
                self.on_event(event)
            self.on_loop()
            self.on_render()
        self.on_cleanup()



#Cell: contains both empty space AND walls:
'''
    # # # #
    # _ W # 
    # W W #
    # # # # 

    W = wall, size is wallSize
    _ = blank cell space, size is windowSize/numTiles - wallSize
'''

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
        self._tileSize = (self.width/numTiles[0], self.height/numTiles[1])
        self._numTiles = numTiles
        self._innerSize = (self._tileSize[0] - wallSize[0], self._tileSize[1] - wallSize[1])
        self._wallSize = wallSize

        self.edges = []

        #self.generate()

    def genTile(self, pos, paths):
        rect = pygame.Rect((pos[0]*self._tileSize[0], pos[1]*self._tileSize[1]), self._innerSize)
        self._display_surf.fill(Color.white.value, rect=rect)
        for path in paths:
            pass

    
    def generate(self):
        pass
    def solve(self):
        pass
    def on_render(self):
        pass
    def draw(self):
        pass

#graph class, to hold nodes and edges
class Graph:
    def __init__(self, nodes = None, edges = None):
        self._nodes = nodes
        self._edges = edges

#Maze class, brings together graph and Board
class Maze(Board, Graph):
    def __init__(self, windowSize, numTiles, wallSize):
        super().__init__(windowSize, numTiles, wallSize)

        for i in range(numTiles[0]):
            for j in range(numTiles[1]):
                self.genTile((i,j), [Direction.up.value])

    def on_render(self):
        pygame.display.update()
    
if __name__ == "__main__":
    app = Maze((800, 600), (13,10), (5,5))
    app.start()
    
    
    
###--- algorithms section ---###
    
    
    
@generate(Maze)
def DFSGen(self):
    self._nodes = self._numTiles[0]*self._numTiles[1]
    self._edges = []
    visited = [False for i in range(self._nodes)]  
    stack = [] 

    stack.append(0)  

    while(len(stack)):
        s = stack[-1]  
        stack.pop() 

        if (not visited[s]):  
            visited[s] = True 

        for node in self._edges[s]:  #need to modify this part to make DFS graph generation work
            if (not visited[node]):  
                stack.append(node)  

@solve(Maze)
def DFS(self):
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