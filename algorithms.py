from PyMaze import Color, Direction, generate, solve
from random import randint

'''
Your function must take a single parameter - "self".
You must precede your function with either an @generate or @solve decorator.
This will hook your function into the Maze & Graph classes for you.

The following functions are accessible to you through self:

    FUNCTIONS
    _____________________________________________________________________________________________________________________________________________

    self.on_loop() - causes the screen to update with the newest information (e.g from genTile)

    self.genTile(XY, Direction, [Color])
        XY - Tuple of (X, Y) coordinate to generate a cell in
        Direction - of ARRAY consiting of type Direction.up, Direction.down, Direction.left, or Direction.right
        Color - optional - of type Color.xxx - look in PyMaze.py if you'd like to add your own color

        This function generates a tile at position X,Y with connections listed in Direction and fill-color of Color
    
    self.vertexToDirs(vertex)
        vertex - the vertex to look up the corresponding connected edges

        This function returns the array of Directions that vertex v is connected to, if such a connection exists.
        This function is to be used in tangent with self.genTile. See DFS for example.
    
    self.getNeighbors(cell)
        cell - the cell index (=X*width+height) to return the 4 surrounding cell index neighbors of (always returns all 4, unless out of bounds)

        This function is a helper function for getting the surrounding cells.

    self.toXY(cell)
        cell - the cell index (=X*width+height) to change to an (X,Y) Tuple representing the same index

        This function is a helper function for converting between 1d and 2d space

    self.add_edge(vertex1, vertex2):
        vertex1 - the first vertex to draw an edge between
        vertex2 - the second vertex to draw an edge between

        this function is to be used ONLY in @generate functions for defining the underlying graph that the maze is solved upon


    VARIABLES
    _____________________________________________________________________________________________________________________________________________

    self._numVertices - the number of vertices in the graph (=numCellWidth * numCellHeight)

    
'''


#############################################
#########-----------DFS-------------#########
#############################################


@generate()
def DFS(self):
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



@solve()
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




#############################################
#########-----------BFS-------------#########
#############################################

@generate()
def BFS(self):
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

@solve()
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



