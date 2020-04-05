from PyMaze import Color, Direction, Directions, generate, solve
from random import randint, sample
import math
from collections import defaultdict
'''
Your function must take a single parameter - "self".
Your function must return the solution path
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

    self.edges():

    self.matrix():

    VARIABLES
    _____________________________________________________________________________________________________________________________________________

    self._numVertices - the number of vertices in the graph (=numCellWidth * numCellHeight)

    
'''

#Implement the following solving algorithms and try and also convert them to be generation algorithms:
#Wilsons            - generating done
#DFS/DFS+           - generating and solving done
#BFS/BFS+           - generating and solving done
#floyd-warshall     - solving done
#dijkstra           - solving done
#A*                 - solving done
#bellman-ford       - solving done
#Kruskals
#Prims              - solving done
#Kruskal's          - TODO WIP - ETHAN

#hunt and kill
#Ellers
#SideWinder
#binary tree



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


@generate()
def PRIM(self):
    """PRIM algorthihm"""
    start_index = randint(0, self._numVertices - 1)
    # start_index = 0
    maze = {start_index}
    frontier = set(self.getNeighbors(start_index))
    self.genTile(start_index)

    # while we haven't added all the cells into the set
    while len(maze) != self._numVertices:

        # grab random cell from the frontier set
        random_cell = sample(frontier, 1)[0]
        frontier.remove(random_cell)
        # add cells not part of the maze
        connections = set()
        for open_cell in self.getNeighbors(random_cell):
            if open_cell not in maze and open_cell not in frontier:
                frontier.add(open_cell)
            if open_cell in maze:
                connections.add(open_cell)

        # now pick a random part of the maze to connect to this random cell
        random_maze_cell = sample(connections, 1)[0]
        self.add_edge((random_cell, random_maze_cell))
        self.genTile(random_cell)
        maze.add(random_cell)



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
        s = stack.pop() 

        neighbors = self.getNeighbors(s)
        neighbors = [x for x in neighbors if not visited[x]]
        l = len(neighbors)
        
        if l == 0:
            self.genTile(s)
            continue
        
        stack.append(s)
        ns = neighbors[randint(0,l-1)]

        self.add_edge((s, ns))
        self.genTile(s)

        visited[ns] = True
        stack.append(ns)

dfs = DFS
@generate()
def DFSplus(self):
    dfs(self)
    rngBreakWalls(self, int(self._numVertices/20))


@solve()
def DFS(self):
    start = 0
    end = self._numVertices-1
    path = None

    visited = set()
    stack = [(0, [0])]   

    while stack:
        (v1,p) = stack.pop() 

        if v1 not in visited:  
            if v1 == end:
                path = p
                break
            visited.add(v1) 
            self.genTile(v1, Color.red.value)

        for v2 in self.edge(v1):  
            if v2 not in visited:  
                stack.append((v2, p+[v2]))  
                
    return path


#############################################
#########-----------BFS-------------#########
#############################################

@generate()
def BFS(self):
    visited = [False]*self._numVertices
    queue = [0]
    visited[0] = True

    while queue:
        s = queue.pop(0)

        neighbors = self.getNeighbors(s)
        neighbors = [x for x in neighbors if not visited[x]]
        l = len(neighbors)
        
        if l == 0:
            self.genTile(s)
            continue
        
        queue.append(s)
        ns = neighbors[randint(0,l-1)]

        self.add_edge((s, ns))
        self.genTile(s)

        visited[ns] = True
        queue.append(ns)

bfs = BFS
@generate()
def BFSplus(self):
    bfs(self)
    rngBreakWalls(self, int(self._numVertices/20))

@solve()
def BFS(self):
    start = 0
    end = self._numVertices-1
    path = None

    visited = set()
    queue = [(0, [0])]   

    while queue:
        (v1,p) = queue.pop(0) 

        if v1 not in visited:  
            if v1 == end:
                path = p
                break
            visited.add(v1) 
            self.genTile(v1, Color.red.value)

        for v2 in self.edge(v1):  
            if v2 not in visited:  
                queue.append((v2, p+[v2]))  
                
    return path


#############################################
#########-------Hunt and Kill-------#########
#############################################
@generate()
def HuntandKill(self):

    start = 0

    notVisited = [x for x in range(self._numVertices)]
    notVisited.remove(start)
    s = start

    while(notVisited):

        neighbors = self.getNeighbors(s)
        neighbors = [x for x in neighbors if x in notVisited]
        l = len(neighbors)
        
        if l == 0:
            self.genTile(s)
            for v in notVisited:
                adjacentUsedNeighbors = [x for x in self.getNeighbors(v) if x not in notVisited]
                if len(adjacentUsedNeighbors) > 0:
                    s = v
                    self.add_edge((s, adjacentUsedNeighbors[0]))
                    self.genTile(s)
                    break
            notVisited.remove(s)
            continue
        
        ns = neighbors[randint(0,l-1)]
        notVisited.remove(ns)
        self.add_edge((s, ns))
        self.genTile(s)
        s = ns


#############################################
#########----------Wilsons----------#########
#############################################
@generate()
def Wilsons(self):

    notInMaze = [i for i in range(self._numVertices)]
    
    '''
    leads to disjoint sets - need to manage that
    gap = (10,10)
    for i in range(0, self._numTiles[1], gap[1]):
        for j in range(0, self._numTiles[0], gap[0]):
            start = self.toIndex((i,j))
            notInMaze.remove(start)
            self.genTile(start)
    '''

    start = self.toIndex((int(self._numTiles[1]/2), int(self._numTiles[0]/2)))
    notInMaze.remove(start)
    self.genTile(start)

    s = notInMaze[randint(0, len(notInMaze)-1)]
    walk = [s]
    while notInMaze:
        self.genTile(s)
    
        neighbors = self.getNeighbors(s)
        l = len(neighbors)
        ns = neighbors[randint(0,l-1)]

        if ns not in walk:
            self.add_edge((s, ns))
            walk.append(ns)
        
        if ns in notInMaze:
            s = ns
        else:
            notInMaze = [x for x in notInMaze if x not in walk]
            self.genTile(s)
            self.genTile(ns)
            if len(notInMaze) == 0:    
                break
            s = notInMaze[randint(0, len(notInMaze)-1)]
            walk = [s]


#############################################
#########------floyd_Warshall-------#########
#############################################

@solve()
def FloydWarshall(self): 
    Next = defaultdict(lambda: defaultdict(lambda: None))
    dist = list(map(lambda i : list(map(lambda j : j , i)), self.matrix()))

    for (u, v) in self.edges():
        Next[u][v] = v
        Next[v][u] = u
    
    for v in range(self._numVertices):
        Next[v][v] = v

    maxVal = 1
    for k in range(self._numVertices):  
        updates = {}
        for i in range(self._numVertices): 
            for j in range(self._numVertices):  
                if dist[i][j] > dist[i][k] + dist[k][j]:
                    dist[i][j] = dist[i][k] + dist[k][j]
                    Next[i][j] = Next[i][k]
                    val = int(dist[i][j]/maxVal) if dist[i][j] is not math.inf else 0
                    maxVal = max(maxVal, val)
                    val /= maxVal
                    updates[j] = val
        for j in updates:
            self.genTile(j, (255,255-255*val,255-255*val))

    #path reconstruction to display result
    path = [0]
    u = 0
    v = self._numVertices-1
    while u != v:
        u = Next[u][v]
        path.append(u)
    return path


#############################################
#########---------Dijkstra----------#########
#############################################


@solve()
def Dijkstra(self):
    
    Q = set()
    dist = []
    prev = []

    start = 0
    end = self._numVertices-1

    for v in range(self._numVertices):
        dist.append(math.inf)
        prev.append(None)
        Q.add(v)
    dist[start] = 0

    while Q:
        u = min(Q, key=lambda v: dist[v])
        Q.remove(u)

        if u == end:
            break
        
        maxVal = 1
        for v in self.edge(u):
            alt = dist[u] + 1
            if alt < dist[v]:
                dist[v] = alt
                prev[v] = u
                maxVal = max(maxVal, alt)
                val = alt/maxVal
                self.genTile(v, (255,255-255*val,255-255*val))

    path = []
    u = end
    while u:
        path.insert(0, u)
        u = prev[u]
    path.insert(0, start)
    return path


#############################################
#########------------A*-------------#########
#############################################

@solve()
def A_star(self):
    start = 0
    end = self._numVertices-1

    def h(v):
        v = self.toXY(v)
        g = self.toXY(end)
        return (g[0]-v[0]) + (g[1]-v[1])

    openSet = set([start])
    closeSet = set()

    cameFrom = {}

    gscore = defaultdict(lambda: math.inf)
    gscore[start] = 0

    fscore = defaultdict(lambda: math.inf)
    fscore[start] = h(start)

    while openSet:
        current = min(openSet, key=lambda x:fscore.get(x))
        if current == end:
                path = [current]
                while current in cameFrom.keys():
                    current = cameFrom[current]
                    path.insert(0, current)
                return path

        closeSet.add(current)
        openSet.remove(current)
        self.genTile(current, Color.red.value)

        for neighbor in self.edge(current):
            tentative_gScore = gscore[current] + 1  #assume each has edge weight of 1
            if tentative_gScore < gscore[neighbor]:
                cameFrom[neighbor] = current
                gscore[neighbor] = tentative_gScore
                fscore[neighbor] = gscore[neighbor] + h(neighbor)
                if neighbor not in closeSet:
                    openSet.add(neighbor)


#############################################
#########-------Bellman-Ford--------#########
#############################################

@solve()
def BellmanFord(self):

    start = 0
    end = self._numVertices-1

    dist = [math.inf]*self._numVertices
    prev = [None]*self._numVertices
    dist[start] = 0

    for i in range(self._numVertices-1): 
        maxval = 1#
        change = {}#
        for u, v in self.edges():
            if dist[u] + 1 < dist[v]: 
                dist[v] = dist[u] + 1  
                prev[v] = u
                change[v] = dist[v]#
                maxval = max(maxval, dist[v])#
            elif dist[v] + 1 < dist[u]:
                dist[u] = dist[v] + 1
                prev[u] = v
                change[u] = dist[u]#
                maxval = max(maxval, dist[u])#
        
        for i in change.keys():#
            val = min(change[i]/maxval*255,255)#
            self.genTile(i, (val,0,0))#

    #just a check, might not include in final result
    for u, v in self.edges(): 
        if dist[u] != math.inf and dist[u] + 1 < dist[v]:
            return None
                        
    path = []
    u = end
    while u:
        path.insert(0, u)
        u = prev[u]
    path.insert(0, start)
    return path