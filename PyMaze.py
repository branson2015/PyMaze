import pygame
from pygame.locals import *

class GraphViz:
    def __init__(self):
        self._running = True
        self._display_surf = None
        self.size = self.width, self.height = 800, 800
    
    def on_init(self):
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
        if self.on_init() == False:
            self._running = False
        
        while(self._running):
            for event in pygame.event.get():
                self.on_event(event)
            self.on_loop()
            self.on_render()
        self.on_cleanup()


if __name__ == "__main__":
    app = GraphViz()
    app.start()


def abstractAlgorithm(args):
    def decorator(func):
        def inner(*args, **kwargs):
            return func(*args, **kwargs)
        return inner
    return decorator



def algorithm1(nodes, edges):
    visited = [False for i in range(nodes)]  
    stack = [] 

    stack.append(0)  

    @While(len(stack))
    def inner():  
        s = stack[-1]  
        stack.pop() 

        if (not visited[s]):  
            visited[s] = True 

        for node in edges[s]:  
            if (not visited[node]):  
                #do stuff here
                stack.append(node)  