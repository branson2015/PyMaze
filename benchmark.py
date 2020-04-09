from PyMaze import *
import algorithms
import time

#Dead Ends
#Straightaways
#Turns
#Junctions
#Crossroads
#time

##########

#needed to instrument in code:
#number of loops
#number of visited
#number of other stuff

class Benchmark(Maze):
    def __init__(self):
        Maze.__init__(self, None, (100,100))
        self.no_visuals = True

    def benchmark_all(self):
        self.benchmark_solve_all(gen=True)

    def benchmark(self, a, b):
        stats = self.benchmark_solve(a, b, True)
        print(stats)

    def benchmark_solve_all(self, gen=False):
        for galg in self.genAlgs:
            for salg in self.solveAlgs:
                self.__init__()
                stats = self.benchmark_solve(galg, salg, gen)
                print(galg, salg, stats)
            self.__init__()
    
    def benchmark_solve(self, a, b, gen=False):
        stats = {}
        if gen:
            stats["gen"] = self.benchmark_gen(a)
        else:
            self.genAlg = self.genAlgs[a] if isinstance(a, str) else a
            self.genAlg(self)
        self.solveAlg = self.solveAlgs[b] if isinstance(b, str) else b

        

        t0 = time.time()
        self.solveAlg(self)
        ttime = time.time() - t0

        stats["solve"] = {}
        stats["solve"]["time"] = ttime

        return stats

    def benchmark_gen_all(self):
        for alg in self.genAlgs:
            self.__init__()
            stats = self.benchmark_gen(alg)
            print(alg, stats)
        self.__init__()

    def benchmark_gen(self, a):
        self.genAlg = self.genAlgs[a] if isinstance(a, str) else a

        t0 = time.time()
        self.genAlg(self)
        ttime = time.time() - t0
        
        stats = {}
        deadends, straights, turns, junctions, crossroads = self.calc_dirs()
        stats["deadends"]       = deadends
        stats["straightAways"]  = straights
        stats["turns"]          = turns
        stats["junctions"]      = junctions
        stats["crossroads"]     = crossroads
        stats["time"]           = ttime

        return stats
    
    def calc_deadEnds(self):
        pass
    
    def calc_dirs(self):
        straightCount = 0
        turnCount = 0
        junctionCount = 0
        crossroadCount = 0
        deadendCount = 0

        DIR = [0,1,2,1,2]
        for v in self.vertices():
            e = self.edge(v)
            l = len(e)
            if l == 1:
                deadendCount += 1
            elif l == 2:
                e1 = self.edgeToDir(e[0],v) #down or right
                e2 = self.edgeToDir(e[1],v) #up or left
                dir1 = DIR[e1.value]
                dir2 = DIR[e2.value]
                if dir1 == dir2:
                    straightCount += 1
                else:
                    turnCount += 1
            elif l == 3:
                junctionCount += 1
            elif l == 4:
                crossroadCount += 1
        return (deadendCount, straightCount, turnCount, junctionCount, crossroadCount)

    




def main():
    bm = Benchmark()
    bm.benchmark("Ellers", "A_star")

if __name__ == "__main__":
    main()