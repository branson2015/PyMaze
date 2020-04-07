class DisjointSet:
    def __init__(self, nSets):
        self.Sets = [[i] for i in range(nSets)]
        self.Cells = [i for i in range(nSets)]
        self.nsets = nSets
    
    def __iter__(self):
        for s in self.Sets:
            if s == None:
                continue
            yield s
    
    def add(self, l):
        if l == None:
            l = len(self.Sets)
        self.Sets.append([l])
        self.Cells.append(l)
        self.nsets += 1

    def find(self, i):
        return self.Cells[i]

    def merge(self, i, j):
        i = self.find(i)
        j = self.find(j)

        if i == j:
            return i
        
        self.Sets[i].extend(self.Sets[j])

        for k in self.Sets[j]:
            self.Cells[k] = i
        self.Sets[j] = None
        self.nsets -= 1
        return i