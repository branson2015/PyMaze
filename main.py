import PyMaze
import algorithms

if __name__ == "__main__":
    app = PyMaze.Maze((800, 600), (100,75), (1,1))

    while True:
        app.selectAlg("Select Generation Algorithm:", "generate")
        app.selectAlg("Select Solve Algorithm:", "solve")
        app.start()
        q = input("Quit? (y/n)\n")
        if q == "y":
            break
    app.on_cleanup()