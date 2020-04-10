import PyMaze
import algorithms

if __name__ == "__main__":
    app = PyMaze.Maze((800, 800), (100,100))

    while True:
        app.start()
        q = input("Quit? (y/n)\n")
        if q == "y":
            break
    app.on_cleanup()