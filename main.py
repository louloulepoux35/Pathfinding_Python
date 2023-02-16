from tkinter import *
import numpy as np
from enum import Enum
import math
from abc import ABC, abstractmethod
import threading
import time

# TODO: Forbid diagonal button
# TODO: Teleport through screen border button ?


SPEED = 10
BOARD_SIZE = 600
GRID_RES = 50
GUI_SIZE = 300
START_X = 5
START_Y = 25
END_X = 45
END_Y = 25


class CellType(Enum):
    EMPTY = 0
    WALL = 1


class Cell:
    def __init__(self, pX, pY):
        self.type = CellType.EMPTY
        self.parentCell = None

        # Astar membres
        self.startToThisLengh = 0
        self.heuristicLenght = np.inf
        self.estimatedPathLenght = np.inf

        # Dijkstras membres
        self.curentDist = np.inf

        self.x = pX
        self.y = pY
        self.pathFindingDrawColor = 'grey'


# Base class for differents pathfinding algorithms
class PathFinder(ABC):
    @abstractmethod
    def __init__(self, startCell, grid, allowDiag):
        self.diagonalForbidden = allowDiag

    @abstractmethod
    def reset(self, startCell, grid, allowDiag):
        self.diagonalForbidden = allowDiag

    @abstractmethod
    def computeOneStep(self, grid, endCell, drawJustPath):
        pass

    def heuristic(self, curentCell, endCell):
        xDist = math.fabs(endCell.x - curentCell.x)
        yDist = math.fabs(endCell.y - curentCell.y)
        return math.sqrt(xDist * xDist + yDist * yDist)

    def colorShortestPath(self, curentCell):
        cellTmp = curentCell
        while cellTmp.parentCell != None:
            cellTmp.parentCell.pathFindingDrawColor = 'blue'
            cellTmp = cellTmp.parentCell


    def loopThroughAllWalkableNeighbours(self, curentCell, grid, callbackFunc):
        # Loop through each adjacent cell
        for x in curentCell.x + np.array([-1, 0, 1]):
            for y in curentCell.y + np.array([-1, 0, 1]):
                if x >= 0 and y >= 0 and x < np.size(grid, 0) and y < np.size(grid, 0):
                    if not (x == curentCell.x and y == curentCell.y):
                        adjCell = grid[x][y]
                        # Not walkable
                        if adjCell.type == CellType.WALL:
                            continue
                        if self.diagonalForbidden and x != curentCell.x and y != curentCell.y:
                            continue
                        # Call the callback
                        callbackFunc(adjCell)


class Dijkstras(PathFinder):
    def __init__(self, startCell, grid, allowDiag):
        # Call parent class' constructor
        super(Dijkstras, self).__init__(startCell, grid, allowDiag)

        self.unexploredCells = []
        self.stop = None
        self.reset(startCell, grid, allowDiag)


    def reset(self, startCell, grid, allowDiag):
        # Call parent class' reset method
        super(Dijkstras, self).reset(startCell, grid, allowDiag)

        # Add every cell in the inexplored list
        #self.unexploredCells = [cell for cell in np.nditer(grid, flags=["refs_OK"]) if cell.type == CellType.EMPTY]
        self.unexploredCells.clear()
        for x in range(np.size(grid, 0)):
            for y in range(np.size(grid, 1)):
                if grid[x][y].type == CellType.EMPTY:
                    self.unexploredCells.append(grid[x][y])

        # Init grid cells
        #for cell in np.nditer(grid):
        for x in range(np.size(grid, 0)):
            for y in range(np.size(grid, 1)):
                grid[x][y].curentDist = np.inf

        # Set starting cell distance to 0
        startCell.curentDist = 0

        self.stop = False


    def computeOneStep(self, grid, endCell, drawJustPath):
        if self.stop:
            return False
        if len(self.unexploredCells) == 0:
            self.stop = True
            return False

        # Find the cell with the smallest distance
        curentCell = min(self.unexploredCells, key=lambda x: x.curentDist)
        curCellIndex = self.unexploredCells.index(curentCell)

        if curentCell == endCell:
            self.stop = True
            # Color path
            self.colorShortestPath(curentCell)
            return False

        # Remove if from the unexplored list
        self.unexploredCells.pop(curCellIndex)

        # Code to be executed on each adjacent cells
        def adjCellInnerFunc(adjCell):
            # Compute distance
            dist = curentCell.curentDist + self.heuristic(curentCell, adjCell)
            if dist < adjCell.curentDist:
                # Set new distance
                adjCell.curentDist = dist
                # Set parent cell
                adjCell.parentCell = curentCell

        # Loop through each adjacent cell
        self.loopThroughAllWalkableNeighbours(curentCell, grid, adjCellInnerFunc)

        # Color
        if not drawJustPath:
            curentCell.pathFindingDrawColor = 'brown'

        return True



class Astar(PathFinder):
    def __init__(self, startCell, grid, allowDiag):
        # Call parent class' constructor
        super(Astar, self).__init__(startCell, grid, allowDiag)

        self.stop = None
        self.closedList = None
        self.openList = None
        self.reset(startCell, grid, allowDiag)

    def reset(self, startCell, grid, allowDiag):
        # Call parent class' reset method
        super(Astar, self).reset(startCell, grid, allowDiag)

        self.openList = [startCell]  # Add first cell in the open list
        self.closedList = []
        self.stop = False
        startCell.parentCell = None # Avoid inf loop if run multiple times


    def computeOneStep(self, grid, endCell, drawJustPath):
        if self.stop:
            return False
        if len(self.openList) == 0:
            self.stop = True
            return False

        # Look for the shortest estimated path on the open list
        #currentcellIndex = -1
        #lowestPath = np.inf
        #for i in range(len(self.openList)):
            #if self.openList[i].estimatedPathLenght < lowestPath or i == 0:
                #lowestPath = self.openList[i].estimatedPathLenght
                #currentcellIndex = i
        currentcell = min(self.openList, key=lambda x: x.estimatedPathLenght)
        currentcellIndex = self.openList.index(currentcell)

        # Remove it from open list and add it to closed list
        self.closedList.append(self.openList.pop(currentcellIndex))

        # Make it the curent cell
        curentCell = self.closedList[-1]

        if curentCell == endCell:
            self.stop = True
            # Color path
            self.colorShortestPath(curentCell)
            return False

        # Code to be executed on each adjacent cells
        def adjCellInnerFunc(adjCell):
            # Already on the closed list
            if adjCell in self.closedList:
                return
            # If not in open list, add it
            if adjCell not in self.openList:
                self.openList.append(adjCell)
                # Make the current cell the parent of this one
                adjCell.parentCell = curentCell
                adjCell.startToThisLengh = curentCell.startToThisLengh + self.heuristic(curentCell, adjCell)
                adjCell.heuristicLenght = self.heuristic(adjCell, endCell)
                adjCell.estimatedPathLenght = adjCell.startToThisLengh + adjCell.heuristicLenght
            else:
                # Already in the open list, check if this path is better
                if adjCell.startToThisLengh > (curentCell.startToThisLengh + 1):
                    # Change the parent of this cell to the current cell
                    adjCell.parentCell = curentCell
                    adjCell.startToThisLengh = curentCell.startToThisLengh + 1
                    adjCell.heuristicLenght = self.heuristic(adjCell, endCell)
                    adjCell.estimatedPathLenght = adjCell.startToThisLengh + adjCell.heuristicLenght

        # Loop through each adjacent cell
        self.loopThroughAllWalkableNeighbours(curentCell, grid, adjCellInnerFunc)

        # Color
        if not drawJustPath:
            for cell in self.openList:
                cell.pathFindingDrawColor = 'orange'
            for cell in self.closedList:
                cell.pathFindingDrawColor = 'brown'
            curentCell.pathFindingDrawColor = 'red'

        return True



class pathFindingGame:
    def __init__(self):
        # Create graphical window
        self.window = Tk()
        self.window.title('Path finding')
        self.canvas = Canvas(self.window, width=BOARD_SIZE, height=BOARD_SIZE)
        self.canvas.pack(side=LEFT)

        # Mouse click input
        self.window.bind('<ButtonRelease-1>', self.mouseReleaseHandler)
        self.window.bind('<Button-1>', self.mouseClickHandler)
        self.window.bind('<Motion>', self.mouseMotionHandler)
        self.isLeftClicked = False
        self.movingEnd = False
        self.movingStart = False
        self.mousePos = np.array([0, 0])

        self.fastMode = False

        #T = Text(self.window, height=2, width=30, background='grey')
        #T.pack()
        #T.insert(END, "Just a text Widget\nin two lines\n")

        # Start pathfind button
        self.startButton = Button(self.window, text="Start/reset", command=self.startPathFind)
        self.startButton.pack()
        self.startFind = True
        self.clearWallsButton = Button(self.window, text="Clear walls", command=self.clearWalls)
        self.clearWallsButton.pack()
        self.fastModeButton = Button(self.window, text="Enable fast mode", command=self.toogleFastMode)
        self.fastModeButton.pack()
        self.switchAlgoButton = Button(self.window, text="Astar is selected. Switch to Dijkstras", command=self.toogleAlgo)
        self.switchAlgoButton.pack()
        self.isAstar = True
        self.forbidDiagonalButton = Button(self.window, text="Forbid diagonal", command=self.toogleDiagonal)
        self.forbidDiagonalButton.pack()
        self.areDiagonalForbidden = False

        # Exit window handling
        self.window.protocol("WM_DELETE_WINDOW", self.onClosing)
        self.run = True

        # Create grid game and initialize it
        #create_cell = np.vectorize(lambda x: Cell())
        #self.grid = create_cell(np.empty((GRID_RES, GRID_RES)))
        self.grid = np.empty((GRID_RES, GRID_RES), dtype=Cell)
        for i in range(GRID_RES):
            for j in range(GRID_RES):
                self.grid[i][j] = Cell(i, j)

        # Init start and end cell
        self.startCell = self.grid[START_X][START_Y]
        self.endCell = self.grid[END_X][END_Y]

        # Init scene
        for i in range(GRID_RES):
            if i != 20:
                self.grid[int(GRID_RES/2)][i].type = CellType.WALL
            if i < GRID_RES*0.6:
                self.grid[int(GRID_RES / 4)][i].type = CellType.WALL

        # Pathfind algorithm instance
        self.pathFinderInstance = Astar(self.startCell, self.grid, self.areDiagonalForbidden)
        #self.pathFinderInstance = Dijkstras(self.startCell, self.grid)


    def toogleDiagonal(self):
        self.areDiagonalForbidden = not self.areDiagonalForbidden

        if self.areDiagonalForbidden:
            self.forbidDiagonalButton['text'] = "Allow diagonal"
        else:
            self.forbidDiagonalButton['text'] = "Forbid diagonal"

        # Restart pathfind
        self.startPathFind()


    def toogleAlgo(self):
        # Toogle algo flag
        self.isAstar = not self.isAstar

        # Stop ongoing pathfind
        self.startFind = False

        # Rename button and Actually do the switch
        if self.isAstar:
            self.switchAlgoButton['text'] = "Astar is selected. Switch to Dijkstras"
            self.pathFinderInstance = Astar(self.startCell, self.grid, self.areDiagonalForbidden)
        else:
            self.switchAlgoButton['text'] = "Dijkstras is selected. Switch to Astar"
            self.pathFinderInstance = Dijkstras(self.startCell, self.grid, self.areDiagonalForbidden)

        # Restart pathfind
        self.startPathFind()


    def toogleFastMode(self):
        # Toogle flag
        self.fastMode = not self.fastMode

        # Rename button
        if self.fastMode:
            self.fastModeButton['text'] = "Enable slow mode"
        else:
            self.fastModeButton['text'] = "Enable fast mode"

        # Restart pathfind
        self.startPathFind()

    def clearWalls(self):
        for x in range(np.size(self.grid, 0)):
            for y in range(np.size(self.grid, 1)):
                if self.grid[x][y].type == CellType.WALL:
                    self.grid[x][y].type = CellType.EMPTY

    def clearPath(self):
        for x in range(np.size(self.grid, 0)):
            for y in range(np.size(self.grid, 1)):
                if self.grid[x][y].pathFindingDrawColor != 'grey':
                    self.grid[x][y].pathFindingDrawColor == 'grey'


    def startPathFind(self):
        self.startFind = True

        # Reset pathfinding algorithm
        #self.astar.reset(self.startCell)
        self.pathFinderInstance.reset(self.startCell, self.grid, self.areDiagonalForbidden)

        # Refresh grid for drawing
        for x in range(np.size(self.grid, 0)):
            for y in range(np.size(self.grid, 1)):
                self.grid[x][y].pathFindingDrawColor = 'grey'


    def draw(self):
        # Clear screen
        self.canvas.delete("all")

        # Draw grid
        for x in range(GRID_RES):
            pX = (BOARD_SIZE / GRID_RES) * x
            self.canvas.create_line(pX, 0, pX, BOARD_SIZE)
        for y in range(GRID_RES):
            pY = (BOARD_SIZE / GRID_RES) * y
            self.canvas.create_line(0, pY, BOARD_SIZE, pY)

        # Draw cells
        cellSize = BOARD_SIZE / GRID_RES
        for x in range(np.size(self.grid, 0)):
            pX = x * cellSize
            for y in range(np.size(self.grid, 1)):
                pY = y * cellSize
                if self.grid[x][y] == self.startCell:
                    self.canvas.create_rectangle(pX, pY, pX + cellSize, pY + cellSize, fill='orange')
                elif self.grid[x][y] == self.endCell:
                    self.canvas.create_rectangle(pX, pY, pX + cellSize, pY + cellSize, fill='green')
                elif self.grid[x][y].type == CellType.WALL:
                    self.canvas.create_rectangle(pX, pY, pX + cellSize, pY + cellSize, fill='black')
                else:
                    self.canvas.create_rectangle(pX, pY, pX + cellSize, pY + cellSize, fill=self.grid[x][y].pathFindingDrawColor)



    def mainLoop(self):
        #self.window.mainloop()
        while self.run:
            # Refresh drawing
            self.draw()

            # Handle GUI
            self.window.update_idletasks()
            self.window.update()

            # Call pathfind algorithm
            if not self.fastMode:
                if self.startFind:
                    for i in range(SPEED):
                        if not self.pathFinderInstance.computeOneStep(self.grid, self.endCell, False):
                            self.startFind = False
                            break
            else:
                self.clearPath()
                while self.pathFinderInstance.computeOneStep(self.grid, self.endCell, True):
                    pass

        # Clean everything
        self.window.destroy()

    def isPosValid(self, pos):
        if pos[0] >= 0 and pos[1] >= 0 and pos[0] < np.size(self.grid, 0) and pos[1] < np.size(self.grid, 1):
            return True
        return False

    # Called when the window is closed
    def onClosing(self):
        self.run = False


    def mouseClickHandler(self, event):
        self.isLeftClicked = True
        if self.isPosValid(self.mousePos):
            if not self.movingEnd and self.grid[int(self.mousePos[0])][int(self.mousePos[1])] == self.endCell:
                self.movingEnd = True
            if not self.movingStart and self.grid[int(self.mousePos[0])][int(self.mousePos[1])] == self.startCell:
                self.movingStart = True


    def mouseReleaseHandler(self, event):
        self.isLeftClicked = False
        if self.isPosValid(self.mousePos):
            if self.movingEnd:
                self.movingEnd = False
                self.endCell = self.grid[int(self.mousePos[0])][int(self.mousePos[1])]
            if self.movingStart:
                self.movingStart = False
                self.startCell = self.grid[int(self.mousePos[0])][int(self.mousePos[1])]


    def mouseMotionHandler(self, event):
        cellSize = BOARD_SIZE / GRID_RES
        self.mousePos = np.array([event.x, event.y]) / cellSize
        if self.isLeftClicked and (not self.movingEnd) and (not self.movingStart):
            if self.isPosValid(self.mousePos):
                self.grid[int(self.mousePos[0])][int(self.mousePos[1])].type = CellType.WALL
        if self.movingStart and self.isPosValid(self.mousePos):
            if self.grid[int(self.mousePos[0])][int(self.mousePos[1])] != self.startCell: # Start cell change
                self.startCell = self.grid[int(self.mousePos[0])][int(self.mousePos[1])]
                if self.fastMode:
                    self.startPathFind()
        if self.movingEnd and self.isPosValid(self.mousePos):
            if self.grid[int(self.mousePos[0])][int(self.mousePos[1])] != self.endCell:  # End cell change
                self.endCell = self.grid[int(self.mousePos[0])][int(self.mousePos[1])]
                if self.fastMode:
                    self.startPathFind()



gameInstance = pathFindingGame()
gameInstance.mainLoop()
