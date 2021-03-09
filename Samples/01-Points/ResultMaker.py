import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class Result:
    def __init__(self, fig, ax, problem, result_ij):
        self.showProblem(ax, problem)
        self.showTree(ax, result_ij["trees"][0], 'blue')
        if(len(result_ij["trees"]) > 1):
            self.showTree(ax, result_ij["trees"][1], 'green')
        self.showSolution(fig, ax, result_ij["solution"])

    def showProblem(self, ax, problem):
        for b in problem["obstacles"]:
            ax.broken_barh([(b[0],b[2] - b[0])], (b[1],b[3] - b[1]), facecolors='tab:orange')
        ax.set_xlim(problem["limits"][0], problem["limits"][2])
        ax.set_ylim(problem["limits"][1], problem["limits"][3])
        
    def showLine(self, ax, pointA, pointB, color, linewidth = 0.5):
        ax.plot([pointA[0] , pointB[0]], [pointA[1] , pointB[1]], color, linewidth = linewidth)    
            
    def showTree(self, ax, tree, color):
        for edge in tree:
            self.showLine(ax, edge[0:2], edge[2:4], color)

    def showSolution(self, fig, ax, solution):
        self.solution_data = solution
        self.solution_x = []
        self.solution_y = []
        self.solution_line, = ax.plot(self.solution_x, self.solution_y, 'r', linewidth = 2)
        self.animation = FuncAnimation(fig, func=self.updateSolution, frames=range(0 , len(self.solution_data), 1), interval=1, repeat=True)

    def updateSolution(self, index):
        if(index == 0):
            self.solution_x = []
            self.solution_y = []
        else:
            self.solution_x.append(self.solution_data[index][0])
            self.solution_y.append(self.solution_data[index][1])
        self.solution_line.set_xdata(self.solution_x)
        self.solution_line.set_ydata(self.solution_y)
          
def make_result(fig, ax, problem, result_ij):
    return Result(fig, ax, problem, result_ij)