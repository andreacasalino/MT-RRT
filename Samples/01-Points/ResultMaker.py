import matplotlib.pyplot as plt

class Result:
    def __init__(self, ax, problem, result_ij):
        self.showProblem(ax, problem)
        self.showTree(ax, result_ij["trees"][0], 'blue')
        if(len(result_ij["trees"]) > 1):
            self.showTree(ax, result_ij["trees"][1], 'green')
        self.showSolution(ax, result_ij["solution"])

    def showProblem(self, ax, problem):
        for b in problem["obstacles"]:
            ax.broken_barh([(b[0],b[2] - b[0])], (b[1],b[3] - b[1]), facecolors='tab:orange')
        ax.set_xlim(problem["limits"][0], problem["limits"][2])
        ax.set_ylim(problem["limits"][1], problem["limits"][3])
        
    def showLine(self, ax, pointA, pointB, color, linewidth = 0.5):
        ax.plot([pointA[0] , pointB[0]], [pointA[1] , pointB[1]], color, linewidth = linewidth)    
    
    def showSolution(self, ax, solution):
        for k in range(1,len(solution),1):
            self.showLine(ax, solution[k-1], solution[k], 'r', 2)
            
    def showTree(self, ax, tree, color):
        for edge in tree:
            self.showLine(ax, edge[0:2], edge[2:4], color)
          
def make_result(fig, ax, problem, result_ij):
    return Result(ax, problem, result_ij)