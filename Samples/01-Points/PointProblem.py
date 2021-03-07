from Visualizer import Visualizer
import sys

class Result:
    def __init__(self, axes, resultData, problem):
        for p in range(0,len(axes),1):
            self.showProblem(axes[p], problem)
            self.showTree(axes[p], resultData[p]["trees"][0], 'blue')
            if(len(resultData[p]["trees"]) > 1):
                self.showTree(axes[p], resultData[p]["trees"][1], 'green')
            self.showSolution(axes[p], resultData[p]["solution"])

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
                      
        

class PointProblem(Visualizer):
    def __init__(self, fileName):
        Visualizer.__init__(self, fileName)
        
    def make_result_fig(self, fig, axes, resultData):
        return Result(axes, resultData, self.data["problem"])
                        
vis = PointProblem(sys.argv[1])
vis.show()