from Visualizer import Visualizer
import sys

class PointProblem(Visualizer):
    def __init__(self, fileName):
        Visualizer.__init__(self, fileName)
        
    def showProblem(self, ax):
        for b in self.Problem["obstacles"]:
            ax.broken_barh([(b[0],b[2] - b[0])], (b[1],b[3] - b[1]), facecolors='tab:orange')
        ax.set_xlim(self.Problem["limits"][0], self.Problem["limits"][2])
        ax.set_ylim(self.Problem["limits"][1], self.Problem["limits"][3])
        
    def showLine(self, ax, pointA, pointB, color, linewidth = 0.5):
        ax.plot([pointA[0] , pointB[0]], [pointA[1] , pointB[1]], color, linewidth = linewidth)    
    
    def showSolution(self, ax, solution):
        for k in range(1,len(solution),1):
            self.showLine(ax, solution[k-1], solution[k], 'r', 2)
            
    def showTrees(self, ax, trees):
        self.showTree(ax, trees[0], 'b')
        if(len(trees) > 1):
            self.showTree(ax, trees[1], 'g')
            
    def showTree(self, ax, tree, color):
        for edge in tree:
            self.showLine(ax, edge[0:2], edge[2:4], color)
                        


vis = PointProblem(sys.argv[1])
vis.show()