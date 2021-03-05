from Parser import Parser
import matplotlib.pyplot as plt
import sys

class Visualizer:
    def __init__(self, fileName):
        self.log = Parser(fileName)
        
    def show(self):
        fig, ax = plt.subplots()
        ax.set_xlim(-0.1, 1.1)
        ax.set_ylim(-0.1, 1.1)
        self.showBoxes(ax)
        self.showTrees(ax)
        self.showSolution(ax)
        plt.show()
        
    def showBoxes(self, ax):
        for b in self.log.data["boxes"]:
            ax.broken_barh([(b[0],b[2] - b[0])], (b[1],b[3] - b[1]), facecolors='tab:orange')
            
    def showSolution(self, ax):
        for k in range(1,len(self.log.data["solution"]),1):
            self.showLine(ax, self.log.data["solution"][k-1], self.log.data["solution"][k], 'r', 2)
            
    def showTrees(self, ax):
        self.showTree(ax, self.log.data["trees"][0], 'b')
        if(len(self.log.data["trees"]) > 1):
            self.showTree(ax, self.log.data["trees"][1], 'g')
            
    def showTree(self, ax, tree, color):
        for e in tree:
            self.showLine(ax, e[0:2], e[2:4], color)
                        
    def showLine(self, ax, pointA, pointB, color, linewidth = 0.5):
        ax.plot([pointA[0] , pointB[0]], [pointA[1] , pointB[1]], color, linewidth = linewidth)

vis = Visualizer(sys.argv[1])
vis.show()