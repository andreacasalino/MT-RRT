import json
import matplotlib.pyplot as plt

class Visualizer:
    def __init__(self, fileName):
        self.Problem = None
        self.Results = None
        with open(fileName) as json_file:
            data = json.load(json_file)
            self.Problem = data["problem"]
            self.Results = data["results"]
            
    def show(self):
        for mtStrt in self.Results:
            fig, ax = plt.subplots(nrows=1, ncols=len(self.Results[mtStrt]))
            if(len(self.Results[mtStrt]) == 1):
                rrtStrt = list(self.Results[mtStrt].keys())[0]
                ax.set_title(rrtStrt)
                self.showProblem(ax)
                self.showSolution(ax, self.Results[mtStrt][rrtStrt]["solution"])
                self.showTrees(ax, self.Results[mtStrt][rrtStrt]["trees"])
            else:    
                a = 0
                for rrtStrt in self.Results[mtStrt]:
                    ax[a].set_title(rrtStrt)
                    self.showProblem(ax[a])
                    self.showSolution(ax[a], self.Results[mtStrt][rrtStrt]["solution"])
                    self.showTrees(ax[a], self.Results[mtStrt][rrtStrt]["trees"])
                    a = a+1
            fig.suptitle(mtStrt)
            plt.show()
            
    def showTrees(self, ax, trees):
        self.showTree(ax, trees[0], 'b')
        if(len(trees) > 1):
            self.showTree(ax, trees[1], 'g')

### implement in ancestor ###
    # def showProblem(self, ax):
        # return    
    # def showSolution(self, ax, solution):
        # return
    # def showTree(self, ax, tree, color):
        # return