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
            a = 0
            for rrtStrt in self.Results[mtStrt]:
                ax[a].set_title(rrtStrt)
                self.showProblem(ax[a])
                self.showSolution(ax[a], self.Results[mtStrt][rrtStrt]["solution"])
                self.showTrees(ax[a], self.Results[mtStrt][rrtStrt]["trees"])
                a = a+1
            fig.suptitle(mtStrt)
            plt.show()

### implement in ancestor ###
    # def showProblem(self, ax):
        # return    
    # def showSolution(self, ax, solution):
        # return
    # def showTrees(self, ax, trees):
        # return