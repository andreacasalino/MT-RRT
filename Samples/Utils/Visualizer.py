import json
import matplotlib.pyplot as plt

class Visualizer:
    def __init__(self, fileName):
        self.data = None
        self.figs = []
        self.axes = []
        self.resultFig = []
        with open(fileName) as json_file:
            self.data = json.load(json_file)
            
    def show(self):
        for mtStrt in self.data["results"]:
            fig, ax = plt.subplots(nrows=1, ncols=len(self.data["results"][mtStrt]))
            fig.suptitle(mtStrt)
            resultData = []
            axx = []
            if(len(self.data["results"][mtStrt]) == 1):
                rrtStrt = list(self.data["results"][mtStrt].keys())[0]
                ax.set_title(rrtStrt)
                resultData += [self.data["results"][mtStrt][rrtStrt]]
                axx += [ax]
            else:    
                a = 0
                for rrtStrt in self.data["results"][mtStrt]:
                    ax[a].set_title(rrtStrt)
                    resultData += [self.data["results"][mtStrt][rrtStrt]]
                    axx += [ax[a]]
                    a = a+1
            self.resultFig += [self.make_result_fig(fig, axx, resultData)]
            self.figs += [fig]
            self.axes += axx
            
### implement in ancestor ###
    # def make_result_fig(self, fig, axes, resultData)