import sys
import json
import matplotlib.pyplot as plt
from ResultMaker import make_result

class Visualizer:
    def __init__(self, fileName):
        self.figures = []
        self.axes = []
        self.results = []
        with open(fileName) as json_file:
            data = json.load(json_file)
            for mtStrt in data["results"]:
                fig, ax = plt.subplots(nrows=1, ncols=len(data["results"][mtStrt]))
                fig.suptitle(mtStrt)
                if(len(data["results"][mtStrt]) == 1):
                    rrtStrt = list(data["results"][mtStrt].keys())[0]
                    ax.set_title(rrtStrt)
                    self.results.append(make_result(fig, ax, data["problem"], data["results"][mtStrt][rrtStrt]))
                else:    
                    a = 0
                    for rrtStrt in data["results"][mtStrt]:
                        ax[a].set_title(rrtStrt)
                        self.results.append(make_result(fig, ax[a], data["problem"], data["results"][mtStrt][rrtStrt]))
                        a = a+1
                self.figures.append(fig)
                self.axes.append(ax)

visualizer = Visualizer(sys.argv[1])
plt.show()