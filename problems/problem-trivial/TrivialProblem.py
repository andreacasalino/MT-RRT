from common.Figure import Figure as FigureBase
from common.Figure import show
from common.Geometry import print_edge
from numpy import linspace

class Figure(FigureBase):
    def printScene(self):
        for point, color in zip(['start', 'end'], ['red', 'green']):
            if point in self.json['scene']:
                coordinates =self.json['scene'][point]
                self.ax.plot(coordinates[0], coordinates[1], marker='8', c=color)

    def printTree(self, tree, color):
        for edge in tree:
            print_edge(self.ax, edge['from'], edge['state'], color, width=0.15)

    def printSolutions(self):        
        if len(self.json['solutions']) == 0:
            return
        for (w, solution) in zip(linspace(0.1, 2.0, len(self.json['solutions'])), self.json['solutions']):
            x = [state[0] for state in solution['sequence']]
            y = [state[1] for state in solution['sequence']]
            width = 2.0 - w + 0.1
            self.ax.plot(x, y, c='red', linewidth= width)

if __name__ == '__main__':
    show(Figure)
