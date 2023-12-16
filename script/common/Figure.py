import matplotlib.pyplot as plt
import os, json
from common.Geometry import *
from common.Colors import make_colors

class Figure:
    def __init__(self, title, json):
        self.title = title
        self.fig, self.ax = plt.subplots()
        self.json = json
        self.ax.set_aspect('equal', adjustable='box')
        self.fig.suptitle(title, fontsize=14)

    def printObstacle(self, json):
        cataloge = {
            'Segment':Segment,
            'Box':Box,
            'Sphere':Sphere
        }
        obj_type = cataloge[json['type']]
        obj = obj_type.fromJson(json)
        obj.print(self.ax)

    def show(self):
        self.printScene()
        for obstacle in self.json['scene']['obstacles']:
            self.printObstacle(obstacle)
        colors = make_colors(len(self.json['trees']), ['blue', 'green', '#DAF7A6', '#32c8a3', '#af32c8'])
        for tree, color in zip( self.json['trees'], colors ):
            self.printTree(tree, color)
        self.printSolutions()

    ########################################################
    ########################################################
            
    def printScene(self):
        return

    def printTree(self, tree, color):
        return

    def printSolutions(self):
        return

def show(FigureT):
    figures = []
    for tag in os.listdir(os.environ['MT_RRT_LOG_FOLDER']):
        subfolder = os.path.join(os.environ['MT_RRT_LOG_FOLDER'], tag)
        for res in os.listdir(subfolder):
            filename = os.path.join(subfolder, res)
            with open(filename, 'r') as stream:
                fig = FigureT('{}-{}'.format(tag, os.path.basename(res)), json.load(stream))
                fig.show()
                figures.append(fig)  
    plt.autoscale()
    plt.show()

if __name__ == '__main__':
    show(Figure)
