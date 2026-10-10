import matplotlib.pyplot as plt
import os, json
from common.Geometry import *
from common.Colors import make_colors
import sys

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
        labels = {
            'place_holders':[],
            'desc':[]
        }
        for tree, color, index in zip( self.json['trees'], colors, range(0, len(self.json['trees'])) ):
            self.printTree(tree, color)
            labels['place_holders'].append(self.ax.plot([0], [0], c=color)[0])
            labels['desc'].append('States from tree {}'.format(index + 1))
        self.ax.legend(labels['place_holders'], labels['desc'])
        self.printSolutions()

    ########################################################
    ########################################################
            
    def printScene(self):
        return

    def printTree(self, tree, color):
        return

    def printSolutions(self):
        return

def files_gen():
    log_folder = os.environ['MT_RRT_LOG_PATH']
    if 1 == len(sys.argv):
        for filename in os.listdir(log_folder):
            filename_abs = os.path.join(log_folder, filename)
            yield filename, filename_abs
    else:
        filename = str(sys.argv[1])
        filename_abs = os.path.join(log_folder, filename)
        if not os.path.exists(filename_abs):
            raise Exception(f"Unable to open file named: '{filename}'")
        yield filename, filename_abs

def show(FigureT):
    if 0 < len(sys.argv):
        pass

    figures = []
    for filename, filename_abs in files_gen():
        with open(filename_abs, 'r') as stream:
            fig = FigureT(filename, json.load(stream))
            fig.show()
            figures.append(fig)  
    plt.autoscale()
    plt.show()

if __name__ == '__main__':
    show(Figure)
