from common.Figure import Figure as FigureBase
from common.Figure import show
import matplotlib.pyplot as plt
import math
from numpy import linspace

class Figure(FigureBase):
    def __init__(self, title, json):
        FigureBase.__init__(self, title, json)

    def printPose_(self, pose, color, remark=False):
        if remark:
            self.ax.plot(pose[0], pose[1], marker='8', c=color)
        arrow = self.ax.quiver(pose[0], pose[1], math.cos(pose[2]), math.sin(pose[2]))

    def show(self):
        self.printPose_(self.json['start'], 'red', True)
        self.printPose_(self.json['end'], 'red', True)

class FigureInfo(Figure):
    def printSequence(self, points, remark=False):
        x = [point[0] for point in points]
        y = [point[1] for point in points]
        if remark:
            self.ax.plot(x, y, c='black', linewidth=0.3, linestyle='dashed', marker='8')
        else:
            self.ax.plot(x, y, c='black', linewidth=0.3, linestyle='dashed')

    def printComplex_(self):
        skeleton = [self.json['start'][:2], self.json['arc_begin'], self.json['center'], self.json['arc_end'], self.json['end'][:2]]
        self.printSequence(skeleton, True)
        circle = []
        for angle in linspace(0, 2*math.pi, num=25):
            circle.append([self.json['center'][0] + self.json['ray']*math.cos(angle), self.json['center'][1] + self.json['ray']*math.sin(angle)])
        self.printSequence(circle)

    def printTrivial_(self):
        self.printSequence([self.json['start'][:2], self.json['end'][:2]])

    def show(self):
        Figure.show(self)
        if self.json['type'] == 'trivial':
            self.printTrivial_()
        else:
            self.printComplex_()
        self.ax.set_aspect('equal', adjustable='box')

class FigureTrajectory(Figure):
    def show(self):
        Figure.show(self)
        x = []
        y = []
        for pose in self.json['sequence']:
            self.printPose_(pose, color='black')
            x.append(pose[0])
            y.append(pose[1])
        self.ax.plot(x, y, c='black', linewidth=0.3)
        self.ax.set_aspect('equal', adjustable='box')

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--kind', dest='kind')
    args = parser.parse_args()

    FigureT = FigureInfo if args.kind == 'info' else FigureTrajectory
    show(FigureT)
