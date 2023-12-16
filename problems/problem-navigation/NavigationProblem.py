from common.Figure import Figure as FigureBase
from common.Figure import show
from common.Transform import Trasform
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.path as mpath
import matplotlib.patches as mpatches
import math

class Cart:
    def __init__(self, ax, dataJson):
        self.ax = ax
        w = dataJson['width']*0.5
        l = dataJson['length']*0.5
        self.shape = [[l,w], [-l,w], [-l,-w], [l,-w]]
        self.shape.append(self.shape[0])
        self.body = []
        self.color = '#9939fa'

    def printBody_(self, pose, color = None, alpha = None):
        trsf = Trasform(pose[2], pose[:2])
        shape_trsf = trsf.applyAll(self.shape)
        path = mpath.Path(shape_trsf)
        patch_body = mpatches.PathPatch(path, facecolor=color, alpha=0.5)
        self.ax.add_patch(patch_body)
        arrow = self.ax.quiver(pose[0], pose[1], math.cos(pose[2]), math.sin(pose[2]))
        return [patch_body, arrow]

    def setPose(self, pose):
        for b in self.body:
            b.remove()
        self.body = self.printBody_(pose, color=self.color, alpha=1.0)

    def printPose(self, pose, color = None, alpha = None):
        self.printBody_(pose, color= self.color if color == None else color, alpha=1.0 if alpha == None else alpha)

class Animation:
    def __init__(self, fig, ax, cart, waypoints):
        self.ax = ax
        self.cart = cart        
        self.lastWaypoint = None
        self.pathSoFar = []
        self.waypoints = waypoints
        self.animation = animation.FuncAnimation(fig, self.update_, init_func=self.reset_, frames=self.waypoints, interval=250, blit=True)

    def reset_(self):     
        self.lastWaypoint = None
        for p in self.pathSoFar:
            p.remove()
        self.pathSoFar = []
        return ()

    def update_(self, pose):
        self.cart.setPose(pose)
        if not self.lastWaypoint == None:
            line = self.ax.plot([self.lastWaypoint[0], pose[0]], [self.lastWaypoint[1], pose[1]], c='red', linewidth=2.0)[0]
            self.pathSoFar.append(line)
        self.lastWaypoint = pose
        return ()

def extractTreeIndices(tree_size):
    max_size = 35
    step = 1
    if tree_size > max_size:
        step = math.floor(float(tree_size) / float(max_size))
        step = int(step)
    return range(0, tree_size, step)

class Figure(FigureBase):
    def __init__(self, title, json):
        FigureBase.__init__(self, title, json)
        self.cart = Cart(self.ax, json['scene']['cart'])

    def printScene(self):
        for point in ['start', 'end']:
            if point in self.json['scene']:
                self.cart.printPose(self.json['scene'][point], alpha=0.5)

    def printTree(self, tree, color):
        for index in extractTreeIndices(len(tree)):
            edge = tree[index]
            self.cart.printPose(edge['state'], alpha=0.05, color=color)

    def printSolutions(self):    
        if not len(self.json['solutions']) == 0:
            best = self.json['solutions'][0]['sequence']
            if len(best) == 0:
                return
            self.animation = Animation(self.fig, self.ax, self.cart, best)

class FigureSimple(FigureBase):
    def __init__(self, title, json):
        FigureBase.__init__(self, title, json)
        self.cart = Cart(self.ax, json['cart'])

    def show(self): 
        for obstacle in self.json['obstacles']:
            self.printObstacle(obstacle)
        for pose in self.json['poses']:
            self.cart.printPose(pose, alpha=1.0)
        self.ax.set_aspect('equal', adjustable='box')

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--simplified', dest='simplified', action='store_true', default=False)
    args = parser.parse_args()

    if args.simplified:
        show(FigureSimple)
    else:
        show(Figure)
