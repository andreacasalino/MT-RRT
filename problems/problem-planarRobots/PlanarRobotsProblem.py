from common.Figure import Figure as FigureBase
from common.Figure import show
from common.Transform import Trasform
from common.Colors import make_colors
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.path as mpath
import matplotlib.patches as mpatches
from numpy import linspace, arange
import math

def make_capsule(ax, ray, len, trsf, color, alpha=None):
    points = []
    for angle in linspace(- 0.5 * math.pi, 0.5 * math.pi, 10):
        x = len + math.cos(angle) * ray
        y = math.sin(angle) * ray
        points.append([x, y])
    for angle in linspace(0.5 * math.pi, 3.0 * math.pi / 2.0, 10):
        x = math.cos(angle) * ray
        y = math.sin(angle) * ray
        points.append([x, y])
    points.append([0, -ray])
    points.append([len, -ray])
    points = trsf.applyAll(points)
    path = mpath.Path(points)
    patch = mpatches.PathPatch(path, facecolor=color, alpha=1.0 if alpha == None else alpha)
    ax.add_patch(patch)
    return patch

class Robot:
    def __init__(self, ax, dataJson, color):
        self.ax = ax
        self.base = {'center':dataJson['base']['center'], 'angle':dataJson['base']['angle']} 
        self.joints = [{'ray':joint['ray'],'length':joint['length']} for joint in dataJson['joints']]
        self.color = color
        self.body = []
        self.endEffector = None

    def makeBodyChain_(self, q, color = None, alpha = None):
        if not len(q) == len(self.joints):
            raise Exception('Invalid pose')
        caps = []
        point = [p for p in self.base['center']]
        angle = self.base['angle']
        for q_angle, joint in zip(q, self.joints):
            angle += q_angle
            cap = make_capsule(self.ax, joint['ray'], joint['length'], Trasform(angle, point), self.color if color == None else color, alpha=alpha)
            caps.append(cap)
            point[0] += math.cos(angle) * joint['length']
            point[1] += math.sin(angle) * joint['length']
        return (caps, point)

    def setPose(self, q):
        for cap in self.body:
            cap.remove()
        self.body, self.endEffector = self.makeBodyChain_(q)

    def printPose(self, q, color = None, alpha = None):
        self.makeBodyChain_(q, color=color, alpha=0.2 if alpha == None else alpha)

def forEachPose(q, robots, lam):
    pos = 0
    for robot in robots:
        dof = len(robot.joints)
        lam(robot, q[pos:pos+dof])
        pos += dof

# object like version of https://docs.python.org/3.3/library/functions.html#zip
class Zip:
    def __init__(self):
        self.iterators = []

    def add(self, iterable):
        self.iterators.append(iter(iterable))

    def forEach(self):
        sentinel = object()
        while self.iterators:
            result = []
            for it in self.iterators:
                elem = next(it, sentinel)
                if elem is sentinel:
                    return
                result.append(elem)
            yield result

class QTrajectory:
    MAX_ADVANCMENT = 5.0 * math.pi / 180.0

    @staticmethod
    def interpolate_(front, back):
        N = 1
        for f, b in zip(front, back):
            delta_abs = abs(f - b)
            N = max(N, delta_abs / QTrajectory.MAX_ADVANCMENT)
        N = round(N)
        intervals = Zip()
        for f, b in zip(front, back):
            intervals.add(linspace(f, b, N + 1))
        return [q for q in intervals.forEach()]

    def __init__(self, waypoints):
        self.points = [waypoints[0]]
        for index in range(1, len(waypoints)):
            for to_add in QTrajectory.interpolate_(waypoints[index-1], waypoints[index])[1:]:
                self.points.append(to_add)
        self.index = 0

class EndEffectorTrajectory:
    def __init__(self, ax):
        self.ax = ax
        self.line = None
        self.reset()

    def append(self, point):
        self.x.append(point[0])
        self.y.append(point[1])
        if not self.line == None:
            self.line.remove()
        self.line = self.ax.plot(self.x, self.y, '--', color='b')[0]

    def reset(self):
        if not self.line == None:
            self.line.remove()
        self.line = None
        self.x = []
        self.y = []

class Animation:
    def __init__(self, fig, ax, robots, waypoints):
        self.robots = robots
        self.qTraj = QTrajectory(waypoints)
        self.eeTraj = [EndEffectorTrajectory(ax) for _ in robots]
        self.endEffectorsLines = []
        self.animation = animation.FuncAnimation(fig, self.update_, init_func=self.reset_, frames=self.qTraj.points, interval=250, blit=True)

    def reset_(self):
        for ee in self.eeTraj:
            ee.reset()
        return ()

    def update_(self, q):
        pos = 0
        for robot, eeTraj in zip(self.robots, self.eeTraj):
            dof = len(robot.joints)
            robot.setPose(q[pos:pos+dof])
            eeTraj.append(robot.endEffector)
            pos += dof
        return ()

def extractTreeIndices(tree_size):
    max_size = 20
    step = 1
    if tree_size > max_size:
        step = math.floor(float(tree_size) / float(max_size))
        step = int(step)
    return range(0, tree_size, step)

class Figure(FigureBase):
    ROBOTS_COLORS = ['#1ee9b2', '#a359ee']

    def __init__(self, title, json):
        FigureBase.__init__(self, title, json)
        self.robots = [Robot(self.ax, dataJson, color) for dataJson, color in zip(json['scene']['robots'], make_colors(len(json['scene']['robots']), Figure.ROBOTS_COLORS))]

    def printPose_(self, q, alpha, color=None):
        forEachPose(q, self.robots, lambda robot, q: robot.printPose(q, color=color, alpha=0.2))

    def printScene(self):
        for point in ['start', 'end']:
            if point in self.json['scene']:
                self.printPose_(self.json['scene'][point], 0.5)

    def printTree(self, tree, color):
        for index in extractTreeIndices(len(tree)):
            edge = tree[index]
            self.printPose_(edge['state'], 0.2, color)

    def printSolutions(self):   
        if not len(self.json['solutions']) == 0:
            best = self.json['solutions'][0]['sequence']
            if len(best) == 0:
                return
            self.animation = Animation(self.fig, self.ax, self.robots, best)

if __name__ == '__main__':
    show(Figure)
