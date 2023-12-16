from common.Transform import Trasform
import matplotlib.path as mpath
import matplotlib.patches as mpatches
from numpy import linspace
import math

def print_edge(ax, start, end, color, width=None):
    segment_x = [start[0], end[0]]
    segment_y = [start[1], end[1]]
    ax.plot(segment_x, segment_y, c=color, linewidth=0.5 if None else width)

class Segment:
    def __init__(self, start, end):
        self.start = start
        self.end = end

    @staticmethod
    def fromJson(json):
        return Segment(json['start'], json['end'])

    def print(self, ax):
        print_edge(ax, self.start, self.end, 'b', 1)

class Box:
    def __init__(self, min, max, transform=None):
        self.min = min
        self.max = max
        self.transform = transform

    @staticmethod
    def fromJson(json):
        res = Box(json['min'], json['max'])
        if 'transform' in json:
            res.transform = Trasform.fromJson(json['transform'])
        return res

    def print(self, ax):
        points = [self.min, [self.max[0],self.min[1]], self.max, [self.min[0],self.max[1]]]
        points.append(points[0])
        if not None == self.transform:
            points = self.transform.applyAll(points)
        path = mpath.Path(points)
        patch = mpatches.PathPatch(path, facecolor='r', alpha=0.5)
        ax.add_patch(patch)

class Sphere:
    def __init__(self, center, ray):
        self.center = center
        self.ray = ray

    @staticmethod
    def fromJson(json):
        return Sphere(json['center'], json['ray'])

    def print(self, ax):
        def make_point(center, ray, angle):
            x = center[0] + math.cos(angle) * ray
            y = center[1] + math.sin(angle) * ray
            return [x, y]
        points = [make_point(self.center, self.ray, angle) for angle in linspace(0, 2*math.pi, 40)]
        path = mpath.Path(points)
        patch = mpatches.PathPatch(path, facecolor='r', alpha=0.5)
        ax.add_patch(patch)
