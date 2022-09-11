import json
import matplotlib.pyplot as plt
import matplotlib.path as mpath
import matplotlib.patches as mpatches
import numpy as np
import sys
import os

def import_json(file_name):
    f = open(file_name)
    data = json.load(f)
    f.close()
    return data

def to_rad(angle):
    return angle * np.pi / 180.0

class AxisBox:
    def __init__(self):
        self.range=[0,0]

    def extend(self, val):
        if(self.range[1] < val):
            self.range[1] = val
        if(self.range[0] > val):
            self.range[0] = val

    def get(self):
        delta = 0.1* (self.range[1] - self.range[0])
        return [self.range[0] - delta, self.range[1] + delta]

class BoundingBox:
    def __init__(self):
        self.interval_x=AxisBox()
        self.interval_y=AxisBox()

    def extend(self, point):
        self.interval_x.extend(point[0])
        self.interval_y.extend(point[1])

    def printCorners(self, ax):
        range_x = self.interval_x.get()
        range_y = self.interval_y.get()
        ax.plot([range_x[0]],[range_y[0]], 'r')
        ax.plot([range_x[1]],[range_y[1]], 'r')

limits = BoundingBox()

def print_patch(ax, samples, facecolor, alpha=1.0, edgecolor=None):
    if edgecolor == None:
        edgecolor = facecolor
    path = mpath.Path(samples)
    for sample in samples:
        limits.extend(sample)
    patch = mpatches.PathPatch(path, facecolor=facecolor, edgecolor=edgecolor, alpha=alpha)
    return ax.add_patch(patch)

def print_sphere(ax, sphere):
    samples = []
    for angle in np.arange(0.0, 2 * np.pi, to_rad(5)):
        x = sphere["center"][0] + sphere["ray"] * np.cos(angle)
        y = sphere["center"][1] + sphere["ray"] * np.sin(angle)
        samples.append([x, y])
    return print_patch(ax, samples, 'r')

def print_cart(ax, cart, state):
    w = 0.5 * cart["width"]
    l = 0.5 * cart["length"]
    points = []
    points.append([ l, w])
    points.append([-l, w])
    points.append([-l,-w])
    points.append([ l,-w])

    points_trsf = []
    cos_angle = np.cos(state[2])
    sin_angle = np.sin(state[2])
    for point in points:
        x = state[0] + cos_angle * point[0] - sin_angle * point[1]
        y = state[1] + sin_angle * point[0] + cos_angle * point[1]
        points_trsf.append([x,y])
    print_patch(ax, points_trsf, 'cyan')

def print_case(log):
    fig, ax = plt.subplots()
    if log['collides']:
        plt.title("collides")
    else:
        plt.title("no collision")

    print_sphere(ax, log["sphere"])
    print_cart(ax, log["cart"], log["state"])
    limits.printCorners(ax)

    ax.set_aspect('equal', adjustable='box')

def import_and_print(file_name):
    print("reading from " + file_name)
    data = import_json(file_name)
    print_case(data)

def gather_cases(folder):
    cases = []
    for files in os.scandir(folder):
        if files.path.endswith('json'):
            cases.append(files.path)
    return cases

if len(sys.argv) > 1:
    import_and_print(sys.argv[1])
else:
    for case in gather_cases('./'):
        import_and_print(case)

plt.show()
