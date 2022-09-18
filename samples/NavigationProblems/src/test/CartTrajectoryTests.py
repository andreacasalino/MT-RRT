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

def print_state(ax, state, start_end, scale=20):
    vx = np.cos(state[2])
    vy = np.sin(state[2])
    tail = [state[0], state[1]]
    head = [tail[0] + vx, tail[1] + vy]
    if start_end :
        tail = [tail[0] - vx, tail[1] - vy]
        head = [head[0] - vx, head[1] - vy]
    arrow = mpatches.FancyArrowPatch((tail[0], tail[1]), (head[0], head[1]), mutation_scale=scale)
    ax.add_patch(arrow)
    limits.extend(tail)
    limits.extend(head)

def print_line(ax, start, end, options):
    ax.plot([start[0], end[0]], [start[1], end[1]], options)
    limits.extend(start)
    limits.extend(end)

def print_circle(ax, center, ray):
    samples_x = []
    samples_y = []
    for angle in np.arange(0.0, 2 * np.pi, 5 * np.pi / 180.0):
        x = center[0] + ray * np.cos(angle)
        y = center[1] + ray * np.sin(angle)
        samples_x.append(x)
        samples_y.append(y)
        limits.extend([x,y])
    ax.plot(samples_x, samples_y, "r--")
    
def print_blended(ax, info):
    print_circle(ax, info["center"], info["ray"])
    print_line(ax, info["start"], info["arc_begin"], 'b')
    print_line(ax, info["end"], info["arc_end"], 'b')
    print_line(ax, info["arc_begin"], info["center"], 'r--')
    print_line(ax, info["arc_end"], info["center"], 'r--')
    
def print_sequence(ax, sequence):
    line_x = []
    line_y = []
    for sample in sequence:
        print_state(ax, sample, False, 10)
        line_x.append(sample[0])
        line_y.append(sample[1])
    ax.plot(line_x, line_y, 'r')

def print_case(log):
    fig, ax = plt.subplots()
    
    print_state(ax, log['start'], True)
    print_state(ax, log['end'], False)

    if 'info' in log:
        if log['info']['type'] == "trivial":
            print_line(ax, log["start"], log["end"], 'b')
        if log['info']['type'] == "blended":
            print_blended(ax, log['info'])

    if 'sequence' in log:
        print_sequence(ax, log['sequence'])

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
