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

def print_case(log):
    if(len(log["segment"]["start"]) != 2):
        return

    fig, ax = plt.subplots()
    if log['collides']:
        plt.title("collides")
    else:
        plt.title("no collision")

    segment_x = [log["segment"]["start"][0], log["segment"]["end"][0]]
    segment_y = [log["segment"]["start"][1], log["segment"]["end"][1]] 
    ax.plot(segment_x, segment_y, "b")
    ax.plot(segment_x, segment_y, "bo")

    box_x = [log["box"]["start"][0], log["box"]["end"][0], log["box"]["end"][0], log["box"]["start"][0]]
    box_y = [log["box"]["start"][1], log["box"]["start"][1], log["box"]["end"][1], log["box"]["end"][1]]
    path = mpath.Path([[box_x[0], box_y[0]],[box_x[1], box_y[1]],[box_x[2], box_y[2]],[box_x[3], box_y[3]]])
    patch = mpatches.PathPatch(path, facecolor='r', alpha=0.5)
    ax.add_patch(patch)
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
