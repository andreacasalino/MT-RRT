import json
import matplotlib.pyplot as plt
import matplotlib.path as mpath
import matplotlib.patches as mpatches
import numpy as np
import random
import sys
import os

def import_json(file_name):
    f = open(file_name)
    data = json.load(f)
    f.close()
    return data

def print_limits(ax, limits):
    limits_x = [limits["start"][0], limits["end"][0], limits["end"][0], limits["start"][0], limits["start"][0]]
    limits_y = [limits["start"][1], limits["start"][1], limits["end"][1], limits["end"][1], limits["start"][1]]
    ax.plot(limits_x, limits_y, 'r--')

def print_box(ax, box):
    box_x = [box["start"][0], box["end"][0], box["end"][0], box["start"][0]]
    box_y = [box["start"][1], box["start"][1], box["end"][1], box["end"][1]]
    path = mpath.Path([[box_x[0], box_y[0]],[box_x[1], box_y[1]],[box_x[2], box_y[2]],[box_x[3], box_y[3]]])
    patch = mpatches.PathPatch(path, facecolor='r', alpha=0.5)
    ax.add_patch(patch)

def print_edge(ax, edge, color):
    segment_x = [edge["start"][0], edge["end"][0]]
    segment_y = [edge["start"][1], edge["end"][1]]
    ax.plot(segment_x, segment_y, c=color, linewidth=0.5)

def print_solutions(ax, solutions):
    if(len(solutions) == 0):
        return
    w = 3.0
    w_decrement = w / len(solutions)
    for solution in solutions:
        sequence = solution["sequence"]
        sequence_x = []
        sequence_y = []
        for state in sequence:
            sequence_x.append(state[0])
            sequence_y.append(state[1])
        ax.plot(sequence_x, sequence_y, c='r', linewidth=w)
        w -= w_decrement

def make_trees_colors(trees_numb):
    result = []
    result.append([0,0,1])
    result.append([0,1,0])
    for i in range(2, trees_numb, 1):
        result.append([random.random(), random.random(), random.random()])
    return result

def print_case(log):
    fig, ax = plt.subplots()

    if "scene" in log:
        print_limits(ax, log["scene"]["limits"])

        for box in log["scene"]["boxes"]:
            print_box(ax, box)

    if "trees" in log:
        trees_colors = make_trees_colors(len(log["trees"]))
        tree_counter = 0
        for tree in log["trees"]:
            ax.plot(tree[0]["start"][0], tree[0]["start"][1], c=trees_colors[tree_counter])
            for edge in tree:
                print_edge(ax, edge, trees_colors[tree_counter])
            tree_counter += 1

    if "solutions" in log:
        print_solutions(ax, log["solutions"])

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
