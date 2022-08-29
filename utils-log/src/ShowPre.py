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

def make_trees_colors(trees_numb):
    result = []
    result.append([0,0,1])
    result.append([0,1,0])
    for i in range(2, trees_numb, 1):
        result.append([random.random(), random.random(), random.random()])
    return result

printers = []

def print_case(log):
    fig, ax = plt.subplots()
    printer = Printer(log)

    printer.printScene(ax, fig)
    
    if "trees" in log:
        trees_colors = make_trees_colors(len(log["trees"]))
        tree_counter = 0
        for tree in log["trees"]:
            printer.printTree(ax, fig, tree, trees_colors[tree_counter])
            tree_counter += 1

    if "solutions" in log:
        printer.printSolutions(ax, fig, log["solutions"])

    printer.finalize(ax, fig)

    ax.set_aspect('equal', adjustable='box')
    
    printers.append(printer)

#####################################################################
##                    problem specific code                        ##
# class Printer:
#     def __init__(self, scene):
#         # bla bla
#
#     def printScene(self, ax, fig):
#
#     def printTree(self, ax, fig, tree, color):
#
#     def printSolutions(self, ax, fig, solutions):
#
#     def finalize(self, ax, fig):
