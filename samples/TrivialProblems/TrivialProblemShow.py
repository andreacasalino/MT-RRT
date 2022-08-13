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

class Printer:
    def __init__(self, log):
        self.scene = log["scene"]

    def printScene(self, ax):
        print_limits(ax, self.scene["limits"])
        for box in self.scene["boxes"]:
            print_box(ax, box)

    def printTree(self, ax, tree, color):
        ax.plot(tree[0]["start"][0], tree[0]["start"][1], c=color)
        for edge in tree:
            print_edge(ax, edge, color)

    def printSolutions(self, ax, solutions):
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

    def finalize(self, ax):
        return