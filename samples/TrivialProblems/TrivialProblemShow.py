import math

def print_limits(ax):
    limits = {}
    limits["start"]=[-1,-1]
    limits["end"]=[1,1]
    limits_x = [limits["start"][0], limits["end"][0], limits["end"][0], limits["start"][0], limits["start"][0]]
    limits_y = [limits["start"][1], limits["start"][1], limits["end"][1], limits["end"][1], limits["start"][1]]
    ax.plot(limits_x, limits_y, 'r--')

def print_edge(ax, edge, color):
    segment_x = [edge["start"][0], edge["end"][0]]
    segment_y = [edge["start"][1], edge["end"][1]]
    ax.plot(segment_x, segment_y, c=color, linewidth=0.5)

def combine(t1, t2):
    result = Trasform(0, [0,0])
    result.cos_angle = t1.cos_angle*t2.cos_angle - t1.sin_angle*t2.sin_angle
    result.sin_angle = t1.sin_angle*t2.cos_angle + t1.cos_angle*t2.sin_angle
    result.traslation = t1.apply(t2.traslation)
    return result

def rotation_around_center(angle, center):
    result = Trasform(angle, [0,0])
    x =  center[0] * (1.0 - result.cos_angle) + center[1] * result.sin_angle
    y = -center[0] * result.sin_angle + center[1] * (1.0 - result.cos_angle)
    result.traslation = [x,y]
    return result

class Trasform:
    def __init__(self,angle, trasl):        
        self.cos_angle = np.cos(angle)
        self.sin_angle = np.sin(angle)
        self.traslation = trasl

    def apply(self, point):
        x = self.cos_angle*point[0] - self.sin_angle*point[1] + self.traslation[0]
        y = self.sin_angle*point[0] + self.cos_angle*point[1] + self.traslation[1]
        return [x,y]

    def copy(self, o):
        self.cos_angle = o.cos_angle
        self.sin_angle = o.sin_angle
        self.traslation = [o.traslation[0], o.traslation[1]]

def apply(points, trasform):
    result = []
    for point in points:
        result.append(trasform.apply(point))
    return result

class Box:
    def __init__(self, box_json, boxes_labeled):
        self.trsf = Trasform(0, [0,0])
        if "box" in box_json:
            self.min = box_json["box"]["min"]
            self.max = box_json["box"]["max"] 
        elif "copy" in box_json:
            box_to_copy = boxes_labeled[box_json["copy"]["source"]]
            self.min = box_to_copy.min
            self.max = box_to_copy.max 
            self.trsf.copy(box_to_copy.trsf)

        if "transform" in box_json:
            traslation_json = box_json["transform"]

            angle = 0
            if "angle" in traslation_json:
                angle = traslation_json["angle"] * np.pi / 180.0

            traslation = [0,0]
            if "traslation" in traslation_json:
                traslation = traslation_json["traslation"]

            if "center" in traslation_json:
                temp = combine(Trasform(0, traslation), rotation_around_center(angle, traslation_json["center"]))
                self.trsf = combine(temp, self.trsf)
            else:
                self.trsf = combine(Trasform(angle, traslation), self.trsf)

    def print_box(self, ax):
        points = [self.min, [self.max[0],self.min[1]], self.max, [self.min[0],self.max[1]]]
        points = apply(points, self.trsf)
        path = mpath.Path(points)
        patch = mpatches.PathPatch(path, facecolor='r', alpha=0.5)
        ax.add_patch(patch)

class Printer:
    def __init__(self, log):
        self.scene = log["scene"]

    def printScene(self, ax):
        print_limits(ax)
        labeled={}
        for box_json in self.scene:
            box = Box(box_json, labeled)
            box.print_box(ax)
            if "label" in box_json:
                labeled[box_json["label"]] = box

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