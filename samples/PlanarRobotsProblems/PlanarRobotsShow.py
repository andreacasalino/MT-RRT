import math

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

def to_rad(angle):
    return angle * np.pi / 180.0

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

class Transform:
    def __init__(self, traslation, rotation):
        self.delta = traslation
        self.cos_angle = np.cos(rotation)
        self.sin_angle = np.sin(rotation)

    def apply(self, points):
        result = []
        for point in points:
            x = self.delta[0] + self.cos_angle * point[0] - self.sin_angle * point[1]
            y = self.delta[1] + self.sin_angle * point[0] + self.cos_angle * point[1]
            result.append([x,y])
        return result

def print_capsule(ax, link_json, trasform, color, alpha):
    samples = []
    for angle in np.arange(-0.5 * np.pi, 0.5 * np.pi, to_rad(5)):
        x = link_json["ray"] * np.cos(angle) + link_json["length"]
        y = link_json["ray"] * np.sin(angle)
        samples.append([x, y])
    for angle in np.arange(0.5 * np.pi, 3 * np.pi / 2.0, to_rad(5)):
        x = link_json["ray"] * np.cos(angle)
        y = link_json["ray"] * np.sin(angle)
        samples.append([x, y])
    samples.append([link_json["length"], -link_json["ray"]])
    samples = trasform.apply(samples)
    return print_patch(ax, samples, color, alpha, 'k')

def print_pose(ax, robots_json, pose, color, alpha):
    index = 0
    ee = []
    patches = []
    for robot in robots_json:
        angle_abs = 0
        position = [0,0]
        if "base" in robot:
            angle_abs = robot["base"]["angle"]
            angle_abs = to_rad(angle_abs)
            position = [robot["base"]["position"][0], robot["base"]["position"][1]]
        for link in robot["links"]:
            angle_abs += pose[index]
            patch = print_capsule(ax, link, Transform(position, angle_abs), color, alpha)
            patches.append(patch)
            position[0] += np.cos(angle_abs) * link["length"]
            position[1] += np.sin(angle_abs) * link["length"]
            index += 1
        ee.append(position)
    return {'ee':ee,'patches':patches}
   
class PoseSequencePrinter:
    def __init__(self, ax, robots_json, sequence):
        self.ax = ax
        self.robotsJson = robots_json
        self.sequence = sequence
        self.sequenceCounter = 0
        self.reset()

    def clearCapsules(self):
        # TODO remove patches from figure
        self.linksPatches = []
        return

    def clearLines(self):
        # TODO remove lines from figure
        self.eeTrajectoriesLines = []
        return

    def reset(self):
        self.clearCapsules()
        self.clearLines()

        self.eeTrajectories = []
        for robot in self.robotsJson:
            self.eeTrajectories.append({'x':[], 'y':[]})

    def draw(self):
        self.clearCapsules()
        self.clearLines()

        ee_and_links = print_pose(self.ax, self.robotsJson, self.sequence[self.sequenceCounter], 'b', 0.5)
        self.linksPatches = ee_and_links['patches']
        ee = ee_and_links['ee']
        for index in range(0,len(ee)):
            recipient = self.eeTrajectories[index]
            recipient['x'].append(ee[index][0])
            recipient['y'].append(ee[index][1])
            ee_line = self.ax.plot(recipient['x'], recipient['y'], '--', color='b')
            self.eeTrajectoriesLines.append(ee_line)

        self.sequenceCounter += 1
        if self.sequenceCounter == len(self.sequence):
            self.sequenceCounter = 0

def to_rad_pose(angles):
    result = []
    for val in angles:
        result.append(to_rad(val))
    return result

class Printer:
    def __init__(self, log):
        self.scene = log["scene"]
        self.sequencePrinter = None

    def printScene(self, ax):
        for sphere in self.scene["obstacles"]:
            print_sphere(ax, sphere)
            
    def printTree(self, ax, tree, color):
        print_pose(ax, self.scene["robots"], tree[0]["end"], color, 0.7)
        tree_size = len(tree)
        if tree_size < 30:
            for edge in tree: 
                print_pose(ax, self.scene["robots"], edge["end"], color, 0.1)
        else:
            step = math.ceil(len(tree) / 30.0)
            for index in range(0, len(tree), step):
                print_pose(ax, self.scene["robots"], tree[index]["end"], color, 0.1)
        
    def printSolutions(self, ax, solutions):
        if(len(solutions) == 0):
            return
        sequence = solutions[0]["sequence"]
        self.sequencePrinter = PoseSequencePrinter(ax, self.scene["robots"], sequence)
        for pose in sequence:
            self.sequencePrinter.draw()

    def finalize(self, ax):
        limits.printCorners(ax)
