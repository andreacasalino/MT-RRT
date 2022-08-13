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
    patch = mpatches.PathPatch(path, facecolor=facecolor, edgecolor=edgecolor, alpha=alpha)
    ax.add_patch(patch)
    for sample in samples:
        limits.extend(sample)

def print_sphere(ax, sphere):
    samples = []
    for angle in np.arange(0.0, 2 * np.pi, to_rad(5)):
        x = sphere["center"][0] + sphere["ray"] * np.cos(angle)
        y = sphere["center"][1] + sphere["ray"] * np.sin(angle)
        samples.append([x, y])
    print_patch(ax, samples, 'r')

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
    print_patch(ax, samples, color, alpha, 'k')

def print_pose(ax, robots_json, pose, color, alpha):
    index = 0
    ee = []
    for robot in robots_json:
        angle_abs = 0
        position = [0,0]
        if "base" in robot:
            angle_abs = robot["base"]["angle"]
            angle_abs = to_rad(angle_abs)
            position = [robot["base"]["position"][0], robot["base"]["position"][1]]
        for link in robot["links"]:
            angle_abs += pose[index]
            print_capsule(ax, link, Transform(position, angle_abs), color, alpha)
            position[0] += np.cos(angle_abs) * link["length"]
            position[1] += np.sin(angle_abs) * link["length"]
            index += 1
        ee.append(position)
    return ee

def print_traj(ax, traj, color):
    x = []
    y = []
    for point in traj:
        x.append(point[0])
        y.append(point[1])
    ax.plot(x, y, '--', color=color)
    

def to_rad_pose(angles):
    result = []
    for val in angles:
        result.append(to_rad(val))
    return result

class Printer:
    def __init__(self, log):
        self.scene = log["scene"]

        self.start=None
        if "start" in log:
            self.start = to_rad_pose(log["start"])

        self.end=None
        if "end" in log:
            self.end = to_rad_pose(log["end"])

    def printScene(self, ax):
        for sphere in self.scene["obstacles"]:
            print_sphere(ax, sphere)
        if self.start != None:
            print_pose(ax, self.scene["robots"], self.start, 'b', 1.0)
        if self.end != None:
            print_pose(ax, self.scene["robots"], self.end, 'g', 1.0)
            
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
        solution = solutions[0]["sequence"]
        ee_traj = []
        for robot in self.scene["robots"]:
            ee_traj.append([])
        for pose in solution:
            ee = print_pose(ax, self.scene["robots"], pose, 'b', 1.0)
            for index in range(0,len(ee)):
                ee_traj[index].append(ee[index])
        for ee in ee_traj:
            print_traj(ax, ee, 'b')

    def finalize(self, ax):
        limits.printCorners(ax)
