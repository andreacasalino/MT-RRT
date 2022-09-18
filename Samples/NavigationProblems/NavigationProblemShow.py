import math
import matplotlib.animation as animation

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

def print_cart(ax, cart, state, alpha=1.0):
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
    return print_patch(ax, points_trsf, 'cyan', alpha)

class CartTrajPrinter:
    def __init__(self, ax, cart_json, sequence):
        self.ax = ax
        self.cartJson = cart_json
        self.sequence = sequence
        self.reset()

    def clearCart(self):
        try:
            self.cartPatch.remove()
        except:
            pass
        self.cartPatch = None

    def clearCartTraj(self):
        try:
            line = self.cartTrajLine.pop(0)
            line.remove()
        except:
            pass
        self.cartTrajLine = None

    def reset(self):
        self.clearCart()
        self.clearCartTraj()

        self.cartTraj = {'x':[], 'y':[]}
        return []

    def draw(self, index):
        self.clearCart()
        self.clearCartTraj()

        self.cartPatch = print_cart(self.ax, self.cartJson, self.sequence[index])
        self.cartTraj['x'].append(self.sequence[index][0])
        self.cartTraj['y'].append(self.sequence[index][1])
        self.cartTrajLine = self.ax.plot(self.cartTraj['x'], self.cartTraj['y'], '--', color='b')
        return []

class Printer:
    def __init__(self, log):
        self.scene = log["scene"]
        self.sequencePrinter = None
        self.sequenceAnimation = None

    def printScene(self, ax, fig):
        for sphere in self.scene["obstacles"]:
            print_sphere(ax, sphere)
            
    def printTree(self, ax, fig, tree, color):
        print_cart(ax, self.scene["cart"], tree[0]["end"], 0.7)
        tree_size = len(tree)
        if tree_size < 30:
            for edge in tree: 
                print_cart(ax, self.scene["cart"], edge["end"], 0.1)
        else:
            step = math.ceil(len(tree) / 30.0)
            for index in range(0, len(tree), step):
                print_cart(ax, self.scene["cart"], tree[index]["end"], 0.1)
        
    def printSolutions(self, ax, fig, solutions):
        if(len(solutions) == 0):
            return
        sequence = solutions[0]["sequence"]
        self.sequencePrinter = CartTrajPrinter(ax, self.scene["cart"], sequence)
        self.sequenceAnimation = animation.FuncAnimation(fig, self.sequencePrinter.draw, frames=len(sequence),
                              interval=250, blit=True, init_func=self.sequencePrinter.reset)

    def finalize(self, ax, fig):
        limits.printCorners(ax)
