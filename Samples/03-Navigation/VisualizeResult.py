from Utils import *
from matplotlib.animation import FuncAnimation

class AnimableCart:
    def __init__(self, ax, width, length, traj):
        self.w = width
        self.l = length
        self.ax = ax
        self.patch = makeRectangle(ax, width, length, "yellow")
        x_coords = []
        y_coords = []
        for t in traj:
            x_coords.append(t[0])
            y_coords.append(t[1])
        self.curveTraj = AnimableCurve(ax, x_coords, y_coords) 
        self.poses = traj

    def setPose(self, pose):
        locatePatch(self.ax, self.patch, pose[0], pose[1], pose[2])

    def makeStaticPose(self, pose, color):
        poseCart = makeRectangle(self.ax, self.w, self.l, color, 0.1)
        locatePatch(self.ax, poseCart, pose[0], pose[1], pose[2])

    def setIndex(self, index):
        self.setPose(self.poses[index])
        self.curveTraj.setIndex(index)

class Scene:
    def __init__(self, fig, ax, problem, solution, trees):
        for o in problem["obstacles"]:
            makeSphere(ax, [o[0], o[1]], o[2], "#E75328")
        self.robots = []
        self.cart = AnimableCart(ax, problem["cart"]["width"], problem["cart"]["lenght"], solution)
        # print tree poses
        self.showTree(ax, trees[0], 'blue')
        if(len(trees) > 1):
            self.showTree(ax, trees[1], 'green')
        # enable the animation
        if(len(solution) > 0):
            self.animation = FuncAnimation(fig, func=self.setIndex, frames=range(0,len(solution),1), interval=50, repeat=True)
        #set axis limit
        ax.set_xlim(problem["boundaries"][0], problem["boundaries"][2])
        ax.set_ylim(problem["boundaries"][1], problem["boundaries"][3])

    def showTree(self, ax, tree, color):
        if(len(tree) < 50):
            treePos = range(0,len(tree),1)
        else:
            treePos = np.linspace(0, len(tree) - 1, 50).astype(int)
        for t in treePos:
            self.cart.makeStaticPose(tree[t], color)

    def setIndex(self, index):
        self.cart.setIndex(index)

def VisualizeResult(fig, ax, problem, result_ij):
    return Scene(fig, ax, problem, result_ij["solution"], result_ij["trees"])
