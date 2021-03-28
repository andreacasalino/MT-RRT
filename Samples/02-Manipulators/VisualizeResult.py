from Utils import *
from matplotlib.animation import FuncAnimation

class Capsules:
    def __init__(self, ax, data, color, alpha=1):
        self.rawData = data
        self.ax = ax
        self.base = [data[0], data[1], data[2]]
        self.lengths = []
        self.rays = []
        self.patches = []
        dof = int((len(data) - 3) / 2)
        for k in range(0, dof, 1):
            self.lengths.append(data[3+k])
            self.rays.append(data[3+dof+k])
            self.patches.append(makeCapsule(ax, self.lengths[k], self.rays[k], color, alpha))
        
    def getDof(self):
        return len(self.patches)

    def setPose(self, pose):
        angleCum = self.base[2]
        posCum = [self.base[0], self.base[1]]
        for p in range(0, self.getDof(), 1):
            angleCum = angleCum + pose[p]
            posNew = [ posCum[0] + self.lengths[p] * np.cos(angleCum),
                        posCum[1] + self.lengths[p] * np.sin(angleCum)]
            locatePatch(self.ax, self.patches[p], posNew[0], posNew[1], angleCum)
            posCum = posNew
        return posCum

class Manipulator:
    def __init__(self, ax, data):
        self.capsules = Capsules(ax, data, "blue")
        self.setPose(np.zeros(self.capsules.getDof()))

    def setPose(self, pose):
        return self.capsules.setPose(pose)

    def makeStaticPose(self, pose, color):
        chain = Capsules(self.capsules.ax, self.capsules.rawData, color, 0.1)
        chain.setPose(pose)

class AnimableManipulator:
    def __init__(self, ax, manipulator, poses):
        self.poses = poses
        self.manipulator = manipulator
        x_coords = []
        y_coords = []
        for p in self.poses:
            cartesian = self.manipulator.setPose(p)
            x_coords.append(cartesian[0])
            y_coords.append(cartesian[1])
        self.tcpLine = AnimableCurve(ax, x_coords, y_coords) 

    def setIndex(self, index):
        self.manipulator.setPose(self.poses[index])
        self.tcpLine.setIndex(index)

class Scene:
    def __init__(self, fig, ax, problem, solution, trees):
        for o in problem["obstacles"]:
            makeSphere(ax, [o[0], o[1]], o[2], "#E75328")
        self.robots = []
        p = 0
        for r in problem["robots"]:
            manip = Manipulator(ax, r)
            poses = []
            for s in solution:
                poses.append(s[p:p+manip.capsules.getDof()])
            self.robots.append(AnimableManipulator(ax, manip, poses))
            p = p + manip.capsules.getDof()
        # print tree poses
        self.showTree(ax, trees[0], 'blue')
        if(len(trees) > 1):
            self.showTree(ax, trees[1], 'green')
        # enable the animation
        if(len(solution) > 0):
            self.animation = FuncAnimation(fig, func=self.setIndex, frames=range(0,len(solution),1), interval=50, repeat=True)
        #set axis limit
        ax.set_xlim(-100, 100)
        ax.set_ylim(-100, 100)

    def showTree(self, ax, tree, color):
        if(len(tree) < 20):
            treePos = range(0,len(tree),1)
        else:
            treePos = np.linspace(0, len(tree) - 1, 20).astype(int)
        for t in treePos:
            pos = 0
            for r in self.robots:
                r.manipulator.makeStaticPose(tree[t][pos:pos + r.manipulator.capsules.getDof()], color)
                pos = pos + r.manipulator.capsules.getDof()

    def setIndex(self, index):
        for r in self.robots:
            r.setIndex(index)

def VisualizeResult(fig, ax, problem, result_ij):
    return Scene(fig, ax, problem, result_ij["solution"], result_ij["trees"])
