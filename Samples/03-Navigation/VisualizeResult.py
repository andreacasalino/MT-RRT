from Utils import *
from matplotlib.animation import FuncAnimation

def adaptLimits(lim, sequence):
    for s in sequence:
        if(s < lim[0]):
            lim[0] = s
        if(s > lim[1]):
            lim[1] = s
    return lim

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
    def __init__(self, fig, ax, problem, solution, trees, start, target):
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
            ax.plot(self.cart.curveTraj.x , self.cart.curveTraj.y, color="blue", linewidth=0.35)
            self.animation = FuncAnimation(fig, func=self.setIndex, frames=range(0,len(solution),1), interval=50, repeat=True)
        #set axis limit
        xLim = [problem["boundaries"][0], problem["boundaries"][2]]
        yLim = [problem["boundaries"][1], problem["boundaries"][3]]        
        if(len(solution) > 0):
            adaptLimits(xLim, self.cart.curveTraj.x)
            adaptLimits(yLim, self.cart.curveTraj.y)
        ax.set_xlim(xLim[0], xLim[1])
        ax.set_ylim(yLim[0], yLim[1])
        # plot start end
        delta = [problem["boundaries"][0] - problem["boundaries"][2] , problem["boundaries"][3] - problem["boundaries"][1]]
        L = 0.1 * np.sqrt(delta[0]*delta[0] + delta[1]*delta[1])
        makearrow(ax, start, L, "red", 2)
        makearrow(ax, target, L, "#F07F00", 2)

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
    return Scene(fig, ax, problem, result_ij["solution"], result_ij["trees"],result_ij["start"], result_ij["target"])



import json

class DebuggerTraj:
    def __init__(self, problem, traj):
        self.fig, self.ax = plt.subplots()
        self.cart = AnimableCart( self.ax, problem["cart"]["width"], problem["cart"]["lenght"], traj["states"])
        self.L = self.getL(traj)

        makearrow(self.ax, traj["start"], self.L, "red", 2)
        makearrow(self.ax, traj["end"], self.L, "#F07F00", 2)
        for t in traj["states"]:
            makearrow(self.ax, t, self.L, "#7E65EE", 0.7)

        if(len(traj["states"]) > 0):
            self.anim = FuncAnimation(self.fig, func=self.cart.setIndex, frames=range(0,len(traj["states"]),1), interval=50, repeat=True)
        self.ax.set_aspect('equal', 'box')

    def getL(self, traj):
        delta = [traj["end"][0] - traj["start"][0], traj["end"][1] - traj["start"][1]]
        return np.sqrt(delta[0]*delta[0] + delta[1]*delta[1])* 0.2

    def addQuiver(self, state, color, width):
        x = [state[0], state[0] + self.L * np.cos(state[2])]
        y = [state[1], state[1] + self.L * np.sin(state[2])]
        self.ax.plot(x , y, color, linewidth = width)
        coeff = 0.1
        vertices = np.zeros((4,2) , dtype=np.float64)
        vertices[:,0] = np.array([0, -self.L*coeff, -self.L*coeff, 0])
        vertices[:,1] = np.array([0, 0.5*self.L*coeff, -0.5*self.L*coeff, 0])
        arrow = makePatch(self.ax, vertices, color)
        locatePatch(self.ax, arrow, state[0] + self.L * np.cos(state[2]), state[1] + self.L * np.sin(state[2]), state[2])

def DebugLog():    
    data = None
    with open("DebugLog.json") as json_file:
        data = json.load(json_file)
    trajs  = []
    for t in data["traj"]:
        trajs.append(DebuggerTraj(data["problem"] , t))
    return trajs