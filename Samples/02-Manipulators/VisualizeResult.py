import matplotlib.pyplot as plt
import matplotlib
import matplotlib.transforms as trsf
import matplotlib.patches as ptc
import numpy as np
from matplotlib.animation import FuncAnimation

def fill(patch, color, alpha=1):
    patch.set_color(color)
    patch.set_alpha(alpha)
    patch.set_edgecolor("black")
    patch.set_linestyle("-")
    patch.set_linewidth(0.7)

def set_transform(ax, patch, tx, ty, angle):
    patch.set_transform(trsf.Affine2D().rotate(angle).translate(tx,ty) + ax.transData)

def add_obstacle(ax, xy, ray):
    ob = ptc.Circle(xy, ray)
    fill(ob, "#E75328")
    ax.add_patch(ob)

def add_capsule(ax, lenght, ray, color, alpha=1):
    points = 200
    def getArc(angleS, angleE):
        angles = np.linspace(angleS, angleE, points)
        vertices = np.zeros((points,2) , dtype=np.float64)
        vertices[:,0] = ray * np.cos(angles)
        vertices[:,1] = ray * np.sin(angles)
        return vertices

    vertices = np.zeros((2*points,2) , dtype=np.float64)
    vertices[0:points,:] = getArc(-0.5 * np.pi, 0.5 * np.pi)
    vertices[points:,:] = getArc(0.5 * np.pi,3.0 * np.pi * 0.5)
    vertices[points:,0] = vertices[points:,0] - lenght

    pathPatch = ptc.PathPatch(matplotlib.path.Path(vertices, closed=True))
    fill(pathPatch, color, alpha)
    ax.add_patch(pathPatch)
    return pathPatch

class Manipulator:
    def __init__(self, ax, data):
        dof = int((len(data) - 2) / 2)
        self.ax = ax
        self.base = [data[0], data[1]]
        self.links = []
        zeroPose = []
        for k in range(0, dof, 1):
            self.links.append({"length":data[2+k], "ray":data[2+dof+k]})
            zeroPose.append(0)
        self.shapes = self.__makeCapsuleChain('blue')
        self.setPose(zeroPose)

    def __makeCapsuleChain(self, color, alpha=1):
        chain = []
        for l in self.links:
            chain.append(add_capsule(self.ax, l["length"], l["ray"], color, alpha))
        return chain

    def __setPose(self, pose, chain):
        angleCum = 0.0
        posCum = [self.base[0], self.base[1]]
        for p in range(0, len(self.links), 1):
            angleCum = angleCum + pose[p]
            posNew = [ posCum[0] + self.links[p]["length"] * np.cos(angleCum),
                       posCum[1] + self.links[p]["length"] * np.sin(angleCum)]
            set_transform(self.ax, chain[p], posNew[0], posNew[1], angleCum)
            posCum = posNew
        return posCum

    def setPose(self, pose):
        return self.__setPose(pose, self.shapes)

    def make_static_pose(self, pose, color):
        chain = self.__makeCapsuleChain(color, 0.1)
        self.__setPose(pose, chain)
        return

class Scene:
    def __init__(self, fig, ax, problem, solution, trees):
        self.robots = []
        self.solution = solution
        self.tcp_trajectories = []
        for o in problem["obstacles"]:
            add_obstacle(ax, [o[0], o[1]], o[2])
        for r in problem["robots"]:
            self.robots.append(Manipulator(ax, r))
            traj, = ax.plot(0, 0)
            traj.set_xdata([])
            traj.set_ydata([])
            self.tcp_trajectories.append({"traj":traj,"x_coord":[],"y_coord":[]})
        # print tree poses
        self.showTree(ax, trees[0], 'blue')
        if(len(trees) > 1):
            self.showTree(ax, trees[1], 'green')
        # enable the animation
        if(len(self.solution) > 0):
            self.animation = FuncAnimation(fig, func=self.setPose, frames=range(0,len(self.solution),1), interval=50, repeat=True)
        #set axis limit
        ax.set_xlim(-1000, 1000)
        ax.set_ylim(-1000, 1000)

    def showTree(self, ax, tree, color):
        if(len(tree) < 50):
            treePos = range(0,len(tree),1)
        else:
            treePos = np.linspace(0, len(tree) - 1, 50).astype(int)
        for t in treePos:
            pos = 0
            for r in self.robots:
                r.make_static_pose(tree[t][pos:pos + len(r.links)], color)
                pos = pos + len(r.links)

    def setPose(self, index):
        if(index == 0):
            for t in self.tcp_trajectories:
                t["x_coord"] = []
                t["y_coord"] = [] 

        pos = 0
        tPos = 0
        for r in self.robots:
            tcp_pos = r.setPose(self.solution[index][pos:pos + len(r.links)])
            self.tcp_trajectories[tPos]["x_coord"].append(tcp_pos[0])
            self.tcp_trajectories[tPos]["y_coord"].append(tcp_pos[1])
            self.tcp_trajectories[tPos]["traj"].set_xdata(self.tcp_trajectories[tPos]["x_coord"])
            self.tcp_trajectories[tPos]["traj"].set_ydata(self.tcp_trajectories[tPos]["y_coord"])
            tPos = tPos+1
            pos = pos + len(r.links)

def VisualizeResult(fig, ax, problem, result_ij):
    return Scene(fig, ax, problem, result_ij["solution"], result_ij["trees"])
