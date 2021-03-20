import json
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.transforms as trsf
import matplotlib.patches as ptc
import numpy as np

def fill(patch, color, alpha=1):
    patch.set_color(color)
    patch.set_alpha(alpha)
    patch.set_edgecolor("black")
    patch.set_linestyle("-")
    patch.set_linewidth(0.7)

def set_transform(ax, patch, tx, ty, angle):
    patch.set_transform(trsf.Affine2D().rotate(angle).translate(tx,ty) + ax.transData)

class FigureManager:
    def __init__(self, fileName):
        self.figures = []
        self.axes = []
        trajectories = None
        with open(fileName) as json_file:
            trajectories = json.load(json_file)
            for t in trajectories:
                self.addTrajectory(t)

    def addTrajectory(self, traj):
        fig, ax = plt.subplots(nrows=1, ncols=1)
        self.figures.append(fig)
        self.axes.append(ax)

        def getL():
            delta = [traj["end"][0] - traj["start"][0], traj["end"][1] - traj["start"][1]]
            return np.sqrt(delta[0]*delta[0] + delta[1]*delta[1])
        L = getL() * 0.2

        def addQuiver(state, color, width):
            x = [state[0], state[0] + L * np.cos(state[2])]
            y = [state[1], state[1] + L * np.sin(state[2])]
            ax.plot(x , y, color, linewidth = width)
            coeff = 0.1
            
            vertices = np.zeros((4,2) , dtype=np.float64)
            vertices[:,0] = np.array([0, -L*coeff, -L*coeff, 0])
            vertices[:,1] = np.array([0, 0.5*L*coeff, -0.5*L*coeff, 0])
            pathPatch = ptc.PathPatch(matplotlib.path.Path(vertices, closed=True))
            fill(pathPatch, color, 1)
            ax.add_patch(pathPatch)
            set_transform(ax, pathPatch, state[0] + L * np.cos(state[2]), state[1] + L * np.sin(state[2]), state[2])

        addQuiver(traj["start"], "red", 2)
        addQuiver(traj["end"], "#F07F00", 2)

        traj_x = []
        traj_y = []
        for s in traj["states"]:
            traj_x.append(s[0])
            traj_y.append(s[1])
            addQuiver(s, "#7E65EE", 0.7)

        ax.plot(traj_x , traj_y, "blue", linewidth = 0.7)
        ax.set_aspect('equal', 'box')


plots = FigureManager("TrajGen.json")
plt.show()