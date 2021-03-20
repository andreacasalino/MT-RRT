import json
import matplotlib.pyplot as plt
import numpy as np

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

        addQuiver(traj["start"], "red", 2)
        addQuiver(traj["end"], "red", 2)

        traj_x = []
        traj_y = []
        for s in traj["states"]:
            traj_x.append(s[0])
            traj_y.append(s[1])
            addQuiver(s, "#5AF0C7", 0.7)

        ax.plot(traj_x , traj_y, "blue", linewidth = 0.7)


plots = FigureManager("TrajGen.json")
plt.show()