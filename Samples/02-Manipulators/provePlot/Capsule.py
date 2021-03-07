import matplotlib
import matplotlib.transforms as trsf
import matplotlib.pyplot as plt
import matplotlib.patches as ptc
import numpy as np
from matplotlib.animation import FuncAnimation

class Capsule:
    def __init__(self, ax, lenght, ray, points=100):
        self.ray = ray
        self.lenght = lenght
        self.pathPatch = None
        self.trsf = None
        self.ax = ax

        vertices = np.zeros((2*points,2) , dtype=np.float64)
        vertices[0:points,:] = self.getArc(-0.5 * np.pi, 0.5 * np.pi, points)
        vertices[points:,:] = self.getArc(0.5 * np.pi,3.0 * np.pi * 0.5, points)
        vertices[points:,0] = vertices[points:,0] - self.lenght

        self.pathPatch = matplotlib.patches.PathPatch(matplotlib.path.Path(vertices, closed=True))
        self.ax.add_patch(self.pathPatch)

    def getArc(self, angleS, angleE, points):
        angles = np.linspace(angleS, angleE, points)
        vertices = np.zeros((points,2) , dtype=np.float64)
        vertices[:,0] = self.ray * np.cos(angles)
        vertices[:,1] = self.ray * np.sin(angles)
        return vertices

    def setTrsf(self, trsf):
        self.pathPatch.set_transform(trsf + ax.transData)


fig, ax = plt.subplots()

c1 = Capsule(ax, 2, 0.2)
c1.pathPatch.set_color("#49C495")
c1.pathPatch.set_edgecolor("black")
c1.pathPatch.set_linestyle("-")
c1.pathPatch.set_linewidth(1)
c1.pathPatch.set_alpha(0.5)

ax.set_xlim(-4, 4)
ax.set_ylim(-3, 3)
ax.set_aspect('equal', 'box')


def animationFnc(a):
    c1.setTrsf(trsf.Affine2D().rotate(a).translate(0.5,0.5))
    # c1.setTrsf(trsf.Affine2D().rotate(a))

animation = FuncAnimation(fig, func=animationFnc, frames=np.arange(0, np.pi*0.5, 0.02), interval=10, repeat=True)
plt.show()
