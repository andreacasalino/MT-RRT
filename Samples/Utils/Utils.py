import matplotlib.pyplot as plt
import matplotlib
import matplotlib.transforms as trsf
import matplotlib.patches as ptc
import numpy as np

def fillPatch(patch, color, alpha=1):
    patch.set_color(color)
    patch.set_alpha(alpha)
    patch.set_edgecolor("black")
    patch.set_linestyle("-")
    patch.set_linewidth(0.7)

def locatePatch(ax, patch, tx, ty, angle):
    patch.set_transform(trsf.Affine2D().rotate(angle).translate(tx,ty) + ax.transData)


def makePatch(ax, vertices, color, alpha=1):
    pathPatch = ptc.PathPatch(matplotlib.path.Path(vertices, closed=True))
    fillPatch(pathPatch, color, alpha)
    ax.add_patch(pathPatch)
    return pathPatch

def makeSphere(ax, xy, ray, color, alpha=1):
    circle = ptc.Circle(xy, ray)
    fillPatch(circle, color, alpha)
    ax.add_patch(circle)
    return circle

def makeCapsule(ax, lenght, ray, color, alpha=1):
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

    return makePatch(ax, vertices, color, alpha)

# width along y, length is along x
def makeRectangle(ax, width, length, color, alpha=1):
    w = 0.5 * width
    l = 0.5 * length
    vertices = np.zeros((5,2) , dtype=np.float64)
    vertices[:,0] = np.array([l,l,-l,-l,l])
    vertices[:,1] = np.array([-w,w,w,-w,-w])

    return makePatch(ax, vertices, color, alpha)

# state = [x, y, angle]
def makearrow(ax, state, length, color, width):
    x = [state[0], state[0] + length * np.cos(state[2])]
    y = [state[1], state[1] + length * np.sin(state[2])]
    ax.plot(x , y, color, linewidth = width)
    coeff = 0.1
    vertices = np.zeros((4,2) , dtype=np.float64)
    vertices[:,0] = np.array([0, -length*coeff, -length*coeff, 0])
    vertices[:,1] = np.array([0, 0.5*length*coeff, -0.5*length*coeff, 0])
    arrow = makePatch(ax, vertices, color)
    locatePatch(ax, arrow, state[0] + length * np.cos(state[2]), state[1] + length * np.sin(state[2]), state[2])


class AnimableCurve:
    def __init__(self, ax, x, y):
        if(len(x) == 0):
            return
        if(len(y) != len(x)):
            return
        self.x = x
        self.y = y
        self.xCumulated = []
        self.yCumulated = []
        self.curve, = ax.plot(0, 0)

    def setIndex(self, index):
        if(index == 0):
            self.xCumulated = []
            self.yCumulated = []
        else:
            self.xCumulated.append(self.x[index])
            self.yCumulated.append(self.y[index])
        self.curve.set_xdata(self.xCumulated)
        self.curve.set_ydata(self.yCumulated)
        
