import matplotlib.pyplot as plt
import matplotlib.patches as ptc
import numpy as np
from matplotlib.animation import FuncAnimation

fig, ax = plt.subplots()

# circle = ptc.Circle([0,0], 1, color="green")
circle = ptc.Circle([0,0], 1, color="#49C495")
circle.set_edgecolor("black")
circle.set_linestyle("-")
circle.set_linewidth(1)
ax.add_patch(circle)

ax.set_xlim(-2, 12)
ax.set_ylim(-2, 12)
ax.set_aspect('equal', 'box')

def animationFnc(f):
    circle.set_center([f,f])

animation = FuncAnimation(fig, func=animationFnc, frames=np.arange(0, 10, 0.1), interval=10, repeat=True)
plt.show()