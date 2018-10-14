import rps.robotarium as robotarium
import numpy as np

#helper function to generate heatmap
def getpts(f, dom, res):
    x = y = np.linspace(dom[0], dom[1], res)
    x, y = np.meshgrid(x, y)
    x = x.ravel()
    y = y.ravel()
    C = np.zeros_like(x)
    for i in range(x.shape[0]):
        C[i] = f((x[i], y[i]))
    return x, y, C

def draw_f(sim, f, dom=(-0.5, 0.5), res=200):
    x, y, z = getpts(f, dom, res)
    sim.axes.hexbin(x, y, C=z)

