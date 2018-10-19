import rps.robotarium as robotarium
import numpy as np

#helper function to generate heatmap
def getpts(f, x_dom, y_dom, res):
    x = np.linspace(x_dom[0], y_dom[1], res)
    y = np.linspace(y_dom[0], y_dom[1], res)
    x, y = np.meshgrid(x, y)
    x = x.ravel()
    y = y.ravel()
    C = np.zeros_like(x)
    for i in range(x.shape[0]):
        C[i] = np.sqrt(f((x[i], y[i])))
    return x, y, C

def draw_f(sim, f, res=200):
    x_dom = (sim.boundary[0], sim.boundary[0] + sim.boundary[2])
    y_dom = (sim.boundary[1], sim.boundary[1] + sim.boundary[3])
    print(x_dom)
    print(y_dom)
    x, y, z = getpts(f, x_dom, y_dom, res)
    sim.axes.hexbin(x, y, C=z, cmap="plasma")

