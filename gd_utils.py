import rps.robotarium as robotarium
import numpy as np

#helper function to generate heatmap
def getpts(f, x_dom, y_dom, res):
    px = int(res * (x_dom[1] - x_dom[0])), int(res * (y_dom[1] - y_dom[0]))
    x_basis = np.linspace(x_dom[0], x_dom[1], px[0])
    y_basis = np.linspace(y_dom[0], y_dom[1], px[1])
    z = np.zeros(px, dtype=np.float32)
    for r in range(px[1]):
        for c in range(px[0]):
            z[c,r] = np.cbrt(f((x_basis[c], y_basis[r])))
    return z

def draw_f(sim, f, res=100):
    x_dom = (sim.boundary[0], sim.boundary[0] + sim.boundary[2])
    y_dom = (sim.boundary[1], sim.boundary[1] + sim.boundary[3])
    print(x_dom)
    print(y_dom)
    z = getpts(f, x_dom, y_dom, res)
    sim.axes.imshow(z, extent=[x_dom[0], x_dom[1], y_dom[1], y_dom[0]], cmap="autumn")

