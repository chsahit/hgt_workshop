import rps.robotarium as robotarium
import matplotlib.patches as patches
import numpy as np
import random
import time


A = ((0.25, 0.0), (0.0, 0.25), (-0.25, 0.0), (0.0, -0.25))
DIM = 0.25

class Vertex:
    def __init__(self, coords, cost, pred):
        self.coords = coords
        self.cost = cost
        self.pred = pred

def l2(start, goal):
    return ((start[0] - goal[0])**2 + (start[1] - goal[1])**2)**0.5

def tuple_to_pt(coords):
    return np.array([[coords[0]], [coords[1]], [0]])

def isValid(x, x_dom, y_dom):
    return x[0] > x_dom[0] and x[0] < x_dom[1] and x[1] > y_dom[0] and x[1] < y_dom[1]

def gen_traj(x_dom, y_dom, length, start):
    print(x_dom)
    print(y_dom)
    start_time = time.time()
    traj = [start]
    while l2(start, traj[-1]) < length:
        a = A[random.randint(0, len(A) - 1)]
        next_state = (traj[-1][0] + a[0], traj[-1][1] + a[1])
        if (not isValid(next_state, x_dom, y_dom)):
            continue
        runtime = time.time() - start_time
        if (runtime > 10):
            pass
        traj.append(next_state)
    return traj

# def truncate_floor(value, dim):
    # value = int(1000 * value)
    # dim = int(1000 * dim)
    # return int(((value // dim ) * dim) / 1000)
def discrete_x(x):
    return (np.digitize([x], np.linspace(-1.5, 1.5, 13))[0] - 1) * 0.25 - 1.5

def discrete_y(y):
    return (np.digitize([y], np.linspace(-1.0, 1.0, 9))[0] - 1) * 0.25 - 1.0


def draw_maze(sim, start, complexity):
    x_dom = (sim.boundary[0] + DIM, sim.boundary[0] + sim.boundary[2] - DIM)
    y_dom = (sim.boundary[1] + DIM, sim.boundary[1] + sim.boundary[3] - DIM)
    traj = gen_traj(x_dom, y_dom, 1.5, start)
    blocks = list()
    for i in range(complexity):
        cell = (discrete_x(random.uniform(x_dom[0], x_dom[1])), discrete_y(random.uniform(y_dom[0], y_dom[1])))
        if (not cell in traj):
            blocks.append(cell)
            #sim.axes.add_patch(patches.Circle(cell, 0.025))
            sim.axes.add_patch(patches.Rectangle((cell[0]-DIM/2, cell[1]-DIM/2), DIM, DIM))
    # goal = (round(traj[-1][0], 1), round(traj[-1][1], 1))
    goal = (discrete_x(traj[-1][0]), discrete_y(traj[-1][1]))
    sim.axes.add_patch(patches.Circle(goal, 0.05, color='g'))
    return blocks, goal
