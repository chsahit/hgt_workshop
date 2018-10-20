import rps.robotarium as robotarium
import rps.utilities.graph as graph
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *

import numpy as np
import time
import traj_utils
from PriorityQueue import PriorityQueue
from traj_utils import Vertex, A
from set_pose import setpos

def h(start, goal):
    pass

def search(start, obstacles, goal):
    visited = set()
    start_vertex = Vertex(start, 0, None)
    OPEN = PriorityQueue()
    OPEN.push(start_vertex, start_vertex.cost)
    #implement the rest here!


init_pt = np.array([[-1.0], [0.5], [0]])

# Instantiate Robotarium object
N = 1

sim = robotarium.Robotarium(number_of_agents=N, show_figure=True, save_data=True, update_time=1.0)
obstacles, goal = traj_utils.draw_maze(sim, (init_pt[0][0], init_pt[1][0]), 64)
solution = search((init_pt[0][0], init_pt[1][0]), obstacles, goal)

# Create barrier certificates to avoid collision
uni_barrier_cert = create_unicycle_barrier_certificate(N, safety_radius=0.05)
si_barrier_cert = create_single_integrator_barrier_certificate(N)

# define x initially
x = sim.get_poses()
#print(x)
sim.step()

for coordinate in solution:
    goal_points = traj_utils.tuple_to_pt(coordinate)
    setpos(sim, x, goal_points, uni_barrier_cert, N)
    time.sleep(1.0)
    sim.step()
    sim.figure.canvas.flush_events()
    print(coordinate)

# Always call this function at the end of your scripts!  It will accelerate the
# execution of your experiment
sim.call_at_scripts_end()
