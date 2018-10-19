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
from setpos import setpos

def h(start, goal):
    return abs(start[0] - goal[0]) + abs(start[1] - goal[1])

def search(start, obstacles, goal):
    visited = set()
    start_vertex = Vertex(start, 0, None)
    pq = PriorityQueue()
    pq.push(start_vertex, start_vertex.cost)
    while (not pq.empty()):
        curr = pq.pop()
        visited.add(curr.coords)
        if curr.coords == goal:
            solution = list()
            while curr != None:
                solution.append(curr.coords)
                curr = curr.pred
            return list(reversed(solution))
        for action in A:
            succ = traj_utils.discrete_x(curr.coords[0] + action[0]), traj_utils.discrete_y(curr.coords[1] + action[1])
            if ((not succ in visited) and (not succ in obstacles)):
                cost = traj_utils.l2(succ, curr.coords)
                succ_vertex = Vertex(succ, cost + curr.cost, curr)
                heur = h(succ, goal)
                pq.push(succ_vertex, succ_vertex.cost + heur)


init_pt = np.array([[-1.0], [0.5], [0]])

# Instantiate Robotarium object
N = 1

sim = robotarium.Robotarium(number_of_agents=N, show_figure=True, save_data=True, update_time=1.0)
obstacles, goal = traj_utils.draw_maze(sim, (init_pt[0][0], init_pt[1][0]), 64)
solution = search((init_pt[0][0], init_pt[1][0]), obstacles, goal)
print(solution)

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
#goal_pt = traj_utils.tuple_to_pt(goal)
#setpos(sim, x, goal_pt, si_barrier_cert, N)
#time.sleep(1.0)

#setpos(sim, x, traj_utils.tuple_to_pt(solution[-1]),  si_barrier_cert, N)
# Always call this function at the end of your scripts!  It will accelerate the
# execution of your experiment
sim.call_at_scripts_end()
