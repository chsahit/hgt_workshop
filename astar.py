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

def h(start, goal):
    return ((start[0] - goal[0])**2 + (start[1] - goal[1])**2)**0.5

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
            succ = (round(curr.coords[0] + action[0], 1), round(curr.coords[1] + action[1], 1))
            if ((not succ in visited) and (not succ in obstacles)):
                succ_vertex = Vertex(succ, curr.cost + 0.1, curr)
                heur = h(succ, goal)
                pq.push(succ_vertex, succ_vertex.cost + heur)



init_pt = np.array([[-1.0], [1.0], [0]])

# Instantiate Robotarium object
N = 1

sim = robotarium.Robotarium(number_of_agents=N, show_figure=True, save_data=True, update_time=1)
obstacles, goal = traj_utils.draw_maze(sim, (init_pt[0][0], init_pt[1][0]), 100)
solution = search((init_pt[0][0], init_pt[1][0]), obstacles, goal)
print(solution)

# Create barrier certificates to avoid collision
si_barrier_cert = create_single_integrator_barrier_certificate(N)

# define x initially
x = sim.get_poses()
#print(x)
sim.step()

for coordinate in solution:
    print(coordinate)
    goal_points = traj_utils.tuple_to_pt(coordinate)
    # While the number of robots at the required poses is less
    # than N...
    while(np.size(at_pose(x, goal_points, rotation_error=5)) != N):

        # Get poses of agents
        x = sim.get_poses()
        x_si = x[:2, :]

        # Create single-integrator control inputs
        dxi = single_integrator_position_controller(x_si, goal_points[:2, :], magnitude_limit=0.08)

        # Create safe control inputs (i.e., no collisions)
        dxi = si_barrier_cert(dxi, x_si)

        # Set the velocities by mapping the single-integrator inputs to unciycle inputs
        sim.set_velocities(np.arange(N), single_integrator_to_unicycle2(dxi, x))
        # Iterate the simulation
        sim.step()
    time.sleep(1)
# Always call this function at the end of your scripts!  It will accelerate the
# execution of your experiment
sim.call_at_scripts_end()
