import rps.robotarium as robotarium
import rps.utilities.graph as graph
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *

import numpy as np

def setpos(sim, x, goal_points, cert, N):
    while(np.size(at_pose(x, goal_points, rotation_error=5)) != N):
        # Get poses of agents
        x = sim.get_poses()
        x_si = x[:2, :]

        ## Create single-integrator control inputs
        #dxi = single_integrator_position_controller(x_si, goal_points[:2, :], magnitude_limit=0.08)

        dxu = unicycle_pose_controller(x, goal_points)

        # Create safe control inputs (i.e., no collisions)
        dxu = cert(dxu, x)
        #dxi = cert(dxi, x_si)

        # Set the velocities by mapping the single-integrator inputs to unciycle inputs
        #sim.set_velocities(np.arange(N), single_integrator_to_unicycle2(dxi, x))
        sim.set_velocities(np.arange(N), dxu)
        # Iterate the simulation
        print(x)
        sim.step()

