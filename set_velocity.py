import rps.robotarium as robotarium
import rps.utilities.graph as graph
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *
from set_pose import setpos
import numpy as np
import time

#construct robotarium simulator
N = 1
sim = robotarium.Robotarium(number_of_agents=N, show_figure=True, save_data=True, update_time=0.1)

#get the initial position of the robots
x = sim.get_poses()
sim.step()

#get the robot onto the screen (we'll come back to this later)
goal_point = np.array([[0.0], [0.0], [0.0]])
si_barrier_cert = create_single_integrator_barrier_certificate(N)
setpos(sim, x, goal_point, si_barrier_cert, N)


for i in range(1000):
    #get new robot poses
    x = sim.get_poses()
    x_si = x[:2,:]

    velocity = np.array([[0.1], [0.1], [0.0]])
    sim.set_velocities(np.arange(N), single_integrator_to_unicycle2(velocity, x))

    sim.step()
    time.sleep(0.1)
