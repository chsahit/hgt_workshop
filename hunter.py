import rps.robotarium as robotarium
import rps.utilities.graph as graph
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *
from set_pose import setpos

import numpy as np
import time


#print(goal_points)

# Instantiate Robotarium object
N = 2

r = robotarium.Robotarium(number_of_agents=N, show_figure=True, save_data=True, update_time=0.01)


# Create barrier certificates to avoid collision
si_barrier_cert = create_single_integrator_barrier_certificate(N)

# define x initially
x = r.get_poses()
#print(x)
r.step()

# While the number of robots at the required poses is less
# than N...
robot1_goal = np.array([[0.0], [0.0], [0.0]])
robot2_goal = np.array([[-0.9], [0.9], [0.0]])
goal_points = np.concatenate((robot1_goal, robot2_goal), axis=1)

while(np.size(at_pose(x, goal_points, rotation_error=5)) != N):

    # Get poses of agents
    x = r.get_poses()
    x_si = x[:2, :]

    # Create single-integrator control inputs
    dxi = single_integrator_position_controller(x_si, goal_points[:2, :], magnitude_limit=0.08)

    # Create safe control inputs (i.e., no collisions)
    dxi = si_barrier_cert(dxi, x_si)

    # Set the velocities by mapping the single-integrator inputs to unciycle inputs
    r.set_velocities(np.arange(N), single_integrator_to_unicycle2(dxi, x))
    # Iterate the simulation
    r.step()
start_time = time.time()
robot1_vel = np.random.uniform(-0.1, 0.1, size=(3, 1))
iterations = 0
while time.time() - start_time < 20:

    # Get poses of agents
    x = r.get_poses()
    x_si = x[:2, :]
    if (iterations % 200 == 0):
        robot1_vel = np.random.uniform(-0.1, 0.1, size=(3, 1))
        robot1_vel[2, 0] = 0.0
    robot2_vel = 0.3 * np.array([[x[0, 0] - x[0, 1]], [x[1, 0] - x[1, 1]], [0]])
    dxi = np.concatenate((robot1_vel, robot2_vel), axis=1)
    #print(dxi)

    r.set_velocities(np.arange(N), single_integrator_to_unicycle2(dxi, x))
    time.sleep(0.01)
    # Iterate the simulation
    iterations += 1
    r.step()

# Always call this function at the end of your scripts!  It will accelerate the
# execution of your experiment
r.call_at_scripts_end()
