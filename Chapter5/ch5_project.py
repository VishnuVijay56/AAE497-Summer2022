"""
ch5_project.py: implementing project
    - Author: Vishnu Vijay
    - Created: 6/25/22
"""

import time
import numpy as np
from compute_models import compute_model

from mav import MAV
from mav_state import MAV_State
from mav_dynamics import MAV_Dynamics
from trim import compute_trim
from wind_simulation import WindSimulation
from delta_state import Delta_State

# Create instance of MAV_Dynamics
Ts = 0.01
mav_dynamics = MAV_Dynamics(time_step=Ts) # time step in seconds
mav_state = mav_dynamics.mav_state
# Create instance of MAV object using MAV_State object
fullscreen = False
this_mav = MAV(mav_state, fullscreen)
# Create instance of wind simulation
wind_sim = WindSimulation(Ts)

# Find trim state
Va = 25
gamma_deg = 0
gamma_rad = gamma_deg * np.pi/180
trim_state, trim_input = compute_trim(mav_dynamics, Va, gamma_rad)
mav_dynamics.internal_state = trim_state
delta = trim_input

# Compute state space model linearized about trim
compute_model(mav_dynamics, trim_state, trim_input)

# Run Simulation
curr_time = 0
end_time = 5 # seconds
view_sim = True
sim_real_time = False

while (curr_time < end_time) and (view_sim):
    step_start = time.time()
    # wind sim
    #wind_steady_gust = wind_sim.update()
    wind_steady_gust = np.zeros((6, 1))

    # Update MAV dynamic state
    mav_dynamics.iterate(delta, wind_steady_gust)

    # Update MAV mesh for viewing
    this_mav.set_mav_state(mav_dynamics.mav_state)
    this_mav.update_mav_state()
    this_mav.update_render()
    
    # DEBUGGING - Print Vehicle's state
    #print("Time: " + str(round(curr_time, 2)) + " ", end="  ")
    #mav_dynamics.mav_state.print()

    # Update time
    step_end = time.time()
    if (((step_end - step_start) < mav_dynamics.time_step) and (sim_real_time)):
        time.sleep(step_end - step_start)
    curr_time += mav_dynamics.time_step
    