"""
ch3_project.py: implementing project
    - Author: Vishnu Vijay
    - Created: 6/7/22
"""

import numpy as np

from mav import MAV
from mav_state import MAV_State
from mav_dynamics import MAV_Dynamics

# Create instance of MAV_Dynamics
mav_dynamics = MAV_Dynamics(time_step=0.01) # time step in seconds
mav_state = mav_dynamics.mav_state

# Create instance of MAV object using MAV_State object
this_mav = MAV(mav_state)

# Run Simulation
curr_time = 0
end_time = 15 # seconds

while curr_time < end_time:
    # applied forces and moments on MAV system
    fx = 0 #x_body (forward)
    fy = 0 #y_body (left)
    fz = 0 #z_body (up)
    Mx = 0. #roll (l)
    My = 0. #pitch (m)
    Mz = 0. #heading (n)
    applied_forces_moments = np.array([[fx, fy, fz, Mx, My, Mz]]).T

    # Update MAV dynamic state
    mav_dynamics.iterate(applied_forces_moments)

    # Update MAV mesh for viewing
    this_mav.set_mav_state(mav_dynamics.mav_state)
    this_mav.update_mav_state()
    this_mav.update_render()
    
    # Update time
    curr_time += mav_dynamics.time_step