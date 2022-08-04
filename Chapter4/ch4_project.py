"""
ch4_project.py: implementing project
    - Author: Vishnu Vijay
    - Created: 6/16/22
"""

import numpy as np

from mav import MAV
from mav_state import MAV_State
from mav_dynamics import MAV_Dynamics
from wind_simulation import WindSimulation
from delta_state import Delta_State

# Create instance of MAV_Dynamics
Ts = 0.01
mav_dynamics = MAV_Dynamics(time_step=Ts) # time step in seconds
mav_state = mav_dynamics.mav_state
# Create instance of MAV object using MAV_State object
this_mav = MAV(mav_state)
# Create instance of wind simulation
wind_sim = WindSimulation(Ts)
# MAV deltas
mav_delta = Delta_State()

# Run Simulation
curr_time = 0
end_time = 10 # seconds

while curr_time < end_time:
    # control surface + throttle input
    mav_delta.aileron_deflection = 0.0006654356888371589
    mav_delta.elevator_deflection = -0.1253725629655638
    mav_delta.rudder_deflection = -0.00010967738975935969
    mav_delta.throttle_level = 0.6126452247726315
    
    # wind sim
    wind_steady_gust = wind_sim.update()
    #print(wind_steady_gust)
    #print()

    # Update MAV dynamic state
    mav_dynamics.iterate(mav_delta, wind_steady_gust)
    print()

    # Update MAV mesh for viewing
    this_mav.set_mav_state(mav_dynamics.mav_state)
    this_mav.update_mav_state()
    this_mav.update_render()
    
    # Update time
    curr_time += mav_dynamics.time_step