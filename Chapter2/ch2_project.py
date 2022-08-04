"""
ch2_project.py: implementing project
    - Author: Vishnu Vijay
    - Created: 6/1/22
"""

from mav import MAV
from mav_state import MAV_State

# Create instance of MAV_State object with default values (all 0)
state = MAV_State()

# Create instance of MAV object using MAV_State object
this_mav = MAV(state)

# Display the MAV
this_mav.display_mav()