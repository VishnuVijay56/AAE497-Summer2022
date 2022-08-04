"""
mav_body_parameters.py: collection of the MAV's body parameters
    - Author: Vishnu Vijay
    - Created: 6/9/22

"""

import numpy as np
import helper as help

#####
## Initial Conditions
#####
# Initial conditions for MAV
north0 = 0.  # initial north position
east0 = 0.  # initial east position
down0 = 0.  # initial down position
u0 = 25.  # initial velocity along body x-axis
v0 = 0.  # initial velocity along body y-axis
w0 = 0.  # initial velocity along body z-axis
phi0 = 0.  # initial roll angle
theta0 = 0.  # initial pitch angle
psi0 = 0.0  # initial yaw angle
p0 = 0  # initial roll rate
q0 = 0  # initial pitch rate
r0 = 0  # initial yaw rate
Va0 = np.sqrt(u0**2 + v0**2 + w0**2)
# Quaternion State
e = help.EulerToQuaternion(phi0, theta0, psi0)
e0 = e.item(0)
e1 = e.item(1)
e2 = e.item(2)
e3 = e.item(3)


#####
## Physical Parameters
#####
m = 11 # kg
Jx = 0.8244 #kg m^2
Jy = 1.135 #kg m^2
Jz = 1.759 #kg m^2
Jxz = 0.1204 #kg m^2
gravity = 9.81 #m/s^2


#####
## Calculation Variables
#####
# gamma parameters pulled from page 36 (dynamics)
gamma = Jx * Jz - (Jxz**2)
gamma1 = (Jxz * (Jx - Jy + Jz)) / gamma
gamma2 = (Jz * (Jz - Jy) + (Jxz**2)) / gamma
gamma3 = Jz / gamma
gamma4 = Jxz / gamma
gamma5 = (Jz - Jx) / Jy
gamma6 = Jxz / Jy
gamma7 = ((Jx - Jy) * Jx + (Jxz**2)) / gamma
gamma8 = Jx / gamma