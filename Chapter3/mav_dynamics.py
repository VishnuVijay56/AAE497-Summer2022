"""
mav_dynamics.py: class file for dynamics of mav
    - Author: Vishnu Vijay
    - Created: 6/9/22

"""

import numpy as np
from helper import QuaternionToEuler, QuaternionToRotationMatrix

import mav_body_parameter as MAV_para
from mav_state import MAV_State

class MAV_Dynamics:
    ###
    # Constructor!
    # Initializes the MAVDynamics object, sets up internal and external state variables
    # internal state (internal_state) used for integration and EOMs
    # external state (mav_state) to be accessed by other files
    # Inputs:
    #   - time_step: time step to be used by simulation
    ###
    def __init__(self, time_step):
        self.time_step = time_step
        
        # Initialize internal state
        # pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r
        self.internal_state = np.empty((13, 1))
        self.internal_state[0] = MAV_para.north0
        self.internal_state[1] = MAV_para.east0
        self.internal_state[2] = MAV_para.down0
        self.internal_state[3] = MAV_para.u0
        self.internal_state[4] = MAV_para.v0
        self.internal_state[5] = MAV_para.w0
        self.internal_state[6] = MAV_para.e0
        self.internal_state[7] = MAV_para.e1
        self.internal_state[8] = MAV_para.e2
        self.internal_state[9] = MAV_para.e3
        self.internal_state[10] = MAV_para.p0
        self.internal_state[11] = MAV_para.q0
        self.internal_state[12] = MAV_para.r0

        self.mav_state = MAV_State()
        self.update_external_state()
        
    
    ###
    # Iterates a singular time step, according to time_step, and calculates the new
    # state values of the MAV
    # Updates the internal and external MAV states
    # Inputs:
    #   - applied_forces_moments: array containing all the forces and moments applied to the MAV
    # Outputs: 
    # - N/A
    ###
    def iterate(self, applied_forces_moments):
        # Integrate ODE using RK4 algorithm
        x1 = self.derivatives(self.internal_state, applied_forces_moments)
        x2 = self.derivatives(self.internal_state + self.time_step/2.*x1, applied_forces_moments)
        x3 = self.derivatives(self.internal_state + self.time_step/2.*x2, applied_forces_moments)
        x4 = self.derivatives(self.internal_state + self.time_step*x3, applied_forces_moments)
        self.internal_state += self.time_step/6 *(x1 + 2*x2 + 2*x3 + x4)

        # Normalize quaternion
        e0 = self.internal_state.item(6)
        e1 = self.internal_state.item(7)
        e2 = self.internal_state.item(8)
        e3 = self.internal_state.item(9)
        quaternion_mag = np.sqrt(e0**2 + e1**2 + e2**2 + e3**2)
        self.internal_state[6][0] = e0 / quaternion_mag
        self.internal_state[7][0] = e1 / quaternion_mag
        self.internal_state[8][0] = e2 / quaternion_mag
        self.internal_state[9][0] = e3 / quaternion_mag

        # Update external state
        self.update_external_state()


    ###
    # Iterates a singular time step, according to time_step, and calculates the new
    # state values of the MAV
    # Inputs:
    #   - curr_state: internal state of the MAV
    #   - applied_forces_moments: array containing all the forces and moments applied to the MAV
    # Outputs:
    #   - array of derivative of all (internal) state variables
    ###
    def derivatives(self, int_state, applied_forces_moments):
        ## Extract the values of int_state and applied_forces_moments
        # int_state
        pn = int_state.item(0)
        pe = int_state.item(1)
        pd = int_state.item(2)
        u = int_state.item(3)
        v = int_state.item(4)
        w = int_state.item(5)
        e0 = int_state.item(6)
        e1 = int_state.item(7)
        e2 = int_state.item(8)
        e3 = int_state.item(9)
        p = int_state.item(10)
        q = int_state.item(11)
        r = int_state.item(12)
        # applied_forces_moments
        fx = applied_forces_moments.item(0)
        fy = applied_forces_moments.item(1)
        fz = applied_forces_moments.item(2)
        l = applied_forces_moments.item(3)
        m = applied_forces_moments.item(4)
        n = applied_forces_moments.item(5)


        ## Find derivatives
        # position kinematics
        pos_dot = QuaternionToRotationMatrix(np.array([e0, e1, e2, e3])) @ np.array([u, v, w]).T
        north_dot = pos_dot.item(0)
        east_dot = pos_dot.item(1)
        down_dot = pos_dot.item(2)

        # position dynamics
        u_dot = (r*v - q*w) + (fx / MAV_para.m)
        v_dot = (p*w - r*u) + (fy / MAV_para.m)
        w_dot = (q*u - p*v) + (fz / MAV_para.m)

        # rotational kinematics
        e0_dot = 0.5*(0*e0 - p*e1 - q*e2 - r*e3)
        e1_dot = 0.5*(p*e0 + 0*e1 + r*e2 - q*e3)
        e2_dot = 0.5*(q*e0 - r*e1 + 0*e2 + p*e3) 
        e3_dot = 0.5*(r*e0 + q*e1 - p*e2 + 0*e3)

        # rotatonal dynamics
        p_dot = MAV_para.gamma1*p*q - MAV_para.gamma2*q*r + MAV_para.gamma3*l + MAV_para.gamma4*n
        q_dot = MAV_para.gamma5*p*r - MAV_para.gamma6*(p**2 - r**2) + m / MAV_para.Jy
        r_dot = MAV_para.gamma7*p*q - MAV_para.gamma1*q*r + MAV_para.gamma4*l + MAV_para.gamma8*n

        # collect the derivative of the states
        x_dot = np.array([[north_dot, east_dot, down_dot, u_dot, v_dot, w_dot,
                           e0_dot, e1_dot, e2_dot, e3_dot, p_dot, q_dot, r_dot]]).T
        return x_dot


    ###
    # Updates the external MAV state based on the new internal state of the MAV
    # Inputs:
    #   - N/A
    # Outputs:
    #   - N/A
    ###
    def update_external_state(self):
        # quaternion to euler angular positions
        phi, theta, psi = QuaternionToEuler(self.internal_state[6:10])
        # inertial positions
        self.mav_state.north = self.internal_state.item(0)
        self.mav_state.east = self.internal_state.item(1)
        self.mav_state.altitude = self.internal_state.item(2)
        # euler angular positions
        self.mav_state.phi = phi
        self.mav_state.theta = theta
        self.mav_state.psi = psi
        # rate of change of angular positions
        self.mav_state.p = self.internal_state.item(10)
        self.mav_state.q = self.internal_state.item(11)
        self.mav_state.r = self.internal_state.item(12)
