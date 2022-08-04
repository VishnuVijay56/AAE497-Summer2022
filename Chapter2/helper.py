"""
helper.py: collection of helper functions for chapter 2 project
    - Author: Vishnu Vijay
    - Created: 6/1/22
"""

import numpy as np

###
# Returns rotation matrx based upon 3-2-1 Euler
# angle rotation sequence
# Inputs:
#   - phi: roll angle
#   - theta: pitch angle
#   - psi: heading (from north)
###
def EulerRotationMatrix(phi, theta, psi):
    # Rotation matrix from vehicle frame to vehicle-1 frame
    c_psi = np.cos(psi)
    s_psi = np.sin(psi)
    R_v_v1 = np.array([[c_psi, s_psi, 0], [-s_psi, c_psi, 0], [0, 0, 1]])

    # Rotation matrix from vehicle-1 frame to vehicle-2 frame
    c_the = np.cos(theta)
    s_the = np.sin(theta)
    R_v1_v2 = np.array([[c_the, 0, -s_the], [0, 1, 0], [s_the, 0, c_the]])

    # Rotation matrix from vehicle-2 frame to body frame
    c_phi = np.cos(phi)
    s_phi = np.sin(phi)
    R_v2_b = np.array([[1, 0, 0], [0, c_phi, s_phi], [0, -s_phi, c_phi]])

    # Rotation matrix from vehicle frame to body frame
    R_v_b = R_v2_b @ R_v1_v2 @ R_v_v1

    return R_v_b

