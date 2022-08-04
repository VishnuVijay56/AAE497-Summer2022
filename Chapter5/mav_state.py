"""
mav_state.py: class file for mav state
    - Author: Vishnu Vijay
    - Created: 6/2/22
    - History:
        - 6/7: Adding functionality for chapter 3
        - 6/16: Adding functionality for chapter 4
"""

class MAV_State:
    def __init__(self):
        # Inertial Position
        self.north = 0
        self.east = 0
        self.altitude = 0
        
        # Angular Positions
        self.phi = 0 # roll in radians
        self.theta = 0 # pitch in radians
        self.psi = 0 # heading in radians

        # Rate of Change of Angular Positions
        self.p = 0 # roll rate in rad/s
        self.q = 0 # pitch rate in rad/s
        self.r = 0 # heading rate in rad/s

    
    def print(self):
        print("North: {}; East: {}; Alt: {}".format(self.north, self.east, self.altitude))
        print("\tPhi: {}; Theta: {}; Psi: {}".format(self.phi, self.theta, self.psi))
        print("\tP: {}; Q: {}; R: {}".format(self.p, self.q, self.r))