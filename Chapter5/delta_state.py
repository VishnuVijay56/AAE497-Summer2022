"""
delta_state.py: class file for state of MAV deltas (aileron, rudder, elevator, throttle)
    - Author: Vishnu Vijay
    - Created: 6/18/22
    - History:
        - 
"""

class Delta_State:
    def __init__(self):
        # Aileron
        self.aileron_deflection = 0
        
        # Elevator
        self.elevator_deflection = 0

        # Rudder
        self.rudder_deflection = 0

        # Throttle
        self.throttle_level = 0.5

    
    def print(self):
        rounding_digits = 4
        print('elevator =', round(self.elevator_deflection, rounding_digits),
              '; aileron =', round(self.aileron_deflection, rounding_digits),
              '; rudder =', round(self.rudder_deflection, rounding_digits),
              '; throttle =', round(self.throttle_level, rounding_digits))