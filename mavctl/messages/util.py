import time
import math
from pymavlink import mavutil



def distance_to_target(target1: tuple, target2: tuple) -> tuple:
    """
    Returns the distance vector to the target
    Target 1: Current Position of Vehicle 
    Target 2: Target Position 
    
    """

    dx = target2[0] - target1[0]
    dy = target2[1] - target1[1]
    dz = target2[2] - target1[2]

    return (dx, dy , dz)

def check_target_reached(target1, target2, tolerance) -> bool:
    """
    Checks if the vehicle has reached the target to a certian level of precision
    target1: Current Position of the vehicle (tuple)
    target2: Target Position (tuple)
    tolerance: Tolerance coefficent 
    """
    delta = distance_to_target(target1, target2)

    target_length = math.sqrt(target1[0] ** 2 + target1[1] ** 2 + target1[2] ** 2)  
 
    delta_length = math.sqrt(delta[0] ** 2 + delta[1] ** 2 + delta[2] ** 2)  

    if delta_length < target_length * tolerance:
        return True
    else:
        return False


