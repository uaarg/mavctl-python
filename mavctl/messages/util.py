import math
from math import radians, cos
from pymavlink import mavutil
from messages.location import LocationGlobal, LocationGlobalRelative, LocationLocal


def distance_to_target(target1: tuple, target2: tuple) -> tuple:
    """
    Returns the distance vector to the target
    Target 1: Current Position of Vehicle 
    Target 2: Target Position 
    
    """

    dx = target2.north - target1.north
    dy = target2.east - target1.east
    dz = target2.down - target1.down

    return LocationLocal(dx, dy, dz)

def check_target_reached(target1, target2, tolerance) -> bool:
    """
    Checks if the vehicle has reached the target to a certian level of precision
    target1: Current Position of the vehicle (tuple)
    target2: Target Position (tuple)
    tolerance: Tolerance coefficent 
    """
    delta = distance_to_target(target1, target2)

    target_length = math.sqrt(target1.north ** 2 + target1.east ** 2 + target1.down ** 2)  
 
    delta_length = math.sqrt(delta.north ** 2 + delta.east ** 2 + delta.down ** 2)  

    if delta_length < target_length * tolerance:
        return True
    else:
        return False

def LatLon_to_XY(point, origin):

    R = 6378137 # Radius of the Earth in meters

    origin_lat = origin.lat
    origin_lon = origin.lon
    point_lat = point.lat
    point_lon = point.lon

    origin_lon_rad = radians(origin_lon)
    origin_lat_rad = radians(origin_lat)
    point_lon_rad = radians(point_lon)
    point_lat_rad = radians(point_lat)

    delta_lon = point_lon_rad - origin_lon_rad
    delta_lat = point_lat_rad - origin_lat_rad

    alt = point.alt - origin.alt

    north = delta_lat * R
    east = delta_lon * cos((origin_lat_rad + point_lat_rad) / 2) * R
    print(origin)
    print(point)
    
    print(north, east, alt)
    return LocationLocal(north, east, alt)
