import time 
from messages import util
from messages.location import LocationGlobal, LocationGlobalRelative, LocationLocal
from math import radians, atan
from messages.util import Heading, LatLon_to_Distance

# This file is meant for implementing more advanced features in mavctl

# This tests the wait target reached and has been verified to work

def simple_goto_local(master, x, y, z): 

    # Mimics simple_goto from dronekit, includes the right yaw angle so that the drone faces where it needs to go.

    type_mask = master.generate_typemask([0, 1, 2, 9])
    print("Moving")
    angle = atan(y/x) 
    master.set_position_local_ned(type_mask = type_mask, x = x, y = y, z = -z, yaw = angle)
    master.wait_target_reached(LocationLocal(x, y, -z))

def simple_goto_global(master, lat, lon, alt):
    type_mask = master.generate_typemask([0, 1, 2, 9])
    print("Moving")
    start_point = master.get_global_position()
    heading = Heading(start_point, LocationGlobal(lat, lon, alt))
    master.set_position_global(type_mask = type_mask, lon = lon, lat = lat, alt = alt, yaw = radians(heading))
    origin = master.get_global_origin()

    master.wait_target_reached_global(LocationGlobal(lat, lon, alt))

