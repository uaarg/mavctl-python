import time 
from messages import util
from messages.location import LocationGlobal, LocationGlobalRelative, LocationLocal
from math import radians, atan

# This tests the wait target reached and has been verified to work

def simple_goto_local(master, x, y, z): 

    # Mimics simple_goto from dronekit, includes the right yaw angle so that the drone faces where it needs to go.

    type_mask = master.generate_typemask([0, 1, 2, 9])
    print("Moving")
    angle = atan(y/x) 
    master.set_position_local_ned(type_mask = type_mask, x = x, y = y, z = -z, yaw = angle)
    master.wait_target_reached(LocationLocal(x, y, z))

def simple_goto_global(master, lat, lon, alt):
    type_mask = master.generate_typemask([0, 1, 2, 9])
    print("Moving")
    master.set_position_global(type_mask = type_mask, lon = lon, lat = lat, alt = alt, yaw = radians(lon + lat))
    origin = master.get_global_origin()
    print(origin)
    pos_local = util.LatLon_to_XY(LocationGlobal(lat, lon, alt), origin) 
    master.wait_target_reached(pos_local, tolerance=0.1)

