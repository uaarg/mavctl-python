import time 
from messages import util


# This tests the wait target reached and has been verified to work

def simple_goto_local(master, x, y, z): 
    type_mask = master.generate_typemask([0, 1, 2])
    print("Moving")
    master.set_position_local_ned(type_mask = type_mask, x = x, y = y, z = -z)
    master.wait_target_reached((x, y, -z))

def simple_goto_global(master, lat, lon, alt):
    type_mask = master.generate_typemask([0, 1, 2])
    print("Moving")
    master.set_position_global(type_mask = type_mask, lon = lon, lat = lat, alt = alt)
    origin = master.get_global_origin()
    print(origin)
    pos_local = util.LatLon_to_XY((lat, lon, alt), origin) 
    master.wait_target_reached(pos_local, tolerance=0.1)

