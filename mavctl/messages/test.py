import time 
import messages.util



# This tests the wait target reached and has been verified to work

def simple_goto(master, x, y, z): 
    type_mask = master.generate_typemask([0, 1, 2])
    print("Moving")
    master.set_position_local_ned(type_mask = type_mask, x = x, y = y, z = -z)
    master.wait_target_reached((x, y, -z))


