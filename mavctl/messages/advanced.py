import time
from typing import Literal 
from math import radians, atan

from mavctl.messages.Navigator import LandingTarget, Navigator
from messages import util
from messages.location import LocationGlobal, LocationGlobalRelative, LocationLocal
from messages.util import Heading, LatLon_to_Distance

# This file is meant for implementing more advanced features in mavctl

# This tests the wait target reached and has been verified to work

def simple_goto_local(master, x, y, z): 
    """Mimics simple_goto from dronekit, 
    includes the right yaw angle so that 
    the drone faces where it needs to go."""

    type_mask = master.generate_typemask([0, 1, 2, 9])
    angle = atan(y/x) 
    master.set_position_local_ned(type_mask = type_mask, x = x, y = y, z = -z, yaw = angle)
    master.wait_target_reached(LocationLocal(x, y, -z))

def simple_goto_global(master, lat, lon, alt):
    type_mask = master.generate_typemask([0, 1, 2, 9])
    start_point = master.get_global_position()
    heading = Heading(start_point, LocationGlobal(lat, lon, alt))
    master.set_position_global(type_mask = type_mask, lon = lon, lat = lat, alt = alt, yaw = 0)
    print("Sent set position message")
    origin = master.get_global_origin()
    print("Waiting for drone to reach target")
    master.wait_target_reached_global(LocationGlobal(lat, lon, alt))
    print("reached target")

def do_precision_landing(master: Navigator,
                         imaging_analysis_delegate,
                         mode: Literal["REQUIRED", "OPPORTUNISTIC"]) -> None:
    """
    This function sets the drone into precision landing mode.

    Parameters:
        imaging_analysis_delegate: an object to the ImageAnalysisDelegate class

        mode (str): Either "REQUIRED" or "OPPORTUNISTIC".

            REQUIRED:
                The vehicle searches for a target if none is visible when
                landing is initiated, and performs precision landing if found.

            OPPORTUNISTIC:
                The vehicle uses precision landing only if the target is visible
                at landing initiation; otherwise it performs a normal landing.
    """
    def callback(_, coords):
        altitude = master.get_rel_altitude()
        target = LandingTarget(x=coords[0], y=coords[1], z=altitude)
        master.broadcast_landing_target(target)

    imaging_analysis_delegate.subscribe(callback)

    if mode == "OPPORTUNISTIC":
        master.land(land_mode=1)
    elif mode == "REQUIRED":
        master.land(land_mode=2)
