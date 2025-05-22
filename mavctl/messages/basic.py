from pymavlink import mavutil
import time

class Basic:

    def __init__(self, mav):
        self.mav = mav


    def arm(self):
        # NOTE: UNDER NO CIRCUMSTANCE SHOULD THIS BE USED IN A REAL FLIGHT SCRIPT
        # NOTE: ONLY FOR USE IN SIMULATED SCRIPTS

        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0
        )    
        print("MAVCTL: Waiting for vehicle to arm")
        self.mav.motors_armed_wait()
        print("MAVCTL: Armed!")
    
    def disarm(self):
        # NOTE: UNDER NO CIRCUMSTANCE SHOULD THIS BE USED IN A REAL FLIGHT SCRIPT
        # NOTE: ONLY FOR USE IN SIMULATED SCRIPTS

        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0
        )    
        print("MAVCTL: Waiting for vehicle to disarm")
        self.mav.motors_disarmed_wait()
        print("MAVCTL: Disarmed!")
 


