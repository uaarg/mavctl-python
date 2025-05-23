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
        time.sleep(2)
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

    def send_status_message(self, message: str):
        self.mav.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE,
                                     message.encode())
        print("MAVCTL Message: " + message)

    def get_global_position(self):
        msg = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1000
        return (lat, lon, alt)


    def takeoff(self, altitude):
        print("MAVCTL: Taking Off to: " + str(altitude) + "m")
        pos = self.get_global_position()
        self.mav.mav.command_long_send(0,
                                       0,
                                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                        0,
                                        0,
                                        0,
                                        0,
                                        0,
                                        0,
                                        0,
                                        altitude)

    def return_to_launch(self):
        print("MAVCTL: RTL")
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0
        )
        print("MAVCTL: RTL COMPLETE")

    def set_position_local_ned(self,
                               coordinate_frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                               type_mask=0x07FF,
                               x = 0,
                               y = 0,
                               z = 0,
                               vx = 0,
                               vy = 0,
                               vz = 0,
                               afx = 0,
                               afy = 0, 
                               afz = 0,
                               yaw = 0,
                               yaw_rate = 0):
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_SET_POSITION_TARGET_LOCAL_NED,
            coordinate_frame,
            type_mask,
            x,
            y,
            z,
            vx,
            vy,
            vz,
            afx,
            afy,
            afz,
            yaw,
            yaw_rate
        )
                                               

