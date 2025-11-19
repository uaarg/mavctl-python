from typing import Literal, Optional, Tuple
from dataclasses import dataclass
import time
from math import sqrt

from pymavlink import mavutil

from src.modules.mavctl.mavctl.messages import util
from src.modules.mavctl.mavctl.messages.location import LocationGlobal, LocationGlobalRelative, LocationLocal, Velocity, Altitude

@dataclass
class LandingTarget:
    """
    Data class to store position of landing target in MAV_FRAME_BODY_FRD frame.
    FRD local frame aligned to the vehicle's attitude (x: Forward, y: Right, z: Down)
    with an origin that travels with vehicle.
    """
    forward: float
    right: float
    altitude: float

class Navigator:

    def __init__(self, mav):
        self.mav = mav
        

        self.TOLERANCE_CE = 0.05
        # For checking if the target position has been reached. This is a coefficient which is multiplied by the distance travelled.
        # The reason why a coefficient method was chosen is because the position tolerance should be a function of distance as opposed to being a constant
        # This method is preferred so that what happened at AEAC 2025 doesnt happen again.

    def arm(self):
        """
        Arms the drone.
        NOTE: UNDER NO CIRCUMSTANCE SHOULD THIS BE USED IN A REAL FLIGHT SCRIPT
        NOTE: ONLY FOR USE IN SIMULATED SCRIPTS
        """

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
   
    def wait_vehicle_armed(self):
        """
        Waits for the vehicle to be armed. See samples directory for examples
        """
        print("MAVCTL: Waiting for vehicle to arm")
        self.mav.motors_armed_wait()
        print("Armed!")

    def disarm(self):
        """
        Disarms the drone.
        NOTE: UNDER NO CIRCUMSTANCE SHOULD THIS BE USED IN A REAL FLIGHT SCRIPT
        NOTE: ONLY FOR USE IN SIMULATED SCRIPTS
        """

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

    def set_mode_wait(self, mode = "GUIDED", timeout = None) -> bool:
        """
        Waits for the specified mode to be selected.
        Timesout after specified timeout

        mode: Mode you want to wait for, leave blank for the most part
        timeout: Timeout in seconds, leave blank to have no timeout
        """ 
        start_time = time.time()
        
        mode_mapping = self.mav.mode_mapping()
        if mode not in mode_mapping:
            raise ValueError("MAVCTL Error: Mode " + mode + "not recognized")

        mode_id = mode_mapping[mode]
        print("MAVCTL: Waiting for " + mode + " mode")
        while True:
            if timeout:
                if time.time() - start_time >= timeout: 
                   print("MAVCTL: Timeout waiting for " + mode + " mode")
                   return False 

            self.mav.mav.request_data_stream_send(
                    self.mav.target_system,
                    self.mav.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL,
                    1,
                    1)

            msg = self.mav.recv_match(type = "HEARTBEAT", blocking = True, timeout = 0.25)

            if msg:
                current_mode_id = msg.custom_mode
                print(current_mode_id, mode_id)
                if current_mode_id == mode_id:
                    print("MAVCTL: Set to " + mode + " mode")
                    return True
                    
    def get_global_position(self):
        """
        Gets Global Position of drone.
        returns lat, lon, and altitude with respect to GLOBAL coordinates
        Altitude is a value which is measured from sea level and is positive

        """
        msg = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1000
         
        return LocationGlobal(lat, lon, alt)
    
    def get_geovector(point):
        origin = self.get_global_position() 
        distance = util.LatLon_to_Distance(point, origin)
        bearing = radians(util.Heading(point, origin))

        vectorX = distance * sin(bearing)
        vectorY = distance * cos(bearing)

        xRot = vectorX * cos(heading) - vectorY * sin(heading)
        yRot = vectorx * sin(heading) + vectorY * cos(heading)
        
        return (xRot, yRot)

    def get_heading(self):
        msg = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            hdg = hdg / 100
        return hdg
  

    def get_local_position(self):
        """
        Gets Local Position of drone.
        returns x, y, z in NED coordinates with respect to local origin
        DOWN is positive
        """
        
        msg = self.mav.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        if msg:
            north = msg.x
            east = msg.y
            down = msg.z

        return LocationLocal(north, east, down)

    def get_global_origin(self):
        """
        Gets the local origin of the drone (local position [0, 0, 0]) in terms of lat, lon and alt
        """
        self.mav.mav.command_long_send(
                                    self.mav.target_system,
                                    self.mav.target_component,
                                    mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                    0,
                                    242,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0)
 
        msg = self.mav.recv_match(type='HOME_POSITION', blocking=True)
        print(msg)
        if msg:
            lat = msg.latitude / 1e7
            lon = msg.longitude / 1e7
            alt = msg.altitude / 1000

        return LocationGlobal(lat, lon, alt)

    def get_velocity(self):
        """
        Gets velocity of drone in NED coordinates
        """
        msg = self.mav.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        if msg:
            north = msg.vx
            east = msg.vy
            down = msg.vz

        return Velocity(north, east, down)

    def get_altitude(self):
        """
        Gets the current system altitude, of various types
        """
        msg = self.mav.recv_match(type='ALTITUDE', blocking=True)
        if msg:
            mono = msg.altitude_monotonic
            amsl = msg.altiude_amsl
            local = msg.altitude_local
            relative = msg.altitude_relative
            terrain = msg.altitude_terrain
            clearance = msg.bottom_clearance

        return Altitude(mono, amsl, local, relative, terrain, clearance)


    def takeoff(self, altitude, pitch=15):
        """
        Initiates a takeoff the drone to target altiude

        Altitude: Altituide to take off to in meters

        NOTE: Positive is upwards
        """

        print("MAVCTL: Taking Off to: " + str(altitude) + "m")
        self.mav.mav.command_long_send(
                                        self.mav.target_system,
                                        self.mav.target_component,
                                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                        0,
                                        pitch,
                                        0,
                                        0,
                                        0,
                                        0,
                                        0,
                                        altitude)

        self.wait_target_reached(LocationLocal(0, 0, -altitude))

    def vtol_takeoff(self, altitude, pitch=15):
        """
        Initiates a takeoff the drone to target altiude

        Altitude: Altituide to take off to in meters

        NOTE: Positive is upwards
        """

        print("MAVCTL: Taking Off to: " + str(altitude) + "m")
        self.mav.mav.command_long_send(
                                        self.mav.target_system,
                                        self.mav.target_component,
                                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                        0,
                                        pitch,
                                        0,
                                        0,
                                        0,
                                        0,
                                        0,
                                        altitude)

        self.wait_target_reached(LocationLocal(0, 0, -altitude))

    def land(self,
             land_mode: Literal[0,1,2],
             abort_alt: float = 0,
             yaw_angle: Optional[float] = 0,
             latitude: Optional[float] = 0,
             longitude: Optional[float] = 0,
             ):
        """
        Initiate a landing sequence.

        Parameters:
            abort_alt (float):
                Minimum target altitude if landing is aborted
                use system default if not specified. Units: meters.

            land_mode (0, 1, 2):
                Precision land mode:
                    0: Precision land disabled
                    1: opportunistic
                    2: required 

            yaw_angle (float, optional):
                Desired yaw angle in degrees.
                Default is to follow the current system yaw heading mode

            latitude (float, optional):
                Latitude in decimal degrees.

            longitude (float, optional):
                Longitude in decimal degrees.
        """

        # Runtime validation
        if land_mode == 0 and (latitude is None or longitude is None): 
            raise ValueError("specify latitude and longitude for disabled precision landing") 
        self.mav.mav.command_long_send(
                                        self.mav.target_system,
                                        self.mav.target_component,
                                        mavutil.mavlink.MAV_CMD_NAV_LAND,
                                        0,
                                        abort_alt,
                                        land_mode,
                                        0,
                                        yaw_angle,
                                        latitude,
                                        longitude,
                                        0)
        
        current_position = self.get_local_position()
        current_position.alt = 0
        target = (current_position.north, current_position.down, 0)

        self.wait_target_reached(current_position)


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

    def generate_typemask(self, keeps):
        # Generates typemask based on which values to be included
        mask = 0
        for bit in keeps:
            mask |= (0 << bit)
        return mask


    def set_position_local_ned(self,
                               coordinate_frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                               type_mask=0x07FF,
                               x=0,
                               y=0,
                               z=0,
                               vx=0,
                               vy=0,
                               vz=0,
                               afx=0,
                               afy=0, 
                               afz=0,
                               yaw=0,
                               yaw_rate=0):
        """
            Set position in local/relative coordinates. Can also set velocity/acceleration setpoints
            x, y, z is in meters in NED coordinates (down is positive)
        """    
        
        self.mav.mav.set_position_target_local_ned_send(
                                                        0,
                                                        self.mav.target_system,
                                                        self.mav.target_component,
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
                                                        yaw_rate)
   

    def set_speed(self, speed):
    
    
        #Adjusts the global speed parameter.
        #(WIP) DO NOT USE IN REAL FLIGHT, THIS METHOD HAS NOT BEEN VERIFIED YET.


        self.mav.mav.param_set_send(
                self.mav.target_system,
                self.mav.target_component,
                b'WPNAV_SPEED', 
                speed*100, # Speed in m/s is converted to cm/s
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    

    def set_position_global(self,
                            coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                            type_mask=0x07FF,
                            lat=0,
                            lon=0,
                            alt=0,
                            vx=0,
                            vy=0,
                            vz=0,
                            afx=0,
                            afy=0,
                            afz=0,
                            yaw=0,
                            yaw_rate=0):
        """
            Set position in global coordinates. Can also set velocity/acceleration setpoints
            lat, lon, and altitude. Altitude depends on the coordinate frame chosen
        """    
        

        self.mav.mav.set_position_target_global_int_send(0,
                                                        self.mav.target_system,
                                                        self.mav.target_component,
                                                        coordinate_frame,
                                                        type_mask,
                                                        int(lat * 10000000),
                                                        int(lon * 10000000),
                                                        alt,
                                                        vx,
                                                        vy,
                                                        vz,
                                                        afx,
                                                        afy,
                                                        afz,
                                                        yaw,
                                                        yaw_rate)


    def do_reposition(self,
               radius,
               lat,
               lon,
               alt):
        """
        Similar to set_position_global but is intended for use with plane frame types. 
        Alt is set to be relative altitude (ground is 0m)
        """

        self.mav.mav.command_int_send(
                self.mav.target_system,
                self.mav.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                0,
                1,
                -1,
                0,
                radius,
                0,
                int(lat * 1e7),
                int(lon * 1e7),
                alt)

                            
    def transition_mc(self): 
        # A method to transition from fixed wing to multi-copter
        # Normal Transition is default, force immediate is not recommended as it can cause damage to the vehicle 

        self.mav.mav.command_long_send(
                self.mav.target_system,
                self.mav.target_component,
                mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
                0,
                mavutil.mavlink.MAV_VTOL_STATE_MC,
                0, # Normal Transition and not a force immediate 
                0,
                0,
                0,
                0,
                0)


                    
    def transition_fw(self): 
        # A method to transition from multi-copter to forward flight
        # Normal Transition is default, force immediate is not recommended as it can cause damage to the vehicle 

        self.mav.mav.command_long_send(
                self.mav.target_system,
                self.mav.target_component,
                mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
                0,
                mavutil.mavlink.MAV_VTOL_STATE_MC,
                0,
                0,
                0,
                0,
                0,
                0)


    def wait_target_reached(self, target, tolerance=0.05, timeout = 30) -> bool:
        """
        Waits for the drone to reach specified target
        """

        current_pos = self.get_local_position() 
        
        check_target = util.check_target_reached(current_pos, target, tolerance)
        start_time = time.time()
        while not check_target:
            current_pos = self.get_local_position()
            check_target = util.check_target_reached(current_pos, target, tolerance)
            if time.time() - start_time > timeout:
                print("MAVCTL Timeout: Failed to reach target within tolerance. Continuing execution")
                return False
            
        return True

    def wait_target_reached_global(self, target, timeout = 30):
        """
        Waits for the drone to reach specified target (in global coordinates)
        """
        current_pos = self.get_global_position()
        distance = util.LatLon_to_Distance(current_pos, target)
        start_time = time.time()

        while not distance < 5:
            current_pos = self.get_global_position()
            distance = util.LatLon_to_Distance(current_pos, target)
            if time.time() - start_time > timeout:
                print("MAVCTL Timeout: Failed to reach target within tolerance. Continuing execution")
                return False
        
        return True

    def broadcast_landing_target(self, landing_target: LandingTarget) -> None: 
        """
        This function broadcasts the position of the landing target in MAV_FRAME_BODY_FRD frame.
        """

        time_usec = int(time.time() * 1000000) # convert to microseconds
            
        print(landing_target)

        self.mav.mav.landing_target_send(time_usec=time_usec,
                        target_num=0,
                        frame=mavutil.mavlink.MAV_FRAME_BODY_FRD,
                        angle_x=landing_target.forward,
                        angle_y=landing_target.right,
                        distance=-landing_target.altitude,
                        size_x = 0.1,
                        size_y = 0.1,
                        position_valid=0
                        ) 
