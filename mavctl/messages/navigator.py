from pymavlink import mavutil
import time
from mavctl.messages import util
from mavctl.messages.location import LocationGlobal, LocationGlobalRelative, LocationLocal, Velocity
from mavctl.connect.conn import Connect
from math import sqrt


class Navigator:

    def __init__(self,
                 ip: str = "udp:127.0.0.1:14551",
                 baud: int = 57600,
                 heartbeat_timeout = None):

        self.master = Connect(ip = ip, baud = baud, heartbeat_timeout = heartbeat_timeout).master

        
        

    def DO_NOT_USE_ARM(self):
        """
        Arms the drone.
        NOTE: UNDER NO CIRCUMSTANCE SHOULD THIS BE USED IN A REAL FLIGHT SCRIPT
        NOTE: ONLY FOR USE IN SIMULATED SCRIPTS
        """

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
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
        self.master.motors_armed_wait()
        print("MAVCTL: Armed!")
   
    def wait_vehicle_armed(self):
        """
        Waits for the vehicle to be armed. See samples directory for examples
        """
        print("MAVCTL: Waiting for vehicle to arm")
        self.master.motors_armed_wait()
        print("Armed!")

    def wait_for_mode_and_arm(self, mode="GUIDED", timeout=None) -> bool:
        """Wait for the vehicle to enter ``mode`` and to be armed"""
        mode_ready = self.set_mode_wait(mode=mode, timeout=timeout)
        if not mode_ready:
            return False
        while not self.wait_vehicle_armed():
            return True
        pass

    def DO_NOT_USE_DISARM(self):
        """
        Disarms the drone.
        NOTE: UNDER NO CIRCUMSTANCE SHOULD THIS BE USED IN A REAL FLIGHT SCRIPT
        NOTE: ONLY FOR USE IN SIMULATED SCRIPTS
        """

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
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
        self.master.motors_disarmed_wait()
        print("MAVCTL: Disarmed!")

    def DO_NOT_USE_SET_MODE(self, mode = "GUIDED"):
        """
        Sets the mode of the drone

        mode: Mode to be set to
        """
        mode_mapping = self.master.mode_mapping()
        if mode not in mode_mapping:
            raise ValueError("MAVCTL Error: Mode " + mode + "not recognized")

        mode_id = mode_mapping[mode]

        self.master.mav.command_long_send(
                                    self.master.target_system,
                                    self.master.target_component,
                                    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                    0,
                                    1,
                                    mode_id,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0)
 

    def set_mode_wait(self, mode = "GUIDED", timeout = None) -> bool:
        """
        Waits for the specified mode to be selected.
        Timesout after specified timeout

        mode: Mode you want to wait for, leave blank for the most part
        timeout: Timeout in seconds, leave blank to have no timeout
        """ 
        start_time = time.time()
        
        mode_mapping = self.master.mode_mapping()
        if mode not in mode_mapping:
            raise ValueError("MAVCTL Error: Mode " + mode + "not recognized")

        mode_id = mode_mapping[mode]
        print("MAVCTL: Waiting for " + mode + " mode")
        while True:
            if timeout:
                if time.time() - start_time >= timeout: 
                   print("MAVCTL: Timeout waiting for " + mode + " mode")
                   return False 

            self.master.mav.request_data_stream_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL,
                    1,
                    1)

            msg = self.master.recv_match(type = "HEARTBEAT", blocking = True, timeout = 0.25)

            if msg:
                current_mode_id = msg.custom_mode
                if current_mode_id == mode_id:
                    print("MAVCTL: Set to " + mode + " mode")
                    return True
                    
    def get_global_position(self):
        """
        Gets Global Position of drone.
        returns lat, lon, and altitude with respect to GLOBAL coordinates
        Altitude is a value which is measured from sea level and is positive

        """
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1000
         
        return LocationGlobal(lat, lon, alt)

    def get_local_position(self):
        """
        Gets Local Position of drone.
        returns x, y, z in NED coordinates with respect to local origin
        DOWN is positive
        """
        
        msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        if msg:
            north = msg.x
            east = msg.y
            down = msg.z

        return LocationLocal(north, east, down)

    def get_global_origin(self):
        """
        Gets the local origin of the drone (local position [0, 0, 0]) in terms of lat, lon and alt
        """
        self.master.mav.command_long_send(
                                    self.master.target_system,
                                    self.master.target_component,
                                    mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                    0,
                                    242,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0)
 
        msg = self.master.recv_match(type='HOME_POSITION', blocking=True)
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
        msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        if msg:
            north = msg.vx
            east = msg.vy
            down = msg.vz

        return Velocity(north, east, down)



    def takeoff(self, altitude, pitch=15):
        """
        Initiates a takeoff the drone to target altiude

        Altitude: Altituide to take off to in meters

        NOTE: Positive is upwards
        """

        print("MAVCTL: Taking Off to: " + str(altitude) + "m")
        self.master.mav.command_long_send(
                                        self.master.target_system,
                                        self.master.target_component,
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
        self.master.mav.command_long_send(
                                        self.master.target_system,
                                        self.master.target_component,
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


    def return_to_launch(self):
        print("MAVCTL: RTL")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
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
        
        self.master.mav.set_position_target_local_ned_send(
                                                        0,
                                                        self.master.target_system,
                                                        self.master.target_component,
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


        self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
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
        

        self.master.mav.set_position_target_global_int_send(0,
                                                        self.master.target_system,
                                                        self.master.target_component,
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

        self.master.mav.command_int_send(
                self.master.target_system,
                self.master.target_component,
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

        self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
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

        self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
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
