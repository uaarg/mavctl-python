from pymavlink import mavutil
import time
from messages import util
from messages.location import LocationGlobal, LocationGlobalRelative, LocationLocal, Velocity
from math import sqrt

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

            msg = self.mav.recv_match(type = "HEARTBEAT", blocking = True, timeout = 1)

            if msg:
                current_mode_id = msg.custom_mode
                if current_mode_id == mode_id:
                    return True

            time.sleep(0.5)
            print("MAVCTL: Waiting for " + mode + " mode")
        

    def send_status_message(self, message: str):
        """
        Sends message to GCS
        WIP, does not quite work yet, try messing around with different status message sending methods
        """
        self.mav.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE,
                                     message.encode())
        print("MAVCTL Message: " + message)

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
        msg = self.mav.recv_match(type='HOME_POSITION', blocking=True)

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



    def takeoff(self, altitude):
        """
        Initiates a takeoff the drone to target altiude

        Altitude: Altituide to take off to in meters

        NOTE: Positive is upwards
        """

        print("MAVCTL: Taking Off to: " + str(altitude) + "m")
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

        current_location = self.get_local_position()
        self.wait_target_reached(LocationLocal(0, 0, -altitude))

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



                            




    def wait_target_reached(self, target, tolerance=0.05, timeout = 30) -> bool:
        """
        Waits for the drone to reach specified target
        """

        current_pos = self.get_local_position() 
        
        print(current_pos.north, current_pos.east, current_pos.down)
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

        current_pos = self.get_global_position()
        distance = util.LatLon_to_Distance(current_pos, target)
        start_time = time.time()

        while not distance < 0.5:
            current_pos = self.get_global_position()
            distance = util.LatLon_to_Distance(current_pos, target)
            if time.time() - start_time > timeout:
                print("MAVCTL Timeout: Failed to reach target within tolerance. Continuing execution")
                return False
        
        return True

        

'''
    def wait_target_reached(self, timeout = 30) -> bool:
        time.sleep(10) 
        current_vel = self.get_velocity()
        start_time = time.time()
        vel_buffer = []
        vel_buffer.append(current_vel)
        print(current_vel.magnitude())
        while current_vel.magnitude() and vel_buffer[-1].magnitude() >= 0.25:
            time.sleep(0.5)
            current_vel = self.get_velocity()
            print(current_vel.magnitude())
            print(vel_buffer[-1].magnitude())
            vel_buffer.append(current_vel)
            if time.time() - start_time > timeout:
                print("MAVCTL Timeout: Failed to reach target within tolerance. Continuing execution")
                return False
        return True 
'''


