
from pymavlink import mavutil
import time
import threading
from pymavlink import mavutil

class Connect:
    def __init__(self,
                 ip: str = "udp:127.0.0.1:14552",
                 baud: int = 57600,
                 heartbeat_timeout = None):

        self.mav = self.connect(ip = ip, baud = baud, heartbeat_timeout=heartbeat_timeout) 


    def send_heartbeat(self, interval: int = 1):

        try:
            while True:
                self.mav.mav.command_long_send(
                    self.mav.target_system,
                    self.mav.target_component,
                    mavutil.mavlink.HEARTBEAT,
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    0)

                time.sleep(interval)

        except Exception as e:
            raise Exception("MAVCTL ERROR: Failed to send heartbeat!: ", e)
        
    def recv_heartbeat(self, interval: int = 1):

        try:
            while True:
                msg_recv = self.mav.mav.recv_match(type="HEARTBEAT", blocking=False)

                # Create disconnect handling here... 

                time.sleep(interval)

        except Exception as e:
            raise Exception("MAVCTL ERROR: Failed to send heartbeat!: ", e)
 

    # Function to connect to mavlink 
    def connect(self, ip: str = "udp:127.0.0.1:14552", # Default IP and port for the UAARG Autopilot Simulator
                baud: int = 57600,             # Default Baud Rate for the PixHawk, Add support for this later
                #source_system = 255,       # Add support for this later 
                #source_component = 0,      # Add support for this later    
                heartbeat_timeout = None    # Automatically assume that there is no timeout 
                ):
        try:
            # NOTE: Using default parameters built into pymavlink, look into how this can be customized
            master = mavutil.mavlink_connection(ip, baud)
            msg_recv = master.recv_match(type="HEARTBEAT", blocking=False)

            send_heartbeat_thread = threading.Thread(target = self.send_heartbeat)
            recv_heartbeat_thread = threading.Thread(target = self.recv_heartbeat)

            send_heartbeat_thread.start()
            recv_heartbeat_thread.start()

            while not msg_recv:
                i = 0
                if heartbeat_timeout != None:
                    if heartbeat_timeout > 0: 
                        i += 1
                        if heartbeat_timeout % 5:
                            print("MAVCTL: Waiting for heartbeat ...")
                    elif heartbeat_timeout == 0:
                        raise Exception("MAVCTL ERROR: Heartbeat not found!")
                print("MAVCTL: Waiting for heartbeat ...")
                msg_recv = master.recv_match(type="HEARTBEAT", blocking=False)
                time.sleep(1)

            print("MAVCTL: Connected at: ", ip)
            return master
        except Exception as e:
            raise Exception("MAVCTL ERROR: Failed to receive heartbeat!: ", e)
