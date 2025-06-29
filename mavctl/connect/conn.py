
from pymavlink import mavutil
import time



# Function to connect to mavlink 
def connect(ip = "udp:127.0.0.1:14552", # Default IP and port for the UAARG Autopilot Simulator
            #baud = 115200,             # Default Baud Rate for the PixHawk, Add support for this later
            #source_system = 255,       # Add support for this later 
            #source_component = 0,      # Add support for this later    
            heartbeat_timeout = None    # Automatically assume that there is no timeout 
            ):
    try:
        # NOTE: Using default parameters built into pymavlink, look into how this can be customized
        master = mavutil.mavlink_connection(ip, baud=57600)
        msg_recv = master.recv_match(type="HEARTBEAT", blocking=False)
        while not msg_recv:
            i = 0
            if heartbeat_timeout != None:
                if heartbeat_timeout > 0: 
                    i += 1
                    if heartbeat_timeout % 5:
                        print("MAVLink Connection: Waiting for heartbeat ...")
                elif heartbeat_timeout == 0:
                    raise Exception("MAVLink Connection Error: Heartbeat not found!")
            msg_recv = master.recv_match(type="HEARTBEAT", blocking=False)
            time.sleep(1)

        print("MAVLink Connected at: ", ip)
        return master
    except Exception as e:
        raise Exception("MAVLink Connection Error: ", e)
