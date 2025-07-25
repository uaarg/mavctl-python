from connect import conn
from pymavlink import mavutil
from messages.Navigator import Navigator
from messages import advanced
import time

mav = conn.connect()
master = Navigator(mav)


while master.wait_vehicle_armed():
    pass
    
while not master.set_mode_wait():
    pass

print("moving")
time.sleep(2)
master.do_reposition(20, 53.496837, -113.545486, 20)
print("Sent waypoint message")
                        
ack = master.mav.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
if ack:
    print(ack.result)
else:
    print("no ack recv")
