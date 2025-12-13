from connect import conn
from pymavlink import mavutil
from messages.Navigator import Navigator
from messages import advanced
import time

mav = conn.connect()
master = Navigator(mav)
while not master.wait_for_mode_and_arm():
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
