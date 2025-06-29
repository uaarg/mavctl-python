from connect import conn
from pymavlink import mavutil
from messages.Navigator import Navigator
from messages import advanced
import time

CONN_STR = "tcp:127.0.0.1:14550"
MESSENGER_PORT = 14550

mav = conn.connect(CONN_STR)
master = Navigator(mav)

while master.wait_vehicle_armed():
    pass
    
while not master.set_mode_wait():
    pass

print("moving")
advanced.simple_goto_global(master, 53.496483, -113.548249, 20)
print("Done")
time.sleep(5)

master.return_to_launch()
