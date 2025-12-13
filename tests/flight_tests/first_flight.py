from pymavlink import mavutil
from mavctl import Navigator
import time

CONN_STR = "udp:127.0.0.1:14551"

drone = Navigator(ip = CONN_STR)

while not drone.wait_for_mode_and_arm():
    pass

drone.takeoff(10)
time.sleep(5)
drone.simple_goto_global(53.496970, -113.545194, 20)

drone.return_to_launch()
