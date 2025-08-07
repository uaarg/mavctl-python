from connect import conn
from pymavlink import mavutil
from messages.Navigator import Navigator
from messages.messenger import Messenger
from messages import advanced
import time

CONN_STR = "udp:127.0.0.1:14551"

mav = conn.connect()
master = Navigator(mav)
master.send_status_message("MAVCTL: Online")

while master.wait_vehicle_armed():
    pass

while not master.set_mode_wait():
    pass

master.takeoff(10)
time.sleep(5)
advanced.simple_goto_global(master, 53.496970, -113.545194, 20)

master.return_to_launch()
