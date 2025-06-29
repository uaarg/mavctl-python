from connect import conn
from pymavlink import mavutil
from messages.Navigator import Navigator
from messages.messenger import Messenger
from messages import advanced
import time

CONN_STR = "tcp:127.0.0.1:14550"
MESSENGER_PORT = 14550

mav = conn.connect(CONN_STR)
master = Navigator(mav)
mavlink_messenger = Messenger(MESSENGER_PORT)
master.send_status_message("MAVCTL: Online")
mavlink_messenger.send("MAVCTL: ONLINE")

while master.wait_vehicle_armed():
    pass
while not master.set_mode_wait():
    pass

time.sleep(0.5)
master.takeoff(10)
advanced.simple_goto_global(master, 53.496483, -113.548249, 20)

master.return_to_launch()
