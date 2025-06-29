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
print("Sent Message")
mavlink_messenger.send("MAVCTL: ONLINE")

while True:
    master.get_global_origin()




