from connect import conn
from pymavlink import mavutil
from messages.Navigator import Navigator
from messages import advanced
import time

CONN_STR = "udp:127.0.0.1:14551"
MESSENGER_PORT = 14552

mav = conn.connect()
master = Navigator(mav)

while not master.wait_for_mode_and_arm():
    pass

print("moving")
advanced.simple_goto_local(master, 53.496483, -113.548249, 20)
print("Done")
time.sleep(5)

master.return_to_launch()
