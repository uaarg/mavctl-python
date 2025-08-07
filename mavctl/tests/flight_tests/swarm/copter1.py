from connect import conn
from pymavlink import mavutil
from messages.Navigator import Navigator
from messages.messenger import Messenger
from messages import advanced
import time

CONN_STR1 = "udp:127.0.0.1:14540"
CONN_STR2 = "udp:127.0.0.1:14541"


mav1 = conn.connect(CONN_STR1)
master1 = Navigator(mav1)


mav2 = conn.connect(CONN_STR2)
master2 = Navigator(mav2)


while master1.wait_vehicle_armed():
    pass

while not master1.set_mode_wait():
    pass


while master2.wait_vehicle_armed():
    pass

while not master2.set_mode_wait():
    pass


master1.takeoff(10)
master2.takeoff(10)
time.sleep(5)
advanced.simple_goto_local(master1, 50, 50, 20)
advanced.simple_goto_local(master2, -50, -50, 20)

master1.return_to_launch()
master2.return_to_launch()
