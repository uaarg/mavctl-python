from mavctl.connect.conn import Connect
from pymavlink import mavutil
from mavctl.messages.navigator import Navigator
from mavctl.messages.messenger import Messenger
from mavctl.messages import advanced
import time

CONN_STR = "tcp:127.0.0.1:14550"

mav = Connect(ip = CONN_STR)
master = Navigator(mav.mav)

while master.wait_vehicle_armed():
    pass

while not master.set_mode_wait():
    pass

master.takeoff(10)
time.sleep(5)
advanced.simple_goto_global(master, 53.496533, -113.550414, 20)
time.sleep(5)
advanced.simple_goto_global(master, 53.496562, -113.552387, 10)
time.sleep(5)
advanced.simple_goto_global(master, 53.496021, -113.545194, 15)
time.sleep(5)
advanced.simple_goto_global(master, 53.496173, -113.547501, 20)
time.sleep(5)
advanced.simple_goto_local(master, 10, 0, 10)
time.sleep(5)
advanced.simple_goto_local(master, 0, 10, 10)
time.sleep(5)
advanced.simple_goto_local(master, -10, 0, 10)
time.sleep(5)
advanced.simple_goto_local(master, 0, -10, 10)
time.sleep(5)

master.return_to_launch()
