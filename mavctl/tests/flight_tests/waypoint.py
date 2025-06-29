from connect import conn
from pymavlink import mavutil
from messages.Navigator import Navigator
from messages import advanced
import time

mav = conn.connect()
master = Navigator(mav)

while master.set_mode_wait() and master.wait_vehicle_armed():
    pass

master.takeoff(10)
time.sleep(5)
master.set_speed(2)
time.sleep(2)
advanced.simple_goto_global(master, 53.496483, -113.548249, 20)
time.sleep(5)

master.return_to_launch()
