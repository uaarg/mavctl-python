from connect import conn
from pymavlink import mavutil
from messages.Navigator import Navigator
from messages import test 
import time

mav = conn.connect()
master = Navigator(mav)

master.send_status_message("hello world")
time.sleep(5)
master.arm()

if master.set_mode_wait():

    master.takeoff(10)
    time.sleep(5)

    test.simple_goto_global(master, 53.496361, -113.546694, 20) 

    master.return_to_launch()
    time.sleep(5)
    master.disarm()

