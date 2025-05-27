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

    test.simple_goto(master, 10, 10, 5) 


    master.return_to_launch()
    time.sleep(5)
    master.disarm()

