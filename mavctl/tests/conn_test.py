from connect import conn
from pymavlink import mavutil
from messages.basic import Basic
import time

mav = conn.connect()
master = Basic(mav)

master.send_status_message("hello world")
time.sleep(5)
master.arm()
time.sleep(5)
master.takeoff(10)
time.sleep(5)
print("Moving")
master.set_position_local_ned(10, 10, -10)
time.sleep(10)
master.return_to_launch()
time.sleep(5)
master.disarm()

