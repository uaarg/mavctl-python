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
master.disarm()
