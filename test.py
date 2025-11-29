from mavctl.messages.navigator import Navigator
from mavctl.connect.conn import Connect
import time

CONN_STR = "udp:127.0.0.1:14553"

connect = Connect(ip = CONN_STR)
drone = Navigator(connect.master)

time.sleep(2)

drone.DO_NOT_USE_SET_MODE()
time.sleep(1)
drone.DO_NOT_USE_ARM()
