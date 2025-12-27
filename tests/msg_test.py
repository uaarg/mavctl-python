from mavctl.connect.conn import Connect
from mavctl.messages.navigator import Navigator
import time

CONN_STR = "udp:127.0.0.1:14550"

connect = Connect(ip=CONN_STR)
master = Navigator(connect)

time.sleep(2)

connect.disconnect()
