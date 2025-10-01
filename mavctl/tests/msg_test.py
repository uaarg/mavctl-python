from mavctl.connect.conn import Connect
from mavctl.messages.navigator import Navigator


CONN_STR = "udp:127.0.0.1:14550"

master = Navigator(ip = CONN_STR)


