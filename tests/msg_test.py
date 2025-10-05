from tests import harness

from mavctl.connect import conn
from mavctl.messages.Navigator import Navigator


def send_status_message_test():
    with harness.SitlVehicle() as sitl:
        mav = conn.connect(sitl.conn_str)
        master = Navigator(mav)
        master.send_status_message("MAVCTL: Online")

send_status_message_test()
