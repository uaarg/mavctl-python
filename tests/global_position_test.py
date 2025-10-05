from tests import harness

from mavctl.connect import conn
from mavctl.messages.Navigator import Navigator
from mavctl.messages.location import LocationGlobal


def get_global_position_test():
    with harness.SitlVehicle() as sitl:
        print(sitl.conn_str)
        mav = conn.connect(sitl.conn_str)
        print(mav)
        master = Navigator(mav)
        pos = master.get_global_position()

        assert pos
        assert type(pos) is LocationGlobal


get_global_position_test()
