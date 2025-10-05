from tests import harness

from mavctl.connect import conn
from mavctl.messages.Navigator import Navigator
from mavctl.messages.location import LocationGlobal


def mini_mission_test():
    with harness.SitlVehicle() as sitl:
        # We need to wait for the SITL gps to init, this takes a while...
        import time; time.sleep(30)

        mav = conn.connect(sitl.conn_str)
        master = Navigator(mav)
        master.set_mode(mode="GUIDED")
        master.set_mode_wait(mode="GUIDED")
        master.arm()
        master.takeoff(5)
        pos = master.get_global_position()

        assert pos
        assert type(pos) is LocationGlobal

        master.return_to_launch()
        master.disarm()


mini_mission_test()
