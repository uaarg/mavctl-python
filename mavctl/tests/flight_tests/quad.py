from connect import conn
from pymavlink import mavutil
from messages.Navigator import Navigator
from messages import advanced
import time

mav = conn.connect()
master = Navigator(mav)


while master.wait_vehicle_armed():
    pass
    
while not master.set_mode_wait():
    pass

master.takeoff(10)

time.sleep(2)
print("moving")
master.do_reposition(0, 53.492843, -113.549665, 10) 
time.sleep(30)
print("Transitioning to Forward Flight")
master.transition_fw()
time.sleep(20)
master.do_reposition(20, 53.496837, -113.545486, 20)
print("moving")
time.sleep(20)
print("transitioning back")
master.transition_mc()
time.sleep(20)
master.return_to_launch()
