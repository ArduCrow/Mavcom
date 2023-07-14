from src.mavcom.mavcontrol import Mavcom
from multiprocessing import Process
import time
import os

def sim_vehicle():
    os.system("gnome-terminal --tab -e 'bash -c \"sim_vehicle.py -v ArduCopter; exec bash\"'")

sim = Process(target=sim_vehicle, daemon=True)
sim.start()
print(sim.pid)

vehicle = Mavcom(
    connection_path="127.0.0.1:14551"
)
vehicle.start()
time.sleep(1)

def test_receiving_mavlink_packets():
    assert len(vehicle.current_values) > 0
    
def test_vehicle_ready():
    for i in range(20):
        if vehicle.ready:
            break
        else:
            time.sleep(2)
            pass
    assert vehicle.ready
    
def test_battery_state():
    assert vehicle.battery_state.voltage > 0
    assert vehicle.battery_state.current is not None
    assert vehicle.battery_state.remaining > 0
    
def test_motion_state():
    assert vehicle.motion_state.lat is not None
    assert vehicle.motion_state.lon is not None
    assert vehicle.motion_state.alt is not None
    assert vehicle.motion_state.heading is not None
    assert vehicle.motion_state.groundspeed is not None
    
def test_nav_state():
    assert vehicle.nav_state.eph is not None
    assert vehicle.nav_state.epv is not None
    assert vehicle.nav_state.satellites_visible is not None
    assert vehicle.nav_state.fix_type is not None
    
def test_set_flight_mode():
    vehicle.flight_mode = "GUIDED"
    assert vehicle.flight_mode == "GUIDED"
    
def test_arm_motors():
    vehicle.motors_armed = True
    time.sleep(1)
    assert vehicle.motors_armed
    
def test_takeoff():
    vehicle.takeoff(alt=5)
    for i in range(11):
        if vehicle.motion_state.alt < 4.9:
            time.sleep(1)
        else: 
            break
    assert vehicle.motion_state.alt >= 4.9
    
def test_sim_terminated():
    sim.join()
    os.system(f"kill {sim.pid}")
    time.sleep(2)
    assert not sim.is_alive()