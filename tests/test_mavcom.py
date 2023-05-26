from mavcom import mavcom
import time


vehicle = mavcom.Mavcom(
    connection_path="127.0.0.1:14551"
)
time.sleep(1)

def test_receiving_mavlink_packets():
    assert len(vehicle.current_values) > 0
    
def test_vehicle_ready():
    for i in range(20):
        if vehicle.ready:
            assert True
        else:
            time.sleep(1)
            continue
    assert False
    
def test_set_flight_mode():
    vehicle.flight_mode = "GUIDED"
    assert vehicle.flight_mode == "GUIDED"
    
def test_takeoff():
    vehicle.takeoff(alt=5)
    time.sleep(5)
    assert vehicle.vehicle_state.alt >= 4.9