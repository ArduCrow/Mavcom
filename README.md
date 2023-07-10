# MAVCOM

### Mavlink Communicator

Provides a simplified python interface for controlling Mavlink capable flight controllers.

## Installation

```pip install mavcom```

## Basic Usage

This is an example of how to use Mavcom with a simulated vehicle. To connect to a flight controller, you will most likely use UART0, therefore the connection path will instead be "/dev/ttyS0".

Run SITL:

```sim_vehicle.py -v ArduCopter```

```python
from mavcom.mavcontrol import Mavcom
import time

vehicle = Mavcom(
    connection_path = "127.0.0.1:14550",
)

vehicle.start()

while not vehicle.ready:
    print("Waiting for vehicle to initialise...")
    time.sleep(1)

vehicle.motors_armed = True
while not vehicle.motors_armed:
    print("Waiting for motors to spin up...")
    time.sleep(1)
    
vehicle.flight_mode = "GUIDED"
vehicle.takeoff(alt=10)
```