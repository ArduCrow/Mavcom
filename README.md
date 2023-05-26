# MAVCOM

### Mavlink Communicator

Provides a simplified python interface for controlling Mavlink capable flight controllers.

## Installation

```pip install mavcom```

## Basic Usage

```python
import mavcom
import time

vehicle = mavcom.Mavcom(
    connection_path = "/dev/ttyS0",
)

vehicle.start()

while not vehicle.ready:
    print("Waiting for vehicle to initialise...")
    time.sleep(1)

vehicle.motors_armed = True
vehicle.takeoff(alt=10)
```