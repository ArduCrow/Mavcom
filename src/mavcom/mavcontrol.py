from pymavlink import mavutil
import threading
import time
import numpy as np

from typing import List
from collections import defaultdict
from .mavconstants import AIRFRAME_TYPES, MODE_MAP
from pymavlink.dialects.v10 import ardupilotmega

class Quaternion:
    def __init__(self, w, x, y, z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def __iter__(self):
        yield self.w
        yield self.x
        yield self.y
        yield self.z

    @classmethod
    def from_euler_angles(cls, roll_deg, pitch_deg, yaw_deg):
        """Convert euler angles to Quaternion"""
        roll_rad = np.radians(roll_deg)
        pitch_rad = np.radians(pitch_deg)
        yaw_rad = np.radians(yaw_deg)

        qw = np.cos(roll_rad / 2) * np.cos(pitch_rad / 2) * np.cos(yaw_rad / 2) + np.sin(roll_rad / 2) * np.sin(pitch_rad / 2) * np.sin(yaw_rad / 2)
        qx = np.sin(roll_rad / 2) * np.cos(pitch_rad / 2) * np.cos(yaw_rad / 2) - np.cos(roll_rad / 2) * np.sin(pitch_rad / 2) * np.sin(yaw_rad / 2)
        qy = np.cos(roll_rad / 2) * np.sin(pitch_rad / 2) * np.cos(yaw_rad / 2) + np.sin(roll_rad / 2) * np.cos(pitch_rad / 2) * np.sin(yaw_rad / 2)
        qz = np.cos(roll_rad / 2) * np.cos(pitch_rad / 2) * np.sin(yaw_rad / 2) - np.sin(roll_rad / 2) * np.sin(pitch_rad / 2) * np.cos(yaw_rad / 2)

        return cls(qw, qx, qy, qz)


class Mavcom:
    """
    The primary Mavlink communication and control object. This class connects to the flight controller
    and sends/receives Mavlink messages. Information from required/desired messages is stored and kept up
    to date.

    Parameters
    ----------

    - `controller`: Object. If you are using an upstream class to utilise the Mavcom control functions, pass
    this object as "self".

    - `required_message_types`: List[string]. If there are any additional Mavlink messages your code needs the data from,
    enter them as a list to this parameter.

    - `connection_path`: String. This is the path to connect to the Mavlink stream from the flight controller. A typical serial
    connection will look like "/dev/ttyS0". A SITL connection will look like "127.0.0.1:14551".

    - `baud`: Int. This is the baud rate which the flight controller is using. This must match.
    """

    def __init__(
        self,
        controller: object = None,
        required_message_types: List[str] = [],
        connection_path: str = "/dev/ttyS0",
        baud: int = 921600,
    ):
        mandatory_message_types = [
            "HEARTBEAT",
            "GLOBAL_POSITION_INT",
            "GPS_STATUS",
            "GPS_RAW_INT",
            "EKF_STATUS_REPORT",
            "VFR_HUD",
            "HOME_POSITION",
            "SYS_STATUS",
        ]
        self.required_message_types = required_message_types + [
            m for m in mandatory_message_types if m not in required_message_types
        ]
        self.current_values = defaultdict(lambda: None)
        self.controller = controller
        has_controller = True if controller else False

        self._flight_mode = None
        self.flight_controller_current_mode = None # separate to _flight_mode, to allow upstream controllers to check the mode without setting it
        self._motors_armed = False
        self.airframe = None
        self.data_rate = 200

        self.telemetry_thread = threading.Thread(
            target=self._monitor_mavlink_messages, daemon=has_controller
        )

        self.connection = mavutil.mavlink_connection(connection_path, baud=baud)
        self._get_heartbeat()

    def start(self) -> None:
        """
        Starts listening to the Mavlink messages from the flight controller. Has a small delay
        so that Mavcom can populate required data types. Call this function first.

        If a controller object was passed to this Mavcom instance, the telemetry thread will run as a daemon
        so that it exits when the caller object terminates.
        """
        print("MAVCOM: Mavcom active")
        self.connection.mav.request_data_stream_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            self.data_rate,
            1,
        )
        self.telemetry_thread.start()
        time.sleep(3)

    def _get_heartbeat(self):
        print("MAVCOM: Waiting for heartbeat...")
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            242,  # Confirmation (set to 0 if no confirmation needed)
            mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT,  
            0, 0, 0, 0, 0, 0
        )
        self.connection.wait_heartbeat(timeout=5)
        print(
            f"MAVCOM: Heartbeat from system (system {self.connection.target_system} "
            f"component {self.connection.target_component})"
        )
        hb = self.connection.recv_match(type="HEARTBEAT", blocking=True)
        self.current_values["HEARTBEAT"] = hb.to_dict()

        self.airframe = AIRFRAME_TYPES[self.current_values["HEARTBEAT"]["type"]]
        # self.flight_mode = MODE_MAP[self.airframe][self.current_values["HEARTBEAT"]["custom_mode"]] # Removed, don't set the mode

    def _monitor_mavlink_messages(self):
        while True:
            message = self.connection.recv_match(type=self.required_message_types)
            if message is not None:
                message_dict = message.to_dict()

                self.current_values[message_dict["mavpackettype"]] = message_dict

            self._motors_armed = (
                self.current_values["HEARTBEAT"]["base_mode"]
                & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            ) != 0
            self.flight_controller_current_mode = MODE_MAP[self.airframe][self.current_values["HEARTBEAT"]["custom_mode"]]

    def get_home_pos(self):
        """
        FC will only send home position message a few times during initialisation.
        Prompt it to transmit and capture it.
        """
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            242,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        time.sleep(0.5)
        home_position = self.current_values["HOME_POSITION"]
        return home_position

    def takeoff(self, alt: int):
        """
        Sends a Takeoff command to the flight controller.
        This will only succeed if the `ready` property is `True`
        and the vehicle is armed (`motors_armed == True`)

        Parameters
        ----------

        - `alt`: Integer. Altitude in meters to takeoff to.
        """
        print(f"MAVCOM: Takeoff to {alt}m relative")
        self.connection.mav.command_long_send(
            0, 0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt
        )

    def travel(self, location: tuple[float], alt: int, groundspeed: int = None):
        """
        Sends a go-to waypoint command to the flight controller. Vehicle must be airborne and armed.
        The vehicle will take the altitude as relative to it's starting position, i.e. the altitude of
        the home position is 0.

        Parameters
        ----------

        - `location`: Tuple[float]. The latitude, longitude to travel to.

        - `alt': Integer. Altitude AGL to achieve at the destination.

        - `groundspeed`: Integer. Groundspeed in meters per second to travel at.
        """
        print(f"MAVCOM: Travel to {location}, {alt}m AGL relative")
        self.connection.mav.mission_item_send(
            0,
            0,
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            2,
            0,
            0,
            0,
            0,
            0,
            location[0],
            location[1],
            alt,
        )
        if groundspeed:
            self.set_groundspeed(groundspeed)

    def set_groundspeed(self, speed: int):
        """
        Sets the groundspeed the vehicle will travel at.

        - `speed`: Integer. Meters per second.
        """
        self.connection.mav.command_long_encode(
            0, 0, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 1, speed, -1, 0, 0, 0, 0
        )
        print(f"MAVCOM: Set groundspeed: {speed}m/s")

    def reset_home(self):
        """
        Resets the home coordinates to the current location.
        Note that the home location also has an altitude of 0.
        """
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )

    def go_to_pickup_point(self):
        """
        Return to user defined collection point (or home coords).

        Unused.
        """
        pass

    def _calculate_quaternion(self, roll_deg, pitch_deg, yaw_deg) -> Quaternion:
        """
        Calculate quaternion based on roll, pitch, and yaw angles.

        Parameters:
            - `roll_deg`: Roll angle in degrees.
            - `pitch_deg`: Pitch angle in degrees.
            - `yaw_deg`: Yaw angle in degrees.

        Returns:
            A Quaternion object representing the required attitude.
        """
        return Quaternion.from_euler_angles(roll_deg, pitch_deg, yaw_deg)

    def _normalize_thrust(self, user_thrust, current_thrust):
        """
        Normalize the user-provided thrust value based on the current thrust value.
        Ensure the normalized thrust is between 0 and 1.

        Parameters:

            - `user_thrust`: User-provided thrust value.
            - `current_thrust`: Current thrust value obtained from the vehicle.

        Returns:
            A normalized thrust value between 0 and 1.
        """
        normalized_thrust = user_thrust / 100
        if normalized_thrust > 1.0 or normalized_thrust < 0:
            return current_thrust

        return normalized_thrust

    def set_attitude(
        self,
        roll_deg: int = 0,
        pitch_deg: int = 0,
        yaw_deg: int = 0,
        thrust: int = None,
    ):
        """
        Sets the vehicle attitude to the specified roll, pitch or yaw angle, in degrees.

        Calculates the quaternion from this angle to send a mavlink attitude command to
        the FC.

        Parameters:
        - `roll_deg` : desired roll angle in degrees
        - `pitch_deg` : desired pitch angle in degrees
        - `yaw_deg` : desired yaw angle in degrees
        - `thrust` : desired throttle power as percent, 0-100
        """
        q = self._calculate_quaternion(roll_deg, pitch_deg, yaw_deg)
        q_vals = [float(i) for i in q]
        current_thrust = self.current_values["VFR_HUD"]["throttle"]
        
        if thrust is None or thrust > 100:
            thrust = current_thrust
        normalized_thrust = self._normalize_thrust(thrust, current_thrust)

        self.connection.mav.set_attitude_target_send(
            0,
            0,
            0,
            0,
            q_vals,
            0,  # roll rate
            0,  # pitch rate
            0,  # yaw rate,
            normalized_thrust,
        )

    def _arm_disarm(self, force=False):
        action = 0 if self.motors_armed else 1
        force = 21196 if force else 0
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            action,
            force,
            0,
            0,
            0,
            0,
            0,
        )
        if force:
            print("MAVCOM: FORCE DISARM")

    def force_disarm(self):
        """
        Forces the motors to stop regardless of the state of the vehicle.

        WARNING: This will cause the vehicle to drop out of the sky.
        """
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            21196,
            0,
            0,
            0,
            0,
            0,
        )

    def land(self):
        """
        The vehicle will land at the current location. Quadrotors only.
        """
        self.travel(location=(self.vehicle_state.lat, self.vehicle_state.lon), alt=0)

    @property
    def ready(self):
        """Return True if vehicle is ready to arm and takeoff."""
        if (
            not self.current_values["EKF_STATUS_REPORT"]
            or self.nav_state == "GPS not initialised"
        ):
            return False
        pred_horiz_pos = (
            self.current_values["EKF_STATUS_REPORT"]["flags"]
            & ardupilotmega.EKF_PRED_POS_HORIZ_ABS
        ) > 0
        return (
            self.flight_mode != "INITIALISING"
            and (self.nav_state.fix_type is not None and self.nav_state.fix_type > 1)
            and pred_horiz_pos
        )

    @property
    def motors_armed(self):
        return self._motors_armed

    @motors_armed.setter
    def motors_armed(self, cmd):
        if cmd == "force_disarm":
            self._arm_disarm(force=True)
            self.motors_armed = False
        if bool(cmd) != self._motors_armed:
            if cmd is True:
                self._arm_disarm()
                print("MAVCOM: ARM")
            else:
                self._arm_disarm()
                print("MAVCOM: DISARM")

    @property
    def flight_mode(self):
        if not self._flight_mode:
            return None
        return self._flight_mode

    @flight_mode.setter
    def flight_mode(self, mode):
        m = self._match_mode(mode)
        if m is None:
            raise ModeError(mode)
        self.connection.set_mode(m)
        self._flight_mode = m
        # print("MAVCOM: Flight mode SET:", mode)
        
    @flight_mode.getter
    def flight_mode(self):
        return self._flight_mode

    def _match_mode(self, mode):
        for mode_dict in MODE_MAP.values():
            for key, value in mode_dict.items():
                if value == mode:
                    return value
        return None

    @property
    def nav_state(self):
        """
        The current state of the navigation system.

        Attributes
        ----------

        - `eph`: standard deviation of horizontal position error (meters)

        - `epv`: standard deviation of vertical position error (meters)

        - `fix_type`: 0-8 numerical representation of GPS fix type/quality

        - `satellites_visible`: number of satellites currently visible
        """
        if "GPS_RAW_INT" not in self.current_values:
            return "GPS not initialised"
        navdata = {
            "eph": self.current_values["GPS_RAW_INT"]["eph"],
            "epv": self.current_values["GPS_RAW_INT"]["epv"],
            "fix_type": self.current_values["GPS_RAW_INT"]["fix_type"],
            "satellites_visible": self.current_values["GPS_RAW_INT"][
                "satellites_visible"
            ],
        }
        return NavState(
            navdata["eph"],
            navdata["epv"],
            navdata["fix_type"],
            navdata["satellites_visible"],
        )

    @property
    def motion_state(self):
        """
        The current position & motion state of the vehicle.

        Attributes
        ----------

        - `alt`: altitude AGL relative to home position

        - `groundspeed`: groundspeed in m/s

        - `vertical_speed`: ascend/descend speed in m/s

        - `heading`: heading in degrees

        - `lat`: latitude in decimal format

        - `lon`: longitude in decimal format
        """
        alt = self.current_values["GLOBAL_POSITION_INT"]["relative_alt"] / 1000
        groundspeed = self.current_values["VFR_HUD"]["groundspeed"]
        vertical_speed = self.current_values["VFR_HUD"]["climb"]
        heading = self.current_values["GLOBAL_POSITION_INT"]["hdg"]
        lat = self.current_values["GLOBAL_POSITION_INT"]["lat"] / 1e7
        lon = self.current_values["GLOBAL_POSITION_INT"]["lon"] / 1e7
        return MotionState(
            alt=alt,
            groundspeed=groundspeed,
            vertical_speed=vertical_speed,
            heading=heading,
            lat=lat,
            lon=lon,
        )

    @property
    def battery_state(self):
        """
        The current state of the battery.

        Attributes
        ----------

        - `voltage`: battery voltage in volts

        - `current`: battery current in amps

        - `remaining`: remaining battery percentage
        """
        voltage = self.current_values["SYS_STATUS"]["voltage_battery"] / 1000
        current = self.current_values["SYS_STATUS"]["current_battery"] / 100
        remaining = self.current_values["SYS_STATUS"]["battery_remaining"]
        return BatteryState(voltage=voltage, current=current, remaining=remaining)


class NavState(object):
    def __init__(self, eph, epv, fix_type, satellites_visible) -> None:
        self.eph = eph
        self.epv = epv
        self.fix_type = fix_type
        self.satellites_visible = satellites_visible

    def current_state_dict(self):
        return {
            "eph": self.eph,
            "epv": self.epv,
            "fix_type": self.fix_type,
            "satellites_visible": self.satellites_visible,
        }


class MotionState(object):
    def __init__(self, alt, groundspeed, vertical_speed, heading, lat, lon) -> None:
        self.alt = alt
        self.groundspeed = groundspeed
        self.vertical_speed = vertical_speed
        self.heading = heading
        self.lat = lat
        self.lon = lon


class ModeError(Exception):
    def __init__(self, value) -> None:
        self.value = value

    def __str__(self):
        return repr(self.value)


class BatteryState(object):
    """
    The current state of the battery.

    Attributes
    ----------

    - `voltage`: voltage in millivolts
    - `current`: current in centiamps
    - `remaining`: list of cell voltages in millivolts
    """

    def __init__(self, voltage, current, remaining) -> None:
        self.voltage = voltage
        self.current = current
        self.remaining = remaining
        return super().__init__()
