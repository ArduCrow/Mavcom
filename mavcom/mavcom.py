from pymavlink import mavutil
import threading
import time

from typing import List
from collections import defaultdict
from mavcom.mavconstants import AIRFRAME_TYPES, MODE_MAP
from pymavlink.dialects.v10 import ardupilotmega

class Mavcom:

    def __init__(self,
                 controller: object = None,
                 required_message_types: List[str] = [],
                 connection_path: str = "/dev/ttyS0",
                 baud: int = 921600
                 ):

        mandatory_message_types = [
            "HEARTBEAT",
            "GLOBAL_POSITION_INT",
            "GPS_STATUS",
            "GPS_RAW_INT",
            "EKF_STATUS_REPORT",
            "VFR_HUD"
        ]
        self.required_message_types = required_message_types + [m for m in mandatory_message_types if m not in required_message_types]
        self.current_values = defaultdict(lambda: None)
        self.controller = controller
        
        self._flight_mode = None
        self._motors_armed = False
        self.airframe = None
        
        self.telemetry_thread = threading.Thread(target=self.monitor_mavlink_messages)

        self.connection = mavutil.mavlink_connection(connection_path, baud=baud)
        self.get_heartbeat()

    def start(self):
        """Starts listening to the Mavlink messages from the flight controller."""
        print("MAVCOM: Mavcom active")
        self.telemetry_thread.start()

    def get_heartbeat(self):
        print("MAVCOM: Waiting for heartbeat...")
        self.connection.wait_heartbeat()
        print(f"MAVCOM: Heartbeat from system (system {self.connection.target_system} "
                    f"component {self.connection.target_component})")
        hb = self.connection.recv_match(type="HEARTBEAT", blocking = True)
        self.current_values['HEARTBEAT'] = hb.to_dict()
        
        self.airframe = AIRFRAME_TYPES[self.current_values['HEARTBEAT']['type']]
        self.flight_mode = MODE_MAP[self.airframe][self.current_values["HEARTBEAT"]["custom_mode"]]

    def monitor_mavlink_messages(self):

        while True:
            message = self.connection.recv_match(type=self.required_message_types)
            if message is not None:
                message_dict = message.to_dict()

                self.current_values[message_dict["mavpackettype"]] = message_dict
                
            self._motors_armed = (self.current_values['HEARTBEAT']['base_mode'] & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0

    def get_home_pos(self):
        """
        FC will only send home position message a few times.
        Prompt it to transmit and capture it.
        """
        i = 0
        while "HOME_POSITION" not in self.current_values:
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
                0,0,0,0,0,0,0,0
            )
            time.sleep(0.5)
            i += 1
            if i > 10:
                break
        home_position = self.current_values['HOME_POSITION']
        return home_position

    def takeoff(self, alt: int):
        print(f"MAVCOM: Takeoff to {alt}m relative")
        self.connection.mav.command_long_send(0, 0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                               0, 0, 0, 0, 0, 0, 0, alt)

    def travel(self, location: tuple[float], alt: int, groundspeed: int = None):
        print(f"MAVCOM: Travel to {location}, {alt}m AGL relative")
        self.connection.mav.mission_item_send(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                           mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0,
                                           0, 0, 0, location[0], location[1],
                                           alt)
        if groundspeed:
            self.set_groundspeed(groundspeed)

    def set_groundspeed(self, speed: int):
        self.connection.mav.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,
            1, 
            speed,  
            -1, 0, 0, 0, 0 
        )
        print(f"MAVCOM: Set groundspeed: {speed}m/s")
        
    def reset_home(self):
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,1,0,0,0,0,0,0,0,
        )

    def go_to_pickup_point(self):
        """
        Return to user defined collection point (or home coords).
        """
        pass
    
    def _arm_disarm(self, force=False):
        action = 0 if self.motors_armed else 1
        force = 21196 if force else 0
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,action,force,0,0,0,0,0,
        )
        if force:
            print("MAVCOM: FORCE DISARM")

    def force_disarm(self):
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,0,21196,0,0,0,0,0,
        )

    def land(self):
        self.travel(
            location=(
                self.vehicle_state.lat,
                self.vehicle_state.lon
            ),
            alt=0
        )
    
    @property
    def ready(self):
        """Return True if vehicle is ready to arm and takeoff."""
        if not self.current_values['EKF_STATUS_REPORT'] or self.nav_state == "GPS not initialised":
            return False
        pred_horiz_pos = (self.current_values['EKF_STATUS_REPORT']['flags'] & ardupilotmega.EKF_PRED_POS_HORIZ_ABS) > 0
        return self.flight_mode != 'INITIALISING' and (self.nav_state.fix_type is not None and self.nav_state.fix_type > 1) and pred_horiz_pos
    
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
        print("MAVCOM: Flight mode SET:", mode)
        
    def _match_mode(self, mode):
        for mode_dict in MODE_MAP.values():
            for key, value in mode_dict.items():
                if value == mode:
                    return value
        return None
    
    @property
    def nav_state(self):
        if "GPS_RAW_INT" not in self.current_values:
            return "GPS not initialised"
        navdata = {
            "eph": self.current_values['GPS_RAW_INT']['eph'],
            "epv": self.current_values['GPS_RAW_INT']['epv'],
            "fix_type": self.current_values['GPS_RAW_INT']['fix_type'],
            "satellites_visible": self.current_values['GPS_RAW_INT']['satellites_visible']
        }
        return NavState(navdata['eph'], navdata['epv'], navdata['fix_type'], navdata['satellites_visible'])
    
    @property
    def vehicle_state(self):
        alt = self.current_values['GLOBAL_POSITION_INT']['relative_alt'] / 1000
        groundspeed = self.current_values["VFR_HUD"]['groundspeed']
        vertical_speed = self.current_values['VFR_HUD']['climb']
        heading = self.current_values['GLOBAL_POSITION_INT']['hdg']
        lat = self.current_values['GLOBAL_POSITION_INT']['lat'] / 1e7
        lon = self.current_values['GLOBAL_POSITION_INT']['lon'] / 1e7
        return VehicleState(alt=alt, groundspeed=groundspeed, vertical_speed=vertical_speed, heading=heading, lat=lat, lon=lon)
    
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
            "satellites_visible": self.satellites_visible
        }
        
class VehicleState(object):
    
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
        return(repr(self.value))