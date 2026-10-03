import time
from math import atan, pi
from dataclasses import dataclass
from typing import Optional, Iterable

from pymavlink import mavutil

from mavctl._internal import util
from mavctl.messages.location import LocationGlobal, LocationLocal, Velocity
from mavctl.connect.conn import Connect
from mavctl._internal.logger import get_logger

# Initialize the Logger
LOGGER = get_logger(__name__)

# ----------------------
# Dataclasses for setpoints
# ----------------------
@dataclass
class PositionSetpointLocal:
    x: float = 0
    y: float = 0
    z: float = 0
    vx: float = 0
    vy: float = 0
    vz: float = 0
    afx: float = 0
    afy: float = 0
    afz: float = 0
    yaw: float = 0
    yaw_rate: float = 0


@dataclass
class PositionSetpointGlobal:
    lat: float = 0
    lon: float = 0
    alt: float = 0
    vx: float = 0
    vy: float = 0
    vz: float = 0
    afx: float = 0
    afy: float = 0
    afz: float = 0
    yaw: float = 0
    yaw_rate: float = 0

MASK_POS_ONLY = 0x0DF8
MASK_POS_YAW = 0x09F8
MASK_VEL_ONLY = 0x0DC7


@dataclass
class ADSBVehicle:
    """Describes a reported ADS-B traffic vehicle, in human-readable units."""
    icao_address: int
    lat: float = 0  # degrees
    lon: float = 0  # degrees
    altitude: float = 0  # metres (ASL)
    heading: float = 0  # degrees, 0-359.99
    hor_velocity: float = 0  # m/s
    ver_velocity: float = 0  # m/s, positive up
    callsign: str = ""
    squawk: int = 0
    altitude_type: int = mavutil.mavlink.ADSB_ALTITUDE_TYPE_GEOMETRIC
    emitter_type: int = mavutil.mavlink.ADSB_EMITTER_TYPE_UAV
    tslc: int = 0


@dataclass
class ADSBAvoidanceConfig:
    """ArduPilot AVOID_ADSB (AVD_*) avoidance configuration."""
    enable: bool = True
    fail_dist_xy: float = 500   # metres, horizontal near-miss distance ("radius")
    fail_dist_z: float = 500    # metres, vertical near-miss distance
    fail_time: float = 0        # seconds of advance warning before reacting
    fail_action: int = 2        # 2=Climb/Descend,3=Move Horiz,4=Move Perp 3D,5=RTL,6=Hover
    fail_recovery: int = 1      # 1=Resume previous flight mode


# ----------------------
# Navigator Class
# ----------------------
class Navigator:  # pylint: disable=too-many-public-methods,too-many-instance-attributes
    """Navigator class for MAVLink drones with logging and helper functions."""

    def __init__(self, ip: str = "udp:127.0.0.1:14551",
                 baud: int = 57600,
                 heartbeat_timeout: Optional[int] = None):
        """
        Initialize a MAVLink connection.

        Args:
            ip: Connection string (UDP/serial).
            baud: Serial baud rate (ignored for UDP).
            heartbeat_timeout: Optional heartbeat timeout.
        """
        self.master = Connect(ip=ip, baud=baud, heartbeat_timeout=heartbeat_timeout).master
        LOGGER.info("Navigator connected to %s", ip)

    # ----------------------
    # Arm / Disarm
    # ----------------------
    def arm(self) -> None:
        """Arms the vehicle (simulation only)."""
        self._send_command_arm_disarm(True)
        LOGGER.info("Waiting for vehicle to arm...")
        self.master.motors_armed_wait()
        LOGGER.info("Vehicle armed.")

    def disarm(self) -> None:
        """Disarms the vehicle (simulation only)."""
        self._send_command_arm_disarm(False)
        LOGGER.info("Waiting for vehicle to disarm...")
        self.master.motors_disarmed_wait()
        LOGGER.info("Vehicle disarmed.")

    def _send_command_arm_disarm(self, arm: bool) -> None:
        """Helper to arm/disarm vehicle."""
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            int(arm),
            0, 0, 0, 0, 0, 0
        )

    # ----------------------
    # Mode management
    # ----------------------
    def set_mode_wait(self, mode: str = "GUIDED", timeout: Optional[int] = None) -> bool:
        """
        Wait until the vehicle is in the specified mode.

        Args:
            mode: Target mode (e.g., "GUIDED").
            timeout: Maximum wait time in seconds.

        Returns:
            True if mode is set, False if timed out.
        """
        start_time = time.time()
        mode_mapping = self.master.mode_mapping()
        if mode not in mode_mapping:
            raise ValueError(f"Mode {mode} not recognized")
        mode_id = mode_mapping[mode]
        LOGGER.info("Waiting for mode %s", mode)

        while True:
            if timeout and (time.time() - start_time) >= timeout:
                LOGGER.warning("Timeout waiting for mode %s", mode)
                return False

            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                1,
                1
            )
            msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.25)
            if msg and getattr(msg, "custom_mode", None) == mode_id:
                LOGGER.info("Mode set to %s", mode)
                return True

    def wait_for_mode_and_arm(self, mode="GUIDED", timeout=None) -> bool:
        """Wait for the vehicle to enter ``mode`` and to be armed"""
        mode_ready = self.set_mode_wait(mode=mode, timeout=timeout)
        if not mode_ready:
            return False
        while not self.wait_vehicle_armed():
            return True
        LOGGER.warning("MAVCTL: Failed to wait for vehicle arm")
        return False

    def wait_vehicle_armed(self):
        """
        Waits for the vehicle to be armed. See samples directory for examples
        """
        LOGGER.info("MAVCTL: Waiting for vehicle to arm")
        self.master.motors_armed_wait()
        LOGGER.info("MAVCTL: Armed!")

    # ----------------------
    # Position / Velocity access
    # ----------------------
    def get_global_position(self) -> LocationGlobal:
        """Returns the current global position."""
        msg = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
        if msg:
            return LocationGlobal(msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000)
        return LocationGlobal(0, 0, 0)

    def get_local_position(self) -> LocationLocal:
        """Returns the current local NED position."""
        msg = self.master.recv_match(type="LOCAL_POSITION_NED", blocking=True)
        if msg:
            return LocationLocal(msg.x, msg.y, msg.z)
        return LocationLocal(0, 0, 0)

    def get_local_position_ned(self) -> tuple[float, float, float]:
        """
        Convenience helper returning the current local NED position as a tuple
        (north, east, down), mirroring older Navigator APIs.
        """
        pos = self.get_local_position()
        return (pos.north, pos.east, pos.down)

    def get_velocity(self) -> Velocity:
        """Returns the current local velocity in NED."""
        msg = self.master.recv_match(type="LOCAL_POSITION_NED", blocking=True)
        if msg:
            return Velocity(msg.vx, msg.vy, msg.vz)
        return Velocity(0, 0, 0)

    # ----------------------
    # Takeoff
    # ----------------------
    def takeoff(self, altitude: float, pitch: float = 15) -> None:
        """Initiates a takeoff to the target altitude."""
        LOGGER.info("Takeoff to %.2f m", altitude)
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            pitch,
            0, 0, 0, 0, 0,
            altitude
        )
        self.wait_target_reached(PositionSetpointLocal(z=-altitude))

    # ----------------------
    # Setpoints
    # ----------------------
    def set_position_local_ned(self, setpoint: PositionSetpointLocal,
                               coordinate_frame: int = mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                               type_mask: int = 0x07FF) -> None:
        """Send a local NED setpoint."""
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            coordinate_frame,
            type_mask,
            setpoint.x, setpoint.y, setpoint.z,
            setpoint.vx, setpoint.vy, setpoint.vz,
            setpoint.afx, setpoint.afy, setpoint.afz,
            setpoint.yaw, setpoint.yaw_rate
        )
        LOGGER.debug("Local setpoint sent: %s", setpoint)

    def set_position_target_local_ned(
        self,
        x: float = 0,
        y: float = 0,
        z: float = 0,
        coordinate_frame: int = mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask: int = 0x07FF,
    ) -> None:
        """
        Convenience wrapper mirroring older set_position_target_local_ned APIs.

        Uses position-only setpoints; velocity/acceleration/yaw fields are left
        at their defaults.
        """
        setpoint = PositionSetpointLocal(x=x, y=y, z=z)
        self.set_position_local_ned(setpoint, coordinate_frame=coordinate_frame, type_mask=type_mask)

    def set_position_global(self, setpoint: PositionSetpointGlobal,
                            coordinate_frame: int = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                            type_mask: int = 0x07FF) -> None:
        """Send a global setpoint."""
        self.master.mav.set_position_target_global_int_send(
            0,
            self.master.target_system,
            self.master.target_component,
            coordinate_frame,
            type_mask,
            int(setpoint.lat * 1e7),
            int(setpoint.lon * 1e7),
            setpoint.alt,
            setpoint.vx, setpoint.vy, setpoint.vz,
            setpoint.afx, setpoint.afy, setpoint.afz,
            setpoint.yaw, setpoint.yaw_rate
        )
        LOGGER.debug("Global setpoint sent: %s", setpoint)

    # ----------------------
    # Target reaching
    # ----------------------
    def wait_target_reached(self, target: Optional[PositionSetpointLocal] = None,
                            tolerance: float = 0.05, timeout: int = 30) -> bool:
        """Wait until the local target is reached within tolerance."""
        if target is None:
            target = PositionSetpointLocal()
        start_time = time.time()
        while True:
            current_pos = self.get_local_position()
            if util.check_target_reached(current_pos, target, tolerance):
                LOGGER.info("Target reached locally: %s", target)
                return True
            if (time.time() - start_time) > timeout:
                LOGGER.warning("Timeout: Failed to reach local target: %s", target)
                return False

    def wait_target_reached_global(self, target: Optional[PositionSetpointGlobal] = None,
                                   timeout: int = 30) -> bool:
        """Wait until the global target is reached within tolerance."""
        if target is None:
            target = PositionSetpointGlobal()
        start_time = time.time()
        while True:
            current_pos = self.get_global_position()
            distance = util.LatLon_to_Distance(current_pos, target)
            if distance < 5:
                LOGGER.info("Target reached globally: %s", target)
                return True
            if (time.time() - start_time) > timeout:
                LOGGER.warning("Timeout: Failed to reach global target: %s", target)
                return False

    # ----------------------
    # Simple goto commands
    # ----------------------
    def simple_goto_local(self, x: float, y: float, z: float) -> None:
        """Move drone to local NED coordinates."""
        type_mask = self.generate_typemask([0, 1, 2, 9])
        yaw_angle = atan(y / x) if x != 0 else 0
        setpoint = PositionSetpointLocal(x=x, y=y, z=-z, yaw=yaw_angle)
        self.set_position_local_ned(setpoint, type_mask=MASK_POS_YAW)
        self.wait_target_reached(setpoint)

    def simple_goto_global(self, lat: float, lon: float, alt: float) -> None:
        """Move drone to global coordinates."""
        type_mask = self.generate_typemask([0, 1, 2, 9])
        start_point = self.get_global_position()
        yaw = util.Heading(start_point, LocationGlobal(lat, lon, alt))
        setpoint = PositionSetpointGlobal(lat=lat, lon=lon, alt=alt, yaw=yaw)
        self.set_position_global(setpoint, type_mask=MASK_POS_YAW)
        self.wait_target_reached_global(setpoint)

    # ----------------------
    # VTOL transitions
    # ----------------------
    def transition_mc(self) -> None:
        """Transition from fixed-wing to multi-copter mode."""
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
            0,
            mavutil.mavlink.MAV_VTOL_STATE_MC,
            0, 0, 0, 0, 0, 0
        )
        LOGGER.info("VTOL transition: multi-copter")

    def transition_fw(self) -> None:
        """Transition from multi-copter to fixed-wing (forward flight) mode."""
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
            0,
            mavutil.mavlink.MAV_VTOL_STATE_FW,
            0, 0, 0, 0, 0, 0
        )
        LOGGER.info("VTOL transition: forward flight")

    # ----------------------
    # Return to Launch
    # ----------------------
    def return_to_launch(self) -> None:
        """Command the vehicle to return to its launch location."""
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        LOGGER.info("Return to Launch (RTL) command sent")

    # ----------------------
    # Repositioning
    # ----------------------
    def do_reposition(self, radius: float, lat: float, lon: float, alt: float) -> None:
        """
        Reposition vehicle for plane frame types (altitude relative to ground).

        Args:
            radius: Desired radius around target location.
            lat: Latitude of target.
            lon: Longitude of target.
            alt: Relative altitude at target.
        """
        self.master.mav.command_int_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            0,
            1, -1, 0,
            radius, 0,
            int(lat * 1e7),
            int(lon * 1e7),
            alt
        )
        LOGGER.info("Reposition command: lat=%.7f lon=%.7f alt=%.2f radius=%.2f",
                    lat, lon, alt, radius)

    # ----------------------
    # Speed control
    # ----------------------
    def set_speed(self, speed: float) -> None:
        """
        Sets the global speed parameter (simulation/WIP).

        Args:
            speed: Desired speed in m/s (converted to cm/s for MAVLink).
        """
        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            b'WPNAV_SPEED',
            speed * 100,  # Convert m/s -> cm/s
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        LOGGER.info("Global speed set to %.2f m/s", speed)

    # ----------------------
    # ADS-B avoidance (AVOID_ADSB)
    # ----------------------
    def set_adsb_avoidance_params(self, config: ADSBAvoidanceConfig) -> None:
        """
        Configure ArduPilot's ADS-B based avoidance (AVOID_ADSB mode).

        There is no "radius" field on ADSB_VEHICLE itself -- ArduPilot
        projects the reported vehicle's position/velocity forward and
        compares it against these horizontal/vertical distance thresholds
        to decide whether a near-miss is imminent.

        Args:
            config: ADSBAvoidanceConfig describing the desired AVD_* params.
        """
        params = {
            "AVD_ENABLE": int(config.enable),
            "AVD_F_DIST_XY": config.fail_dist_xy,
            "AVD_F_DIST_Z": config.fail_dist_z,
            "AVD_F_TIME": config.fail_time,
            "AVD_F_ACTION": config.fail_action,
            "AVD_F_RCVRY": config.fail_recovery,
        }
        for name, value in params.items():
            self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                name.encode("ascii"),
                float(value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            LOGGER.info("Set %s = %s", name, value)

    # ----------------------
    # Heading / relative helpers
    # ----------------------
    def set_heading(self, heading: float, yaw_rate: float = 0) -> None:
        """
        Set absolute heading using MAV_CMD_CONDITION_YAW.

        Args:
            heading: Target yaw in degrees.
            yaw_rate: Desired yaw rate in deg/s (0 to use default).
        """
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            heading,
            yaw_rate,
            1,  # direction: 1=cw, -1=ccw
            0,  # absolute angle
            0,
            0,
            0,
        )
        LOGGER.info("Set heading to %.2f degrees", heading)

    def set_heading_relative(self, delta_heading: float, yaw_rate: float = 0) -> None:
        """
        Adjust heading relative to current using MAV_CMD_CONDITION_YAW.

        Args:
            delta_heading: Relative yaw change in degrees (positive cw).
            yaw_rate: Desired yaw rate in deg/s (0 to use default).
        """
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            delta_heading,
            yaw_rate,
            1,
            1,  # relative angle
            0,
            0,
            0,
        )
        LOGGER.info("Adjusted heading by %.2f degrees", delta_heading)

    def set_position_relative(self, d_north: float, d_east: float, d_alt: float = 0.0) -> None:
        """
        Move the vehicle by a relative offset in metres using global coordinates.

        Args:
            d_north: Distance to move north in metres.
            d_east: Distance to move east in metres.
            d_alt: Change in altitude in metres (positive up).
        """
        current = self.get_global_position()
        if current.alt is None:
            current_alt = 0.0
        else:
            current_alt = current.alt

        # Spherical Earth approximation (same as older Navigator)
        earth_radius = 6378137.0
        d_lat = d_north / earth_radius
        d_lon = d_east / (earth_radius * cos(pi * current.lat / 180.0))

        new_lat = current.lat + (d_lat * 180.0 / pi)
        new_lon = current.lon + (d_lon * 180.0 / pi)
        new_alt = current_alt + d_alt

        target = PositionSetpointGlobal(lat=new_lat, lon=new_lon, alt=new_alt)
        self.set_position_global(target)

    def set_servo(self, servo_number, pwm_value):
        """
        servo_number: Output channel (1-16)
        pwm_value:    PWM in microseconds (typically 1000-2000, center=1500)
        """
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,           # confirmation
            servo_number,  # param1: servo instance (output channel)
            pwm_value,     # param2: PWM value (microseconds)
            0, 0, 0, 0, 0  # params 3-7 unused
        )

    # ----------------------
    # ADSB
    # ----------------------
    def send_adsb_vehicle(self, vehicle: ADSBVehicle) -> None:
        """
        Send an ADSB_VEHICLE message to report a nearby aircraft's position
        to the autopilot (e.g. for ADS-B based collision avoidance).

        Args:
            vehicle: ADSBVehicle describing the traffic to report.
        """
        flags = (
            mavutil.mavlink.ADSB_FLAGS_VALID_COORDS
            | mavutil.mavlink.ADSB_FLAGS_VALID_ALTITUDE
            | mavutil.mavlink.ADSB_FLAGS_VALID_HEADING
            | mavutil.mavlink.ADSB_FLAGS_VALID_VELOCITY
            | mavutil.mavlink.ADSB_FLAGS_VERTICAL_VELOCITY_VALID
        )
        if vehicle.callsign:
            flags |= mavutil.mavlink.ADSB_FLAGS_VALID_CALLSIGN
        if vehicle.squawk:
            flags |= mavutil.mavlink.ADSB_FLAGS_VALID_SQUAWK

        self.master.mav.adsb_vehicle_send(
            vehicle.icao_address,
            int(vehicle.lat * 1e7),
            int(vehicle.lon * 1e7),
            vehicle.altitude_type,
            int(vehicle.altitude * 1000),
            int(vehicle.heading * 100) % 36000,
            int(vehicle.hor_velocity * 100),
            int(vehicle.ver_velocity * 100),
            vehicle.callsign.encode("ascii"),
            vehicle.emitter_type,
            vehicle.tslc,
            flags,
            vehicle.squawk,
        )
        LOGGER.info(
            "ADSB_VEHICLE sent: ICAO=%d lat=%.7f lon=%.7f alt=%.2f heading=%.1f",
            vehicle.icao_address, vehicle.lat, vehicle.lon,
            vehicle.altitude, vehicle.heading
        )

    def generate_typemask(self, ignores: Iterable[int]) -> int:
        """
        Generate a MAVLink type mask based on the bits to ignore (disable).

        Each bit position in `ignores` will be set to 1 in the resulting mask.

        Args:
            ignores (Iterable[int]): Bit positions to disable in the mask.

        Returns:
            int: Generated type mask.
        """
        mask = 0

        for bit in ignores:
            if bit < 0:
                raise ValueError(f"Bit positions must be non-negative, got {bit}")
            mask |= 1 << bit

        return mask
