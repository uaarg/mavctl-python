import time
from math import atan
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
        type_mask = self.master.generate_typemask([0, 1, 2, 9])
        yaw_angle = atan(y / x) if x != 0 else 0
        setpoint = PositionSetpointLocal(x=x, y=y, z=-z, yaw=yaw_angle)
        self.set_position_local_ned(setpoint, type_mask=type_mask)
        self.wait_target_reached(setpoint)

    def simple_goto_global(self, lat: float, lon: float, alt: float) -> None:
        """Move drone to global coordinates."""
        type_mask = self.generate_typemask([0, 1, 2, 9])
        start_point = self.get_global_position()
        yaw = util.Heading(start_point, LocationGlobal(lat, lon, alt))
        setpoint = PositionSetpointGlobal(lat=lat, lon=lon, alt=alt, yaw=yaw)
        self.set_position_global(setpoint, type_mask=type_mask)
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

    def generate_typemask(self, keeps: Iterable[int]) -> int:
        """
        Generate a MAVLink type mask based on the bits to keep (enable).

        Each bit position in `keeps` will be set to 1 in the resulting mask.

        Args:
            keeps (Iterable[int]): Bit positions to enable in the mask.

        Returns:
            int: Generated type mask.
        """
        mask = 0

        for bit in keeps:
            if bit < 0:
                raise ValueError(f"Bit positions must be non-negative, got {bit}")
            mask |= 1 << bit

        return mask
