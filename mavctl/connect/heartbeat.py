"""Heartbeat monitoring system for drone-ground station connection management.

This module provides the HeartbeatManager class, which monitors MAVLink heartbeat
messages from the drone and automatically detects connection loss based on missed
heartbeats. It supports callback-based notifications for connection state changes.
"""
# pylint: disable=too-many-instance-attributes

import threading
import time
from typing import Callable
from pymavlink import mavutil

class HeartbeatManager:
    """Monitor MAVLink heartbeat messages and detect connection loss.

    Runs a background thread that listens for HEARTBEAT messages from the drone.
    Triggers callbacks when connection is established or lost after N missed
    heartbeats.
    """
    def __init__(self, master, heartbeat_timeout: float = 1.0, max_missed_heartbeats: int = 2):
        """Initialize the heartbeat manager.

        Args:
            mav: The MAVLink connection object
            heartbeat_timeout: Time in seconds to wait for a heartbeat before
                considering it missed (default 1.0).
            max_missed_heartbeats: Number of consecutive heartbeats that can be
                missed before connection is considered lost (default 2).
        """
        self.master = master
        self.heartbeat_timeout = heartbeat_timeout
        self.max_missed_heartbeats = max_missed_heartbeats
        self.last_heartbeat = 0
        self.missed_heartbeats = 0
        self.is_connected = False
        self._stop_event = threading.Event()
        self._monitor_thread = None
        self._send_thread = None
        self._callbacks = {
            'on_connection_lost': None,
            'on_connection_established': None,
        }

    def start(self, on_connection_lost: Callable = None,
              on_connection_established: Callable = None):
        """Start the heartbeat monitoring system.

        Args:
            on_connection_lost: Callback function called when connection is lost.
            on_connection_established: Callback function called when connection
                is established.
        """
        self._callbacks['on_connection_lost'] = on_connection_lost
        self._callbacks['on_connection_established'] = on_connection_established
        self._stop_event.clear()
        self._monitor_thread = threading.Thread(target=self._monitor_heartbeat)
        self._monitor_thread.daemon = True
        self._send_thread = threading.Thread(target=self._send_heartbeat)
        self._send_thread.daemon = True
        self._monitor_thread.start()
        self._send_thread.start()

    def stop(self):
        """Stop the heartbeat monitoring system."""
        if self._monitor_thread:
            self._stop_event.set()
            self._monitor_thread.join()
            self._send_thread.join()
            self._monitor_thread = None
            self._send_thread = None

    def _monitor_heartbeat(self):
        """Monitor heartbeat messages and manage connection status."""
        while not self._stop_event.is_set():
            msg = self.master.recv_match(
                type="HEARTBEAT",
                blocking=True,
                timeout=self.heartbeat_timeout
            )

            if msg is not None:
                self.last_heartbeat = time.time()
                self.missed_heartbeats = 0
                if not self.is_connected:
                    self.is_connected = True
                    callback = self._callbacks['on_connection_established']
                    if callback is not None:
                        callback()
            else:
                self.missed_heartbeats += 1
                if (self.missed_heartbeats >= self.max_missed_heartbeats
                        and self.is_connected):
                    self.is_connected = False
                    callback = self._callbacks['on_connection_lost']
                    if callback is not None:
                        callback()

    def _send_heartbeat(self):
        """Send heartbeat messages"""
        while not self._stop_event.is_set():
            self.master.mav.heartbeat_send(
                    type=mavutil.mavlink.MAV_TYPE_GCS,
                    autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    base_mode=0,
                    custom_mode=0,
                    system_status=mavutil.mavlink.MAV_STATE_ACTIVE
                )
            time.sleep(self.heartbeat_timeout)

    def get_connection_status(self) -> bool:
        """Get the current connection status."""
        return self.is_connected

    def get_last_heartbeat_time(self) -> float:
        """Get the timestamp of the last received heartbeat."""
        return self.last_heartbeat

    def get_missed_heartbeats(self) -> int:
        """Get the number of consecutively missed heartbeats."""
        return self.missed_heartbeats
