import threading
import time
from typing import Callable
from pymavlink import mavutil

class HeartbeatManager:
    """
    Manages the heartbeat system between the ground station and the drone.
    Monitors connection status and handles reconnection attempts.
    """
    def __init__(self, mav, heartbeat_timeout: float = 1.0, max_missed_heartbeats: int = 2):
        """
        Initialize the heartbeat manager.
        
        Args:
            mav: The MAVLink connection object
            heartbeat_timeout: Time in seconds to wait for a heartbeat before considering it missed
            max_missed_heartbeats: Number of consecutive heartbeats that can be missed before connection is considered lost
        """
        self.mav = mav
        self.heartbeat_timeout = heartbeat_timeout
        self.max_missed_heartbeats = max_missed_heartbeats
        self.last_heartbeat = 0
        self.missed_heartbeats = 0
        self.is_connected = False
        self._stop_event = threading.Event()
        self._monitor_thread = None
        self._on_connection_lost_callback = None
        self._on_connection_established_callback = None

    def start(self, on_connection_lost: Callable = None, on_connection_established: Callable = None):
        """
        Start the heartbeat monitoring system.
        
        Args:
            on_connection_lost: Callback function to be called when connection is lost
            on_connection_established: Callback function to be called when connection is established
        """
        self._on_connection_lost_callback = on_connection_lost
        self._on_connection_established_callback = on_connection_established
        self._stop_event.clear()
        self._monitor_thread = threading.Thread(target=self._monitor_heartbeat)
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def stop(self):
        """Stop the heartbeat monitoring system."""
        if self._monitor_thread:
            self._stop_event.set()
            self._monitor_thread.join()
            self._monitor_thread = None

    def _monitor_heartbeat(self):
        """Monitor heartbeat messages and manage connection status."""
        while not self._stop_event.is_set():
            msg = self.mav.recv_match(type="HEARTBEAT", blocking=True, timeout=self.heartbeat_timeout)
            
            if msg is not None:
                self.last_heartbeat = time.time()
                self.missed_heartbeats = 0
                if not self.is_connected:
                    self.is_connected = True
                    if self._on_connection_established_callback:
                        self._on_connection_established_callback()
            else:
                self.missed_heartbeats += 1
                if self.missed_heartbeats >= self.max_missed_heartbeats and self.is_connected:
                    self.is_connected = False
                    if self._on_connection_lost_callback:
                        self._on_connection_lost_callback()

    def get_connection_status(self) -> bool:
        """Get the current connection status."""
        return self.is_connected

    def get_last_heartbeat_time(self) -> float:
        """Get the timestamp of the last received heartbeat."""
        return self.last_heartbeat

    def get_missed_heartbeats(self) -> int:
        """Get the number of consecutively missed heartbeats."""
        return self.missed_heartbeats
