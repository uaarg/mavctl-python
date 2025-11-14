import threading
import time
from typing import Callable, Optional
from pymavlink import mavutil


class HeartbeatManager:
    """
    Manages the heartbeat system between the ground station and the drone.
    Monitors connection status and handles reconnection attempts.
    """

    def __init__(
        self,
        mav: mavutil.mavlink.MAVLink_connection,
        heartbeat_timeout: float = 1.0,
        max_missed_heartbeats: int = 2,
    ) -> None:
        """
        Initialize the heartbeat manager.

        Args:
            mav: The MAVLink connection object.
            heartbeat_timeout: Seconds to wait before considering a heartbeat missed.
            max_missed_heartbeats: Number of consecutive missed heartbeats allowed.
        """
        self.mav = mav
        self.heartbeat_timeout = heartbeat_timeout
        self.max_missed_heartbeats = max_missed_heartbeats

        self.last_heartbeat: float = 0.0
        self.missed_heartbeats: int = 0
        self.is_connected: bool = False

        self._stop_event = threading.Event()
        self._monitor_thread: Optional[threading.Thread] = None

        self._on_connection_lost_callback: Optional[Callable[[], None]] = None
        self._on_connection_established_callback: Optional[Callable[[], None]] = None

    def start(
        self,
        on_connection_lost: Optional[Callable[[], None]] = None,
        on_connection_established: Optional[Callable[[], None]] = None,
    ) -> None:
        """
        Start the heartbeat monitoring thread.

        Args:
            on_connection_lost: Callback when connection is lost.
            on_connection_established: Callback when connection becomes established.
        """
        self._on_connection_lost_callback = on_connection_lost
        self._on_connection_established_callback = on_connection_established

        self._stop_event.clear()
        self._monitor_thread = threading.Thread(
            target=self._monitor_heartbeat, name="HeartbeatMonitor", daemon=True
        )
        self._monitor_thread.start()

    def stop(self) -> None:
        """Stop the heartbeat monitoring thread."""
        if self._monitor_thread is not None:
            self._stop_event.set()
            self._monitor_thread.join()
            self._monitor_thread = None

    def _monitor_heartbeat(self) -> None:
        """
        Monitor incoming heartbeat messages and update connection status.
        This runs in a background thread.
        """
        while not self._stop_event.is_set():
            msg = self.mav.recv_match(
                type="HEARTBEAT",
                blocking=True,
                timeout=self.heartbeat_timeout,
            )

            if msg is not None:
                # Heartbeat received
                self.last_heartbeat = time.time()
                self.missed_heartbeats = 0

                if not self.is_connected:
                    self.is_connected = True
                    if self._on_connection_established_callback:
                        self._on_connection_established_callback()
            else:
                # Missed heartbeat
                self.missed_heartbeats += 1

                if (
                    self.missed_heartbeats >= self.max_missed_heartbeats
                    and self.is_connected
                ):
                    self.is_connected = False
                    if self._on_connection_lost_callback:
                        self._on_connection_lost_callback()

    def get_connection_status(self) -> bool:
        """Return whether the connection is currently active."""
        return self.is_connected

    def get_last_heartbeat_time(self) -> float:
        """Return the timestamp of the last received heartbeat."""
        return self.last_heartbeat

    def get_missed_heartbeats(self) -> int:
        """Return the count of consecutively missed heartbeats."""
        return self.missed_heartbeats
