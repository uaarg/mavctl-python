import time
from typing import Optional

from pymavlink import mavutil

from mavctl.connect.heartbeat import HeartbeatManager
from mavctl.exceptions import ConnError
from mavctl._internal.logger import get_logger

LOGGER = get_logger(__name__)

class Connect:
    """Handles MAVLink connections and heartbeat monitors"""

    def __init__(self,
                 ip: str = "udp:127.0.0.1:14552",
                 baud: int = 57600,
                 heartbeat_timeout: Optional[int] = None):

        """
        Initialize a MAVLink connection and run the heartbeat manager on the connection

        Args:
            ip: Connection String
            baud: Baud rate
            heartbeat_timeout: Optional timeout in seconds for waiting for heartbeats
        """
        self.master = self.connect(ip = ip, baud = baud, heartbeat_timeout = heartbeat_timeout)
        self._heartbeat_manager = HeartbeatManager(self.master)
        self._heartbeat_manager.start()

        self.ip = ip

    def connect(self, ip: str = "udp:127.0.0.1:14552",
                baud: int = 57600,
                heartbeat_timeout: Optional[int] = None) -> mavutil.mavlink_connection:
        """
        Function to connect to the MAVLink device

        Args:
            ip: Connection String
            baud: Baud rate
            heartbeat_timeout: Optional timeout in seconds for waiting for heartbeats

        Returns:
            A mavutil.mavlink_connection instance

        Raises:
            ConnError: If heartbeat or the connection fails
        """
        try:
            master = mavutil.mavlink_connection(ip, baud)
            msg_recv = master.recv_match(type="HEARTBEAT", blocking=False)
            while not msg_recv:
                i = 0
                if heartbeat_timeout is not None:
                    if heartbeat_timeout > 0:
                        i += 1
                        if heartbeat_timeout % 5:
                            print("MAVCTL: Waiting for heartbeat ...")
                    elif heartbeat_timeout == 0:
                        raise ConnError("MAVCTL ERROR: Heartbeat not found!")
                print("MAVCTL: Waiting for heartbeat ...")
                msg_recv = master.recv_match(type="HEARTBEAT", blocking=False)
                time.sleep(1)
            LOGGER.info("MAVCTL: Connected at %s", ip)
            return master

        except Exception as e:
            raise ConnError("MAVCTL ERROR: Failed to receive heartbeat!: ", e) from e

    def disconnect(self, timeout: float = 5, device: Optional[str] = None) -> bool:
        """Graceful Disconnect from the MAVLink Device

        Returns:
            Bool saying whether or not the disconnect was successful

        Raises ConnErorr if disconnect fails"""

        LOGGER.info("MAVCTL: Disconnecting ...")
        try:

            self._heartbeat_manager.stop()
            start_time = time.monotonic()
            while self._heartbeat_manager.get_connection_status():
                if (time.monotonic() - start_time) > timeout:
                    LOGGER.warning("MAVCTL: Heartbeat manager did not stop within timeout")
                    return False
            self.master.close()
            LOGGER.info("MAVCTL: Successfully Disconnected from %s", device)
            return True

        except Exception as e:
            raise ConnError("MAVCTL ERROR: Disconnect failed", e) from e
