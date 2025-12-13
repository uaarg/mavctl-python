import logging
import pymavlink.dialects.v20.all as dialect
from mavctl.connect.conn import Connect

LOGGER = logging.getLogger(__name__)
LOGGER.setLevel(logging.DEBUG)

if not LOGGER.handlers:
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)
    formatter = logging.Formatter(
        fmt="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S"
    )
    ch.setFormatter(formatter)
    LOGGER.addHandler(ch)


class Messenger:
    """
    Messenger class for sending messages from the autopilot to the ground station.

    Maintains a separate MAVLink connection for sending status text messages.
    """

    def __init__(self, ip: str = "udp:127.0.0.1:14553") -> None:
        """
        Initialize a MAVLink connection.

        Args:
            ip: Connection string to connect to, must be different than the drone connection.
        """
        LOGGER.info("MAVCTL: Messenger connecting to %s", ip)
        # Explicitly define baud and heartbeat_timeout here
        self.master = Connect(ip=ip, baud=57600, heartbeat_timeout=5).master
        LOGGER.info("MAVCTL: Messenger connected to %s", ip)

    def send(self, message: str, prefix: str = "MAVCTL") -> None:
        """
        Sends a message to the ground station.

        Args:
            message: The message to be sent.
            prefix: The device it's coming from (the name of the drone or the software).
        """
        try:
            full_message = f"{prefix}: {message}"
            mav_message = dialect.MAVLink_statustext_message(
                severity=dialect.MAV_SEVERITY_INFO,
                text=full_message.encode("utf-8")
            )
            self.master.mav.send(mav_message)
        except Exception as e:
            LOGGER.error("Messenger failed to send message: %s", e)
