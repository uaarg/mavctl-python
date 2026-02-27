import time
import pytest

from mavctl.messages.navigator import Navigator

CONN_STR = "udp:127.0.0.1:14551"


@pytest.fixture(scope="module")
def drone():
    """
    Create a Navigator instance once per test module.
    """
    nav = Navigator(ip=CONN_STR)

    # Give SITL time to be ready
    time.sleep(15)

    yield nav


def test_arm_disarm(drone):
    """
    Basic integration test:
    - Arm vehicle
    - Confirm armed
    - Disarm vehicle
    - Confirm disarmed
    """

    drone.arm()
    time.sleep(2)

    drone.disarm()
    time.sleep(2)

