from math import radians, degrees, cos, sin, atan2, sqrt
from mavctl.messages.location import LocationLocal


def distance_to_target(target1, target2) -> LocationLocal:
    """
    Returns the distance vector to the target.

    Args:
        target1: Current Position of Vehicle (LocationLocal)
        target2: Target Position (LocationLocal)
    """
    dx = target2.north - target1.north
    dy = target2.east - target1.east
    dz = target2.down - target1.down

    return LocationLocal(dx, dy, dz)


def check_target_reached(target1, target2, tolerance: float = 0.5) -> bool:
    """
    Checks if the vehicle has reached the target within a certain tolerance.

    Args:
        target1: Current Position of the vehicle (LocationLocal)
        target2: Target Position (LocationLocal)
        tolerance: Distance tolerance in meters
    """
    delta = distance_to_target(target1, target2)
    delta_length = sqrt(delta.north ** 2 + delta.east ** 2 + delta.down ** 2)

    return delta_length < tolerance


def LatLon_to_Distance(point, origin) -> float:
    """
    Computes the distance between two global coordinates using Haversine formula.

    Args:
        point: Target LocationGlobal
        origin: Origin LocationGlobal

    Returns:
        Distance in meters
    """
    R = 6378137  # Earth radius in meters

    origin_lat_rad = radians(origin.lat)
    origin_lon_rad = radians(origin.lon)
    point_lat_rad = radians(point.lat)
    point_lon_rad = radians(point.lon)

    delta_lat = point_lat_rad - origin_lat_rad
    delta_lon = point_lon_rad - origin_lon_rad

    a = sin(delta_lat / 2) ** 2 + cos(origin_lat_rad) * cos(point_lat_rad) * sin(delta_lon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    return R * c


def Heading(point1, point2) -> float:
    """
    Computes the heading from point1 to point2 in degrees (0-360).

    Args:
        point1: Start LocationGlobal
        point2: End LocationGlobal

    Returns:
        Heading in degrees
    """
    lat1_rad = radians(point1.lat)
    lat2_rad = radians(point2.lat)
    delta_lon = radians(point2.lon - point1.lon)

    x = sin(delta_lon) * cos(lat2_rad)
    y = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(delta_lon)

    angle = atan2(x, y)
    return (degrees(angle) + 360) % 360
