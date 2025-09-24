from pymavlink import mavutil
from math import sqrt


'''
Classes adapted from here:
https://github.com/dronekit/dronekit-python/blob/master/dronekit/__init__.py

'''

class LocationGlobal(object):
    def __init__(self, lat, lon, alt=None):
        self.lat = lat
        self.lon = lon
        self.alt = alt

class LocationGlobalRelative(object):
    def __init__(self, lat, lon, alt=None):
        self.lat = lat
        self.lon = lon
        self.alt = alt

class LocationLocal(object):
    def __init__(self, north, east, down):
        self.north = north
        self.east = east
        self.down = down
        
class Altitude(object):
    def __init__(self, mono, amsl, local, relative, terrain, clearance):
        self.mono = mono
        self.amsl = amsl
        self.local = local
        self.relative = relative
        self.terrain = terrain
        self.clearance = clearance

class Battery(object):
    
    def __init__(self, voltage, current, level):
        self.voltage = voltage / 1000.0
        if current == -1:
            self.current = None
        else:
            self.current = current / 100.0

        if level == -1:
            self.level = None
        else:
            self.level = level

class Velocity(object):
    def __init__(self, north, east, down):
        self.north = north
        self.east = east
        self.down = down

    def magnitude(self):
        return sqrt(self.north ** 2 + self.east ** 2 + self.down ** 2)
