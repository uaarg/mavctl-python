"""
Used to create specific exceptions to help with debugging
"""

class MAVCTLError(Exception):
    """Base exception for all mavctl errors"""

class ConnError(MAVCTLError):
    """Exception to handle Connection Errors"""
