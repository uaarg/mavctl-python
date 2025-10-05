import sys
import subprocess


class SitlVehicle:
    conn_str: str
    _process: subprocess.Popen

    def __init__(self):
        """Start a SITL instance. Returns a connection string to access that SITL vehicle"""
        self._process = subprocess.Popen(
            ['docker', 'run', '-p', '5760:5760', '--rm', '-it', 'uaarg-sitl'],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )

        # Wait for the process to finish booting
        while line := self._process.stdout.readline():
            print(line, end='\r\n')
            if line == b'SIM_VEHICLE: Waiting for SITL to exit\r\n':
                print("SITL Started")
                break

        self.conn_str = 'tcp:127.0.0.1:5760'

    def __enter__(self):
        return self

    def __exit__(self, _type, _value, _traceback):
        self.stop()

    def stop(self):
        """Stop the sitl/simulator process"""
        self._process.kill()
        self._process.wait(timeout=5)
