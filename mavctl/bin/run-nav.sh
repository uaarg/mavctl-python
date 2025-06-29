#!/usr/bin/env bash
set -e 
set +x
source venv/bin/activate

PYTHONUNBUFFERED=1 PYTHONPATH="." python3 tests/flight_tests/first_flight.py | tee --append shepard.log
