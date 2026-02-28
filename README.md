mavctl-python

An Open Source Library for Autonomous Drone Navigation.
Things to be Familiar With

Before diving into mavctl, it is recommended that you read up on or familiarize yourself with the following so that you have an easier time working on mavctl

    MAVLink (https://mavlink.io/) Take a look through introduction just to get an idea of how the MAVLink protocol works

    Shepard (https://github.com/uaarg/shepard) Take a look through here (https://github.com/uaarg/shepard/blob/main/src/modules/autopilot/navigator.py) to get an idea of what the navigation functions should sort of look like Another thing to take a look at is the flight tests scripts as well (https://github.com/uaarg/shepard/blob/main/src/modules/autopilot/navigator.py)

    DroneKit (OPTIONAL) (https://github.com/dronekit/dronekit-python) While this is optional, a lot of things from DroneKit are borrowed, specifically the things that DroneKit does well. Overall, mavctl-python is similar to dronekit but it is revamped and is meant to replace DroneKit for all of its functionality. Over time we will reach a point where we will have exceeded dronekit but that time has not been reached just yet.

Contribution Guidelines

Push all of your changes to a seperate branch and make a pull request if you would like to merge to main

IMPORTANT: FOLLOW THE STRUCTURE. It is very important for you to follow the structure that mavctl-python follows in terms of its operation.

mavctl-python works as follows:

MAVLink connection is created and a PyMAVLink object is passed to the Navigator class. In the Navigator class, there are methods which conduct the drone navigation (ex. move the drone 1m North)

More advanced maneuvers should be located in their own files, but should NOT be in their own respective class. Instead, you can pass the Navigator class object into these advanced maneuver functions.

The structure of that would look as follows:

import advanced

master = connect()
nav = Navigator()

advanced_maneuver = advancedManeuver(nav)

advanced would be a file named advanced.py and advancedManeuver() would be the advanced maneuver method in question.
