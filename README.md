# mavctl-python

An Open Source Library for Autonomous Drone Navigation.


## Contribution Guidelines

Push all of your changes to a seperate branch and make a pull request if you would like to merge to main

IMPORTANT: FOLLOW THE STRUCTURE.
It is very important for you to follow the structure that `mavctl-python` follows in terms of its operation.

`mavctl-python` works as follows:

`MAVLink` connection is created and a `PyMAVLink` object is passed to the `Navigator` class.
In the `Navigator` class, there are methods which conduct the drone navigation (ex. move the drone 1m North)

More advanced maneuvers should be located in their own files, but should NOT be in their own respective class. 
Instead, you can pass the `Navigator` class object into these advanced maneuver functions.

The structure of that would look as follows:

```
import advanced

master = connect()
nav = Navigator()

advanced_maneuver = advancedManeuver(nav)
```

`advanced` would be a file named `advanced.py` and `advancedManeuver()` would be the advanced maneuver method in question.


