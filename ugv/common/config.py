"""
Shared robot + site parameters. ONE place, so the simulator, the visualiser's smooth "goal" robot
and the planner/follower all agree on how fast the robot can move. Edit the numbers here for your
real robot; every script reads them as its default (command-line flags still override).

Layout: ugv/common/config.py
"""

# ---- site ----------------------------------------------------------------------------------
# Latitude/longitude of the CENTRE of the map (the origin of the local x=east / y=north frame).
# Set this to your real site when you go to hardware.
ORIGIN_LAT = -31.9505
ORIGIN_LON = 115.8605
MAP_RES = 0.05            # metres per map pixel (cmap.png: 50 mm per pixel)

# ---- how the robot moves (used by the sim robot AND the visualiser's goal robot) -------------
MAX_SPEED = 1.0           # m/s   forward speed for arrow keys; the planner's cruise speed too
MAX_TURN = 1.2            # rad/s turn rate for arrow keys / planner
LIN_ACCEL = 1.0           # m/s^2 how quickly the robot reaches a commanded speed
ANG_ACCEL = 3.0           # rad/s^2
CMD_TIMEOUT = 0.5         # s     robot stops if no cmd_vel arrives for this long

# ---- robot body ------------------------------------------------------------------------------
WHEEL_BASE = 0.30         # m between wheels
WHEEL_DIAMETER = 0.15     # m
TICKS_PER_REV = 1000      # encoder ticks per wheel revolution
ROBOT_LENGTH = 0.45       # m footprint (used for drawing, collisions and obstacle inflation)
ROBOT_WIDTH = 0.35        # m
