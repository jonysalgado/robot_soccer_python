from math import pi
# Simulation Parameters
# soccer field (105,68) x 6
SCREEN_WIDTH = 1000 # 636
SCREEN_HEIGHT = 650 # 414
PIX2M = 0.01  # factor to convert from pixels to meters
M2PIX = 100.0  # factor to convert from meters to pixels

# Sample Time Parameters
FREQUENCY = 60.0  # simulation frequency
SAMPLE_TIME = 1.0 / FREQUENCY  # simulation sample time

# Ball Parameters
FRICTION_SLOWDOWN = 0.5
REFLECTION = pi
BALL_MASS = 0.450
RADIUS_BALL = 0.05

# players Parameters
RADIUS_PLAYER = 0.34 / 2.0
# NUM_1 = 0
# NUM_2 = 1
PLAYER_MASS = 75

# Vision Parameters
RADIAN_TO_DEGREE = 180 / pi

# world Parameters
GRAVITY_ACCLERATION = 9.8
