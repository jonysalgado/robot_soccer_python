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

# world Parameters
GRAVITY_ACCLERATION = 9.8

# Ball Parameters
FRICTION_SLOWDOWN = 0.5
REFLECTION = pi
BALL_MASS = 0.450
RADIUS_BALL = 0.05
FACTOR_FRICTION = GRAVITY_ACCLERATION*FRICTION_SLOWDOWN*SAMPLE_TIME/100

# players Parameters
PLAYER_MASS = 75

# Vision Parameters
RADIAN_TO_DEGREE = 180 / pi

# colors
RED_COLOR = (255,0,0)
GRAY_COLOR = (50, 50, 50)
YELLOW_COLOR = (238, 203, 0)
BLACK_COLOR = (0, 0, 0)
WHITE_COLOR = (255,255,255)
