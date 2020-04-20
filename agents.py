# ______________________________________________________________________________
# importation

from math import sin, cos, fabs
from constants import SAMPLE_TIME
from utils import *

# ______________________________________________________________________________
# class Agent

class Agent:
    def __init__(self, pose, max_linear_speed, max_angular_speed, radius):
        """
        Creates a roomba cleaning robot.

        :param pose: the robot's initial pose.
        :type pose: Pose
        :param max_linear_speed: the robot's maximum linear speed.
        :type max_linear_speed: float
        :param max_angular_speed: the robot's maximum angular speed.
        :type max_angular_speed: float
        :param radius: the robot's radius.
        :type radius: float
        :param bumper_state: its mean if robot colide with other robots or wall
        :type bumper_state: boolean
        """
        self.pose = pose
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.radius = radius
        self.bumper_state = False
    
    def set_velocity(self, linear_speed, angular_speed):
        """
        Sets the robot's velocity.

        :param linear_speed: the robot's linear speed.
        :type linear_speed: float
        :param angular_speed: the robot's angular speed.
        :type angular_speed: float
        """
        self.linear_speed = clamp(linear_speed, -self.max_linear_speed, self.max_linear_speed)
        self.angular_speed = clamp(angular_speed, -self.max_angular_speed, self.max_angular_speed)

    def set_bumper_state(self, bumper_state):
        """
        Sets the bumper state.

        :param bumper_state: if the bumper has detected an obstacle.
        :type bumper_state: bool
        """
        self.bumper_state = bumper_state

    def get_bumper_state(self):
        """
        Obtains the bumper state.

        :return: the bumper state.
        :rtype: bool
        """
        return self.bumper_state
    
    def move(self):
        """
        Moves the robot during one time step.
        """
        dt = SAMPLE_TIME
        v = self.linear_speed 
        w = self.angular_speed

        # If the angular speed is too low, the complete movement equation fails due to a division by zero.
        # Therefore, in this case, we use the equation we arrive if we take the limit when the angular speed
        # is close to zero.
        if fabs(self.angular_speed) < 1.0e-3:
            self.pose.position.x += v * dt * cos(self.pose.rotation + w * dt / 2.0)
            self.pose.position.y += v * dt * sin(self.pose.rotation + w * dt / 2.0)
        else:
            self.pose.position.x += (2.0 * v / w) * cos(self.pose.rotation + w * dt / 2.0) * sin(w * dt / 2.0)
            self.pose.position.y += (2.0 * v / w) * sin(self.pose.rotation + w * dt / 2.0) * sin(w * dt / 2.0)
        self.pose.rotation += w * dt

    def update(self):
        """
        Updates the robot, including its behavior.
        """
        self.move()

# ______________________________________________________________________________
# class Player

class Player(Agent):
    """
    Represents a player robot.
    """
    pass

# ______________________________________________________________________________
# class Ball
    
class Ball(Agent):
    """
    Represents a roomba cleaning robot.
    """

    def set_rotation(self, increase):
            self.pose.rotation += increase
    
    def set_cont_friction(self, initial, increase=0):
        if initial:
            self.cont_friction = 0
        else: 
            self.cont_friction += increase


