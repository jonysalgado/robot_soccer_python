# ______________________________________________________________________________
# importation
import pygame
import numpy as np
from pygame.rect import Rect
from pygame.gfxdraw import pie
from math import sin, cos, fabs, acos, pi, inf
from simulation_soccer_2d.constants import *
from simulation_soccer_2d.utils import *

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
        self.linear_speed = 1.0
        self.angular_speed = 0.0
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.radius = radius
        self.bumper_state = False
        self.collision = None
        self.collision_player_speed = (0,0)
    
    
    def set_velocity(self, linear_speed, angular_speed):
        """
        Sets the robot's velocity.

        :param linear_speed: the robot's linear speed.
        :type linear_speed: float
        :param angular_speed: the robot's angular speed.
        :type angular_speed: float
        """
        self.linear_speed = clamp(linear_speed, -self.max_linear_speed, 
            self.max_linear_speed)
        self.angular_speed = clamp(angular_speed, -self.max_angular_speed, 
            self.max_angular_speed)

    def set_bumper_state_collision(self, bumper_state_collision):
        """
        Sets the bumper state and where agent collide.

        :param bumper_state_collision: if the bumper has detected an obstacle and where agent collide.
        :type bumper_state_collision: tuple
        """
        self.bumper_state, self.collision, self.collision_player_speed = bumper_state_collision


    def get_bumper_state(self):
        """
        Obtains the bumper state.

        :return: the bumper state.
        :rtype: bool
        """
        return self.bumper_state

    def get_collision(self):
        """
        Obtains the collision.

        :return: where agent collide.
        :rtype: string or int or None
        """
        return self.collision
    
    def get_collision_player_speed(self):
        return self.collision_player_speed

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
            self.pose.position.x += ((2.0 * v / w) * 
                cos(self.pose.rotation + w * dt / 2.0) * sin(w * dt / 2.0))
            self.pose.position.y += ((2.0 * v / w) * 
                sin(self.pose.rotation + w * dt / 2.0) * sin(w * dt / 2.0))
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
    def __init__(self, pose, max_linear_speed, max_angular_speed, radius):
        Agent.__init__(self, pose, max_linear_speed, max_angular_speed, radius)
        self.sensors = Sensors(self)
# ______________________________________________________________________________
# class Ball
    
class Ball(Agent):
    """
    Represents a ball.
    """
    def __init__(self, pose, max_linear_speed, max_angular_speed, radius, behavior):
        Agent.__init__(self, pose, max_linear_speed, max_angular_speed, radius)
        self.behavior = behavior
        self.cont_friction = 0

    def set_rotation(self, increase):
            self.pose.rotation += increase
    
    def set_cont_friction(self, initial, increase):
        if initial:
            self.cont_friction = 0
        else: 
            self.cont_friction += increase

    def update(self):
        self.behavior.update(self)

 # ______________________________________________________________________________
# class Sensors
class Sensors:
    """
    Represents the sensors of a player.
    """
    def __init__(self, agent):
        self.flag_points = self.init_flag_points()
        self.agent_center = agent.pose
        self.full_vision = None

    def set_full_vision(self, full_vision):
        self.full_vision = full_vision

    def init_flag_points(self):
        """
        Find the flags around the field.

        return: a list of points.
        rtype: list
        """
        points = []
        for i in range(11):
            points.append((round(SCREEN_WIDTH * i/10), 0))
        for i in range(1,11):
            points.append((SCREEN_WIDTH, round(SCREEN_HEIGHT * i/10)))
        for i in range(10):
            points.append((round(SCREEN_WIDTH * i/10), SCREEN_HEIGHT))
        for i in range(1,10):
            points.append((0, round(SCREEN_HEIGHT * i/10)))
        
        

        return points

    def calculate_distance(self, agent, list_centers):
        """
        Calculate the vector distance between agent and other players, ball and flags.

        param agent: the agent that we are calculating the distances.
        type agent: Player
        param list_centers: the list of center's position that players and ball.
        type list_centers: Pose.position
        return: list of distance to points
        rtype: list
        """

        self.agent_center = agent.pose
        points = self.flag_points + list_centers
        dirvector_list = []
        for point in points:
            center = Vector2(self.agent_center.position.x * M2PIX,
                self.agent_center.position.y * M2PIX)
            dirvector = Vector2(*point).dirvector(center)
            dirvector = self.is_visible(dirvector)
            dirvector_list.append(dirvector)
        
        return dirvector_list

    def is_visible(self, vector):
        """
        Checks if a point is visible for a agent.

        param vector: vector that links the center of the agent to the point in question.
        type vector: Vector2
        return: the same vector if is visible and infinity vector if isn't.
        rtype: Vector2
        """
        if not self.full_vision:
            vector_agent = TransformCartesian(1, self.agent_center.rotation)
            vector_agent = Vector2(vector_agent.x, vector_agent.y)
            angle = acos(vector_agent.dot(vector)/vector.magnitude())

            if angle <= pi/4:
                return vector

            return Vector2(inf, inf)
        
        return vector
        



# ______________________________________________________________________________
# class Environment
class Environment:
    """
    Represents the environment of simulation.
    """
    def __init__(self, window):
        self.window = window
        self.logo = pygame.image.load('simulation_soccer_2d/team_logo.xpm')
        self.font = pygame.font.SysFont('Comic Sans MS', 30)
        self.list_centers = None
        self.list_radius = None
        self.list_rotation = None

    def draw(self, params):
        """
        This method call all other methods for drawing.

        :param params: params for drawing the window.
        """
        self.update(params)
        self.draw_field()
        self.draw_players_and_ball()
        self.draw_soccer_goal_and_scoreboard()
        self.draw_vision()
        
    def draw_players_and_ball(self):
        """
        Drawing players and ball.

        :param params: params for drawing the window.
        """

        # draw players
        for i in range(1, len(self.list_centers)):
            center = self.list_centers[i]
            final_position = self.list_radius[i] * np.array([cos(self.list_rotation[i]), 
                sin(self.list_rotation[i])]) + center
            if i <= len(self.list_centers)/2:
                color = RED_COLOR
            else:
                color = YELLOW_COLOR
            # Drawing player's inner circle
            pygame.draw.circle(self.window, color, (center[0], center[1]), 
                self.list_radius[i], 0)
            # Drawing player's outer circle
            pygame.draw.circle(self.window, GRAY_COLOR, (center[0], center[1]), 
                self.list_radius[i], 4)
            # Drawing player's orientation
            pygame.draw.line(self.window, GRAY_COLOR, (center[0], center[1]), 
                (final_position[0], final_position[1]), 3)


        # draw ball
        center = self.list_centers[0]
        # Drawing player's inner circle
        pygame.draw.circle(self.window, WHITE_COLOR, (center[0], center[1]), 
            self.list_radius[0], 0)

    def draw_field(self):
        """
        Drawing soccer field.

        :param window: pygame's window where the drawing will occur.
        """
        self.window.fill((35,142,35))
        

        pygame.draw.circle(self.window, (255,255,255), (round(SCREEN_WIDTH/2), 
            round(SCREEN_HEIGHT/2)), 70, 3)
        pygame.draw.line(self.window, (255,255,255), (round(SCREEN_WIDTH/2), 30), 
            (round(SCREEN_WIDTH/2), SCREEN_HEIGHT - 30), 3)
        pygame.draw.line(self.window, (255,255,255), (30, 30), 
            (round(SCREEN_WIDTH)-30, 30), 3)
        pygame.draw.line(self.window, (255,255,255), (30, 30), 
            (30, round(SCREEN_HEIGHT)-30), 3)
        pygame.draw.line(self.window, (255,255,255), (round(SCREEN_WIDTH)-30, 30), 
            (round(SCREEN_WIDTH)-30, round(SCREEN_HEIGHT)-30), 3)
        pygame.draw.line(self.window, (255,255,255), (30, round(SCREEN_HEIGHT)-30), 
            (round(SCREEN_WIDTH)-30, round(SCREEN_HEIGHT)-30), 3)

       
    def draw_soccer_goal_and_scoreboard(self):
        """
        Drawing soccer goal and scoreboard.
        """
        scoreboard="Left " + str(self.left_goal) + " x " + str(self.right_goal) + " Right"
        textsurface = self.font.render(scoreboard, False, WHITE_COLOR)
        # Drawing soccer goal
        pygame.draw.rect(self.window, (0, 0, 0), 
            Rect(0, round(SCREEN_HEIGHT)/2-100, 30, 200))
        pygame.draw.rect(self.window, (0, 0, 0), 
            Rect(round(SCREEN_WIDTH)-30, round(SCREEN_HEIGHT)/2-100, 30, 200))
        # scoreboard
        pygame.draw.rect(self.window, (0, 0, 0), 
            Rect(28, round(SCREEN_HEIGHT-30), 250, 30))

        self.window.blit(self.logo, (round(SCREEN_WIDTH)/2+100,40))
        self.window.blit(textsurface, (40,round(SCREEN_HEIGHT-20)))
        
    def draw_vision(self):
        """
        Drawing the vision of the players.

        :param params: params for drawing the window.
        """

        for i in range(1, len(self.list_centers)):
            center = self.list_centers[i]
            pie(self.window, center[0], center[1], round(2.5 * self.list_radius[i]), 
                (int(RADIAN_TO_DEGREE * self.list_rotation[i])-45)%360, 
                (int(RADIAN_TO_DEGREE * self.list_rotation[i])+45)%360 , WHITE_COLOR)
        
    def update(self, params):
        """
        Update params of environment.

        :param params: params for drawing the window.
        """

        self.window = params["window"]
        self.list_centers = params["list_centers"]
        self.list_radius = params["list_radius"]
        self.list_rotation = params["list_rotation"]
        self.left_goal = params["left_goal"]
        self.right_goal = params["right_goal"]