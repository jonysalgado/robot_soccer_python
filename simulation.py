# ______________________________________________________________________________
# importation
import numpy as np
from pygame.rect import Rect
from pygame.gfxdraw import pie
from math import sin, cos, sqrt, pi
from simulation_soccer_2d.constants import *
from simulation_soccer_2d.utils import *
from simulation_soccer_2d.agents import *
from copy import deepcopy
import datetime

# ______________________________________________________________________________
# class Simulation

class Simulation:
    """
    Represents the simulation.
    """
    def __init__(self, player, ball, shockable, full_vision):
        """
        Creates the simulation.

        :param player: the robots used in this simulation.
        :type player: numpy.ndarray
        :param ball: the ball used in this simulation.
        :type ball: Ball
        :param shockable: if player will collide between themselves
        :type shockable: bool
        """
        self.player = player
        self.ball = ball
        self.shockable = shockable
        self.full_vision = full_vision
        self.left_goal = 0
        self.right_goal = 0
        self.goal = datetime.datetime.utcnow()
        self.initial_position = self.get_initial_position()
        
    def get_initial_position(self):
        """
        Get initial position of all players and ball for restart the game.

        :return: a list of players and ball's pose.
        :rtype: list
        """
        initial_position = [deepcopy(self.ball.pose)]
        for player in self.player:
            initial_position.append(deepcopy(player.pose))

        return initial_position

    # __________________________________________________________________________
    # methods for check collision players

    def check_collision(self, num):
        """
        Checks collision between the robot and the walls and the collision between 
        other players.

        :param num: the index of player in player's array.
        :type num: int
        :return: the bumper state (if a collision has been detected).
        :rtype: bool
        """
        params = self.get_collision_params(num)
        # Testing if the bounding box has hit a wall
        if params["left"] <= 0.0:
            self.player[num].pose.position.x = self.player[num].radius
            return True, "left", (0,0)
        if params["right"] >= params["width"]:
            self.player[num].pose.position.x = params["width"] - self.player[num].radius
            return True, "right", (0,0)
        if params["top"] <= 0.0:
            self.player[num].pose.position.y = self.player[num].radius
            return True, "top", (0,0)
        if params["bottom"] >= params["height"]:
            self.player[num].pose.position.y = params["height"] - self.player[num].radius
            return True, "bottom", (0,0)

        return self.check_collision_with_players(num)

        

    def get_collision_params(self, num = 0, isBall = False):
        """
        Get params for check collision.

        :param num: the index of player in player's array.
        :type num: int
        :return: the bumper state (if a collision has been detected).
        :rtype: bool
        """
        if not isBall:
            params = {
                # Converting screen limits from pixels to meters
                "width" : SCREEN_WIDTH * PIX2M,
                "height" : SCREEN_HEIGHT * PIX2M,
                # Computing the limits of the player's bounding box
                "left" : self.player[num].pose.position.x - self.player[num].radius,
                "right" : self.player[num].pose.position.x + self.player[num].radius,
                "top" : self.player[num].pose.position.y - self.player[num].radius,
                "bottom" : self.player[num].pose.position.y + self.player[num].radius
            }
        else:
            params = {
                # Converting screen limits from pixels to meters
                "width" : SCREEN_WIDTH * PIX2M,
                "height" : SCREEN_HEIGHT * PIX2M,
                # Computing the limits of the player's bounding box
                "left" : self.ball.pose.position.x - self.ball.radius,
                "right" : self.ball.pose.position.x + self.ball.radius,
                "top" : self.ball.pose.position.y - self.ball.radius,
                "bottom" : self.ball.pose.position.y + self.ball.radius
            }

        return params


    def check_collision_with_players(self, num):
        """
        Check collision with other player.

        :param num: the index of player in player's array.
        :type num: int
        :return: the bumper state (if a collision has been detected).
        :rtype: bool
        """
        bumper_state, n_player, speed = False, None, (0,0)
        if not self.shockable:
            return bumper_state, n_player, speed

        cont = 0
        for player in self.player:
            if cont != num:
                dist_players = self.player[num].pose.dist_square(player.pose)
                if dist_players <=(self.player[num].radius + player.radius):
                    bumper_state, n_player = True, cont
                    player.set_velocity(BACK_SPEED_COLISION,0)
                cont += 1
            
        return bumper_state, n_player, speed

    

    # __________________________________________________________________________
    # methods for check collision Ball
    
    def check_collision_ball(self):
        """
        Checks collision between the ball with the walls and other players. If the ball
        collide, then return where.

        :return: the bumper state (if a collision has been detected).
        :rtype: bool
        :return: where the ball collide
        :rtype: string or int or None
        """
        params = self.get_collision_params(isBall=True)
        # Testing if the bounding box has hit a wall
        if params["left"] <= 0.0: 
            self.ball.pose.position.x = self.ball.radius
            return True, "left", (0,0)
        if params["right"] >= params["width"]:
            self.ball.pose.position.x = params["width"] - self.ball.radius
            return True, "right", (0,0)
        if params["top"] <= 0.0:
            self.ball.pose.position.y = self.ball.radius
            return True, "top", (0,0)
        if params["bottom"] >= params["height"]:
            self.ball.pose.position.y = params["height"] - self.ball.radius
            return True, "bottom", (0,0)

        # check collision with other player
        return self.check_collision_between_ball_players()



    def check_collision_between_ball_players(self):
        """
        Check if ball collide with others players.

        :return: the bumper state (if a collision has been detected).
        :rtype: bool
        :return: the number of player that ball collide
        :rtype: int
        """

        velocityBall = TransformCartesian(self.ball.linear_speed, self.ball.pose.rotation)
        velocityBall = Vector2(velocityBall.x, velocityBall.y)

        bumper_state, n_player, speed = False, None, (0,0)
        cont = 0
        for players in self.player:
            dist_player = self.ball.pose.dist_square(players.pose)
            dirvector = self.ball.pose.position.dirvector(players.pose.position)
            dirvector.normalize()
            u = velocityBall.dot(dirvector)
            if u < 0 and dist_player <= (RADIUS_BALL + players.radius):
                bumper_state, n_player, speed = True, cont, self.calculate_speed(players, self.ball)
            cont += 1
        
        return bumper_state, n_player, speed
    
    def calculate_speed(self, collide_player, agent):
        """
        Calculate the velocity of agent after collision.

        :param collide_player: player that collide with agent.
        :type collide_player: Agent
        :param agent: agent that we want calculate the velocity
        :type num: Agent
        """
        if agent.linear_speed < 1.0e-2:
            agent.linear_speed = collide_player.linear_speed
        
        velocity_ball = TransformCartesian(agent.linear_speed, agent.pose.rotation)
        velocity_ball = Vector2(velocity_ball.x, velocity_ball.y)
        velocity_player = TransformCartesian(collide_player.linear_speed, collide_player.pose.rotation)
        velocity_player = Vector2(velocity_player.x, velocity_player.y)
        dirvector = agent.pose.position.dirvector(collide_player.pose.position)
        dirvector.normalize()
        u1 = velocity_ball.dot(dirvector)
        u1normal = Vector2(velocity_ball.x - u1 * dirvector.x,velocity_ball.y - u1 * dirvector.y)
        u2 = velocity_player.dot(dirvector)
        v1 = ((BALL_MASS - PLAYER_MASS) * u1 + 2 * BALL_MASS * u2) / ( BALL_MASS + PLAYER_MASS)
        vfinal = Vector2(u1normal.x + v1 * dirvector.x,u1normal.y + v1 * dirvector.y)
        vfinal = TransformPolar(2*vfinal.x, 2*vfinal.y)
        if u1 < 0:
            if vfinal.rotation > 1.0e-2:
                vfinal.rotation *= -1
            else:
                vfinal.rotation = pi
        
        return vfinal.linear_speed, vfinal.rotation

    # __________________________________________________________________________
    # methods for restart game if ball is in the goal
    
    def check_goal(self):
        """
        Check if there was a goal and restart the game if there was.
        """

        ball_position = Vector2(self.ball.pose.position.x * M2PIX, self.ball.pose.position.y * M2PIX)

        # left goal
        if round(ball_position.x - RADIUS_BALL) >= (round(SCREEN_WIDTH)-30) and round(ball_position.y - RADIUS_BALL) >= (round(SCREEN_HEIGHT)/2-100) and round(ball_position.y +  RADIUS_BALL) <= (round(SCREEN_HEIGHT)/2+100):
            if (datetime.datetime.now() - self.goal).seconds > 3:
                self.left_goal += 1
                self.goal = datetime.datetime.now()
                
        
        # Right goal
        if round(ball_position.x + RADIUS_BALL) <= 30 and round(ball_position.y - RADIUS_BALL) >= (round(SCREEN_HEIGHT)/2-100) and round(ball_position.y +  RADIUS_BALL) <= (round(SCREEN_HEIGHT)/2+100):
            if (datetime.datetime.now() - self.goal).seconds > 3:
                self.right_goal += 1
                self.goal = datetime.datetime.now()
        
        if (datetime.datetime.now() - self.goal).seconds < 1.0e-3:
            self.restart_game()


    def restart_game(self):
        """
        Restart the game and put the players and ball in its initial positions.
        """
        self.ball.pose = self.initial_position[0]
        self.ball.linear_speed = 0.0
        for i in range(1, len(self.initial_position)):
            self.player[i-1].pose = self.initial_position[i]
            self.player[i-1].linear_speed = 0.0
    
    # __________________________________________________________________________
    # method for control agents

    def set_commands(self, commands):
        """
        Sets commands.

        param commands: list of tuples for seting velocity of agents.
        type commands: list.
        """
        cont = 0
        for player in self.player:
            player.set_velocity(*commands[cont])
            cont += 1

    def get_sensors(self):
        """
        Get vector distance for flags, player and ball.

        return: list of sensors vector distances of each player.
        rtype: list
        """
        players_sensors = []
        for i in range(len(self.player)):
            list_center = []
            for j in range(len(self.player)):
                if j != i:
                    list_center.append((
                        self.player[j].pose.position.x * M2PIX,
                        self.player[j].pose.position.y * M2PIX))
            sensors_vector = self.player[i].sensors.calculate_distance(
                self.player[i], list_center)
            players_sensors.append(sensors_vector)
        
        return players_sensors


    # __________________________________________________________________________
    # method for update simulation
    def update(self):
        """
        Updates the simulation.
        """

        
        for i in range(len(self.player)):
            # update collision
            self.player[i].set_bumper_state_collision(self.check_collision(i))
            # Updating the player's movement
            self.player[i].update()
        
        # update ball's collision
        self.ball.set_bumper_state_collision(self.check_collision_ball())
        # Updating the ball's movement
        self.ball.update()
        
        self.check_goal()
        

    def draw(self, window, environment):
        """
        Draws the roomba and its movement history.

        :param window: pygame's window where the drawing will occur.
        :param environment: this param is to draw the window
        :type environment: Environment
        """
        list_centers = [np.array([round(M2PIX * self.ball.pose.position.x), round(M2PIX * self.ball.pose.position.y)])]
        list_radius = [round(M2PIX * RADIUS_BALL)]
        list_rotation = [self.ball.pose.rotation]
        for i in range(len(self.player)):
            list_centers.append(np.array([round(M2PIX * self.player[i].pose.position.x), round(M2PIX * self.player[i].pose.position.y)]))
            list_radius.append(round(M2PIX * self.player[i].radius))
            list_rotation.append(self.player[i].pose.rotation)
        params = {
            "window": window,
            "list_centers": list_centers,
            "list_radius": list_radius,
            "list_rotation": list_rotation,
            "left_goal": self.left_goal,
            "right_goal": self.right_goal

        }

        environment.draw(params)

        # # test
        # sensors = self.player[0].sensors
        # player_list = [(round(self.player[2].pose.position.x * M2PIX), round(self.player[2].pose.position.y * M2PIX))]
        # distances = sensors.calculate_distance(self.player[0], player_list)
        # cont = 0
        # for dist in distances:
        #     if dist.x != math.inf:
        #         v = Vector2(self.player[0].pose.position.x * M2PIX + dist.x, self.player[0].pose.position.y * M2PIX + dist.y)
        #         color = BLACK_COLOR
        #         if cont == 40:
        #             color = WHITE_COLOR
        #         pygame.draw.line(window, color, (self.player[0].pose.position.x * M2PIX, self.player[0].pose.position.y * M2PIX), (int(v.x), int(v.y)), 3)
        #     cont += 1

def draw(simulation, window, environment):
    """
    Redraws the pygame's window.

    :param simulation: the simulation object.
    :param window: pygame's window where the drawing will occur.
    """

    simulation.draw(window, environment)
    pygame.display.update()



