# ______________________________________________________________________________
# importation
import pygame
import numpy as np
from pygame.rect import Rect
from pygame.gfxdraw import pie
from math import sin, cos, sqrt
from constants import *
from utils import *
from agents import *
import datetime

# ______________________________________________________________________________
# class Simulation

class Simulation:
    """
    Represents the simulation.
    """
    def __init__(self, player, ball, shockable = True):
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
        initial_position = [self.ball.pose]
        for i in range(len(self.player)):
            initial_position.append(self.player[i].pose)
        
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
            return True
        if params["right"] >= params["width"]:
            self.player[num].pose.position.x = params["width"] - self.player[num].radius
            return True
        if params["top"] <= 0.0:
            self.player[num].pose.position.y = self.player[num].radius
            return True
        if params["bottom"] >= params["height"]:
            self.player[num].pose.position.y = params["height"] - self.player[num].radius
            return True

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

        bumper_state = False
        for i in range(len(self.player)):
            if i != num:
                dist_players = self.player[num].pose.dist_square(self.player[i].pose)
                if dist_players <=(self.player[num].radius + self.player[i].radius):
                    bumper_state = True
        
        return bumper_state and self.shockable
    

    # __________________________________________________________________________
    # methods for check collision Ball
    
    def check_collision_ball(self):
        """
        Checks collision between the ball with the walls and other players.

        :return: the bumper state (if a collision has been detected).
        :rtype: bool
        """
        params = self.get_collision_params(isBall=True)
        # Testing if the bounding box has hit a wall
        if params["left"] <= 0.0: 
            self.ball.pose.position.x = self.ball.radius
            return True
        if params["right"] >= params["width"]:
            self.ball.pose.position.x = params["width"] - self.ball.radius
            return True
        if params["top"] <= 0.0:
            self.ball.pose.position.y = self.ball.radius
            return True
        if params["bottom"] >= params["height"]:
            self.ball.pose.position.y = params["height"] - self.ball.radius
            return True

        # check collision with other player
        return self.check_collision_between_ball_players() or self.check_velocity_collision()


    def check_collision_between_ball_players(self, num=-1):
        """
        Check collision with ball and other player.

        :param num: the index of player in player's array.
        :type num: int
        :return: the bumper state (if a collision has been detected).
        :rtype: bool
        """
        bumper_state = False
        if num not in range(len(self.player)):
            for i in range(len(self.player)):
                dist_player = self.ball.pose.dist_square(self.player[i].pose)
                if dist_player <=(RADIUS_BALL + self.player[i].radius):
                    bumper_state = True
        else:
            dist_player = self.ball.pose.dist_square(self.player[num].pose)
            if dist_player <=(RADIUS_BALL + self.player[num].radius):
                bumper_state = True
        
        return bumper_state


    def check_velocity_collision(self):
        """
        Check if ball will collide or if ball has already collided.

        :return: the bumper state (if a collision has been detected).
        :rtype: bool
        """

        velocityBall = TransformCartesian(self.ball.linear_speed, self.ball.pose.rotation)
        velocityBall = Vector2(velocityBall.x, velocityBall.y)

        for i in range(len(self.player)):
            dist_player = self.ball.pose.dist_square(self.player[i].pose)
            dirvector = Vector2(self.ball.pose.position.x - self.player[i].pose.position.x, self.ball.pose.position.y - self.player[i].pose.position.y)
            dirvector.normalize()
            u = velocityBall.dot(dirvector)
            if u > 0 and dist_player <=(RADIUS_BALL + self.player[i].radius):
                return False
        
        return True
    
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
        for i in range(len(self.initial_position) - 1):
            self.player[i+1].pose = self.initial_position[i+1]
            self.player[i+1].linear_speed = 0.0

    # __________________________________________________________________________
    # method for update simulation
    def update(self):
        """
        Updates the simulation.
        """

        
        for i in range(len(self.player)):
            # update collision
            self.player[i].set_bumper_state(self.check_collision(i))
            # Updating the player's movement
            self.player[i].update()
        
        # update ball's collision
        self.ball.set_bumper_state(self.check_collision_ball())
        # Updating the ball's movement
        self.ball.update()
        
        self.check_goal()
        

    def draw(self, window, environment):
        """
        Draws the roomba and its movement history.

        :param window: pygame's window where the drawing will occur.
        """
        list_centers = [np.array([round(M2PIX * self.ball.pose.position.x), round(M2PIX * self.ball.pose.position.y)])]
        list_radius = [RADIUS_BALL]
        list_rotation = [self.ball.pose.rotation]
        for i in range(len(self.player)):
            list_centers.append(np.array([round(M2PIX * self.player[i].pose.position.x), round(M2PIX * self.player[i].pose.position.y)]))
            list_radius.append(self.player[i].radius)
            list_rotation.append(self.player[i].pose.rotation)
        params = {
            "window": window,
            "list_centers": list_centers,
            "list_radius": list_radius,
            "list_rotation": list_rotation

        }


        ex1 = round(M2PIX * (self.player[NUM_1].pose.position.x +self.player[NUM_1].radius * cos(self.player[NUM_1].pose.rotation)))
        ey1 = round(M2PIX * (self.player[NUM_1].pose.position.y + self.player[NUM_1].radius * sin(self.player[NUM_1].pose.rotation)))
        r1 = round(M2PIX * self.player[NUM_1].radius)
        # Drawing roomba's inner circle
        pygame.draw.circle(window, (255,0,0), (sx1, sy1), r1, 0)
        # Drawing roomba's outer circle
        pygame.draw.circle(window, (50, 50, 50), (sx1, sy1), r1, 4)
        # Drawing roomba's orientation
        pygame.draw.line(window, (50, 50, 50), (sx1, sy1), (ex1, ey1), 3)

        # roomba 2
        sx2 = round(M2PIX * self.player[NUM_2].pose.position.x)
        sy2 = round(M2PIX * self.player[NUM_2].pose.position.y)
        ex2 = round(M2PIX * (self.player[NUM_2].pose.position.x +self.player[NUM_2].radius * cos(self.player[NUM_2].pose.rotation)))
        ey2 = round(M2PIX * (self.player[NUM_2].pose.position.y + self.player[NUM_2].radius * sin(self.player[NUM_2].pose.rotation)))
        r2 = round(M2PIX * self.player[NUM_2].radius)
        # Drawing roomba's inner circle
        pygame.draw.circle(window, (238, 203, 0), (sx2, sy2), r2, 0) 
        # Drawing roomba's outer circle
        pygame.draw.circle(window, (50, 50, 50), (sx2, sy2), r2, 4)
        # Drawing roomba's orientation
        pygame.draw.line(window, (50, 50, 50), (sx2, sy2), (ex2, ey2), 3)

        # ball
        sxB = round(M2PIX * self.ball.pose.position.x)
        syB = round(M2PIX * self.ball.pose.position.y)
        exB = round(M2PIX * (self.ball.pose.position.x +self.ball.radius * cos(self.ball.pose.rotation)))
        eyB = round(M2PIX * (self.ball.pose.position.y + self.ball.radius * sin(self.ball.pose.rotation)))
        rB = round(M2PIX * self.ball.radius)
        # Drawing roomba's inner circle
        pygame.draw.circle(window, (255,255,255), (sxB, syB), rB, 0)
        # Drawing roomba's outer circle
        pygame.draw.circle(window, (255, 255, 255), (sxB, syB), rB, 4)
        # Drawing soccer goal
        pygame.draw.rect(window, (0, 0, 0), Rect(0, round(SCREEN_HEIGHT)/2-100, 30, 200))
        pygame.draw.rect(window, (0, 0, 0), Rect(round(SCREEN_WIDTH)-30, round(SCREEN_HEIGHT)/2-100, 30, 200))
        # scoreboard
        pygame.draw.rect(window, (0, 0, 0), Rect(28, round(SCREEN_HEIGHT-30), 250, 30))
        # draw vision
        self.draw_vision(window, self.player[NUM_1].pose.position, self.player[NUM_2].pose.position)

    def draw_vision(self, window, position1, position2):
        # player1
        pie(window, round(M2PIX * position1.x), round(M2PIX * position1.y), round(M2PIX * 2.5 * self.player[NUM_1].radius), (int(RADIAN_TO_DEGREE * self.player[NUM_1].pose.rotation)-45)%360, (int(RADIAN_TO_DEGREE * self.player[NUM_1].pose.rotation)+45)%360 , (255,255,255))
        # player2
        pie(window, round(M2PIX * position2.x), round(M2PIX * position2.y), round(M2PIX * 2.5 * self.player[NUM_2].radius), (int(RADIAN_TO_DEGREE * self.player[NUM_2].pose.rotation)-45)%360, (int(RADIAN_TO_DEGREE * self.player[NUM_2].pose.rotation)+45)%360, (255,255,255))
        

def draw(simulation, environment):
    """
    Redraws the pygame's window.

    :param simulation: the simulation object.
    :param window: pygame's window where the drawing will occur.
    """


    # scoreboard
    scoreboard = "Left " + str(simulation.left_goal) + " x " + str(simulation.right_goal) + " Right"
    textsurface = font.render(scoreboard, False, (255, 255, 255))
    

    window.fill((35,142,35))
    window.blit(logo, (round(SCREEN_WIDTH)/2+100,40))
    
    simulation.draw(window, environment)
    window.blit(textsurface, (40,round(SCREEN_HEIGHT-20)))
    pygame.display.update()



