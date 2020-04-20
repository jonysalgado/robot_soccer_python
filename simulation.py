import pygame
from pygame.rect import Rect
from pygame.gfxdraw import pie
from math import sin, cos, sqrt
from constants import *
from utils import *
import datetime

# ______________________________________________________________________________
# class Simulation

class Simulation:
    """
    Represents the simulation.
    """
    def __init__(self, player, ball):
        """
        Creates the simulation.

        :param player: the robots used in this simulation.
        :type player: numpy.ndarray
        :param ball: the ball used in this simulation.
        :type ball: Ball
        """
        self.player = player
        self.ball = ball
        self.left_goal = 0
        self.right_goal = 0
        self.goal = datetime.datetime.utcnow()
        
    # __________________________________________________________________________
    # methods for check collision

    def check_collision(self, num):
        """
        Checks collision between the robot and the walls.

        :param 
        :return: the bumper state (if a collision has been detected).
        :rtype: bool
        """
        # Converting screen limits from pixels to meters
        width = SCREEN_WIDTH * PIX2M
        height = SCREEN_HEIGHT * PIX2M
        bumper_state = False
        # Computing the limits of the player's bounding box
        left = self.player[num].pose.position.x - self.player[num].radius
        right = self.player[num].pose.position.x + self.player[num].radius
        top = self.player[num].pose.position.y - self.player[num].radius
        bottom = self.player[num].pose.position.y + self.player[num].radius
        # Testing if the bounding box has hit a wall
        if left <= 0.0:
            self.player[num].pose.position.x = self.player[num].radius
            bumper_state = True
        if right >= width:
            self.player[num].pose.position.x = width - self.player[num].radius
            bumper_state = True
        if top <= 0.0:
            self.player[num].pose.position.y = self.player[num].radius
            bumper_state = True
        if bottom >= height:
            self.player[num].pose.position.y = height - self.player[num].radius
            bumper_state = True

    def get_collision_params(self, num):
        

    def check_collision_with_players(self, num_1, num_2):
        # check collision with other player
        dist_players = sqrt((self.player[num_1].pose.position.x - self.player[num_2].pose.position.x)**2+(self.player[num_1].pose.position.y - self.player[num_2].pose.position.y)**2)
        if dist_players <=(self.player[num_1].radius + self.player[num_2].radius):
            bumper_state = True
        return bumper_state
    


    
    def check_collisionBall(self):
        """
        Checks collision between the ball with the walls and other players.

        :return: the bumper state (if a collision has been detected).
        :rtype: bool
        """
        # Converting screen limits from pixels to meters
        width = SCREEN_WIDTH * PIX2M
        height = SCREEN_HEIGHT * PIX2M
        bumper_state = False
        # Computing the limits of the roomba's bounding box
        left = self.ball.pose.position.x - self.ball.radius
        right = self.ball.pose.position.x + self.ball.radius
        top = self.ball.pose.position.y - self.ball.radius
        bottom = self.ball.pose.position.y + self.ball.radius
        # Testing if the bounding box has hit a wall
        if left <= 0.0: 
            self.ball.pose.position.x = self.ball.radius
            bumper_state = True
        if right >= width:
            self.ball.pose.position.x = width - self.ball.radius
            bumper_state = True
        if top <= 0.0:
            self.ball.pose.position.y = self.ball.radius
            bumper_state = True
        if bottom >= height:
            self.ball.pose.position.y = height - self.ball.radius
            bumper_state = True

        # check collision with other player
        dist_player1 = sqrt((self.ball.pose.position.x - self.player[NUM_1].pose.position.x)**2+(self.ball.pose.position.y - self.player[NUM_1].pose.position.y)**2)
        if dist_player1 <=(self.ball.radius + RADIUS_PLAYER):
            bumper_state = True
        dist_player2 = sqrt((self.ball.pose.position.x - self.player[NUM_2].pose.position.x)**2+(self.ball.pose.position.y - self.player[NUM_2].pose.position.y)**2)
        if dist_player2 <=(self.ball.radius + RADIUS_PLAYER):
            bumper_state = True

        velocityBall = TransformCartesian(self.ball.linear_speed, self.ball.pose.rotation)
        velocityBall = Vector2(velocityBall.x, velocityBall.y)
        dirvector1 = Vector2(self.ball.pose.position.x - self.player[NUM_1].pose.position.x, self.ball.pose.position.y - self.player[NUM_1].pose.position.y)
        dirvector1.normalize()
        u1 = velocityBall.dot(dirvector1)
        if u1 > 0 and dist_player1 <=(self.ball.radius + RADIUS_PLAYER):
            return False

        dirvector2 = Vector2(self.ball.pose.position.x - self.player[NUM_2].pose.position.x, self.ball.pose.position.y - self.player[NUM_2].pose.position.y)
        dirvector2.normalize()
        u2 = velocityBall.dot(dirvector2)
        if u2 > 0 and dist_player2 <=(self.ball.radius + RADIUS_PLAYER):
            return False
        return bumper_state
    
    def check_goal(self):
        """
        Check if there was a goal.
        """
        
        ball_x = self.ball.pose.position.x
        ball_y = self.ball.pose.position.y
        # left goal
        if round(M2PIX * ball_x - RADIUS_BALL) >= (round(SCREEN_WIDTH)-30) and round(M2PIX * ball_y - RADIUS_BALL) >= (round(SCREEN_HEIGHT)/2-100) and round(M2PIX * ball_y +  RADIUS_BALL) <= (round(SCREEN_HEIGHT)/2+100):
            if (datetime.datetime.now() - self.goal).seconds > 3:
                self.left_goal += 1
                self.goal = datetime.datetime.now()
                
        
        # Right goal
        if round(M2PIX * ball_x + RADIUS_BALL) <= 30 and round(M2PIX * ball_y - RADIUS_BALL) >= (round(SCREEN_HEIGHT)/2-100) and round(M2PIX * ball_y +  RADIUS_BALL) <= (round(SCREEN_HEIGHT)/2+100):
            if (datetime.datetime.now() - self.goal).seconds > 3:
                self.right_goal += 1
                self.goal = datetime.datetime.now()
        
        if (datetime.datetime.now() - self.goal).seconds < 1.0e-3:
            self.restard_game()

    def restard_game(self):
        # players
        self.player[NUM_1].pose = Pose(PIX2M * SCREEN_WIDTH/3, PIX2M * SCREEN_HEIGHT/2, 0)
        self.player[NUM_1].linear_speed = 0.0
        self.player[NUM_2].pose = Pose(30, 30, pi) # Pose(PIX2M * 2 * SCREEN_WIDTH/3, PIX2M * SCREEN_HEIGHT/2, pi) 
        self.player[NUM_2].linear_speed = 0.0

        # ball
        self.ball.pose = Pose(PIX2M * SCREEN_WIDTH/2, PIX2M * SCREEN_HEIGHT/2, 0)
        self.ball.linear_speed = 0.0

    def update(self):
        """
        Updates the simulation.
        """
        # Adding roomba's current position to the movement history
        # self.point_list.append((round(M2PIX * self.player[NUM_1].pose.position.x), round(M2PIX * self.player[NUM_1].pose.position.y)))
        # if len(self.point_list) > 2000:
        #     self.point_list.pop(0)
        # Verifying collision
        bumper_state1 = self.check_collision(0,1)
        bumper_state2 = self.check_collision(1,0)
        bumper_stateBall = self.check_collisionBall()
        self.player[NUM_1].set_bumper_state(bumper_state1)
        self.player[NUM_2].set_bumper_state(bumper_state2)
        self.ball.set_bumper_state(bumper_stateBall)
        self.ball.posPlayer = [self.player[NUM_1].pose,self.player[NUM_2].pose]
        self.ball.speedPlayer = [self.player[NUM_1].linear_speed,self.player[NUM_2].linear_speed]
        # Updating the velocity and position in player class
        self.player[NUM_1].otherPlayerVelocity = self.player[NUM_2].linear_speed
        self.player[NUM_1].otherPlayerPose = self.player[NUM_2].pose
        self.player[NUM_2].otherPlayerVelocity = self.player[NUM_1].linear_speed
        self.player[NUM_2].otherPlayerPose = self.player[NUM_1].pose
        self.player[NUM_1].BallVelocity = self.ball.linear_speed
        self.player[NUM_2].BallVelocity = self.ball.linear_speed
        self.player[NUM_1].BallPose = self.ball.pose
        self.player[NUM_2].BallPose = self.ball.pose

        # Updating the robot's behavior and movement
        self.player[NUM_1].update()
        self.player[NUM_2].update()
        self.ball.update()
        self.check_goal()
        

    def draw(self, window):
        """
        Draws the roomba and its movement history.

        :param window: pygame's window where the drawing will occur.
        """
        NUM_1 = 0
        NUM_2 = 1
        # Drawing soccer field
        pygame.draw.circle(window, (255,255,255), (round(SCREEN_WIDTH/2), round(SCREEN_HEIGHT/2)), 70, 3)
        pygame.draw.line(window, (255,255,255), (round(SCREEN_WIDTH/2), 30), (round(SCREEN_WIDTH/2), SCREEN_HEIGHT - 30), 3)
        pygame.draw.line(window, (255,255,255), (30, 30), (round(SCREEN_WIDTH)-30, 30), 3)
        pygame.draw.line(window, (255,255,255), (30, 30), (30, round(SCREEN_HEIGHT)-30), 3)
        pygame.draw.line(window, (255,255,255), (round(SCREEN_WIDTH)-30, 30), (round(SCREEN_WIDTH)-30, round(SCREEN_HEIGHT)-30), 3)
        pygame.draw.line(window, (255,255,255), (30, round(SCREEN_HEIGHT)-30), (round(SCREEN_WIDTH)-30, round(SCREEN_HEIGHT)-30), 3)
        # If we have less than 2 points, we are unable to plot the movement historypygame.draw.line(window, (255,255,255), (30, round(SCREEN_HEIGHT)-30), (round(SCREEN_WIDTH)-30, round(SCREEN_HEIGHT)-30), 3)
        # if len(self.point_list) >= 2:
        #     pygame.draw.lines(window, (255, 0, 0), False, self.point_list, 4)
        # Computing roomba's relevant points and radius in pixels
        sx1 = round(M2PIX * self.player[NUM_1].pose.position.x)
        sy1 = round(M2PIX * self.player[NUM_1].pose.position.y)
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
        

def draw(simulation, window, logo, font):
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
    simulation.draw(window)
    window.blit(textsurface, (40,round(SCREEN_HEIGHT-20)))
    pygame.display.update()



