# ______________________________________________________________________________
# importation

from robot_soccer_python.constants import *
from robot_soccer_python.utils import Pose
from robot_soccer_python.agents import Ball
from robot_soccer_python.simulation import *
from robot_soccer_python.state_machine_ball import FiniteStateMachineBall, MoveForwardStateBall
import datetime
import os

# ______________________________________________________________________________
# simulation2D function
def simulation2D(players, shockable = True, full_vision = False):
    """
    This function initialize the simulation and return a object that the user 
    can pass the controls and get the sensors information.

    :param players: a list of Players for simulation
    :type: list of Player
    :param shockable: parameter that informs if players will collide with themselves
    :type shockable: bool
    :param full_vision: parameter that informs if player will see every thing even if it’s not in the vision cone.
    :type full_vision: bool
    """
    behavierBall = FiniteStateMachineBall(MoveForwardStateBall(False))
    poseBall = Pose(PIX2M * SCREEN_WIDTH*1/4.0, PIX2M * SCREEN_HEIGHT / 2.0, 0)
    ball = Ball(poseBall, 1.0, 100, RADIUS_BALL, behavierBall)
    for player in players:
        player.sensors.set_full_vision(full_vision)
        
    return Simulation(np.array(players), ball, shockable, full_vision)

   

def init_simulation(simulation):
    now = datetime.datetime.now()
    pygame.init()
    window = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Robot soccer 2D environment")
    # icon = pygame.image.load(os.getcwd() + '/icon.PNG')
    # pygame.display.set_icon(icon)
    clock = pygame.time.Clock()

    environment = Environment(window)
    while (datetime.datetime.now() - now).seconds < 1:
        clock.tick(FREQUENCY)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                end_simulation()

        simulation.update()
        draw(simulation, window, environment)



def end_simulation():
    pygame.quit()