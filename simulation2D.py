# ______________________________________________________________________________
# importation
from constants import *
from utils import Pose
from agents import Ball
from simulation import *
from state_machine_ball import FiniteStateMachineBall, MoveForwardStateBall

# ______________________________________________________________________________
# simulation2D function
def simulation2D(players, shockable = True):
    """
    This function initialize the simulation and return a object that the user 
    can pass the controls and get the sensors information.

    :param players: a list of Players for simulation
    :type: list of Player
    :param shockable: parameter that informs if players will collide with themselves
    :type shockable: bool
    """

    pygame.init()
    window = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Robot soccer 2D simulation")
    clock = pygame.time.Clock()

    
    behavierBall = FiniteStateMachineBall(MoveForwardStateBall(False))
    poseBall = Pose(PIX2M * SCREEN_WIDTH * 3 / 4.0, PIX2M * SCREEN_HEIGHT / 2.0, 0)
    ball = Ball(poseBall, 1.0, 100, RADIUS_BALL, behavierBall)

    simulation = Simulation(np.array(players), ball, shockable)


    run = True
    environment = Environment(window)
    while run:
        clock.tick(FREQUENCY)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

        simulation.update()
        draw(simulation, window, environment)


    pygame.quit()
