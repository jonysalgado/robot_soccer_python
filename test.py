from simulation2D import simulation2D, init_simulation
from agents import Player, Pose, Sensors
import pygame

simulation = simulation2D([Player(Pose(300 * 0.01, 300 * 0.01, 3.92), 200, 200, 0.15), Player(Pose(400 * 0.01, 400 * 0.01, 3.14), 20, 20, 0.2),
Player(Pose(500 * 0.01, 300 * 0.01, 0), 2, 2, 0.3)], True, False)

i = 0
while True:
    simulation.set_commands([(2, 2), (2, 2), (0,0)])
    init_simulation(simulation)
    
