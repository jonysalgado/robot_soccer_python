from simulation2D import simulation2D
from agents import Player, Pose

simulation2D([Player(Pose(300 * 0.01, 300 * 0.01, 0), 2, 2, 0.15), Player(Pose(400 * 0.01, 400 * 0.01, 3.14), 2, 2, 0.2),
Player(Pose(500 * 0.01, 300 * 0.01, 0), 2, 2, 0.3)], True)

