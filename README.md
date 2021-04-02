# simulation_soccer_2d

<img src="https://img.shields.io/static/v1?label=Version&message=1.0.1&color=7159c1?style=for-the-badge"/> 

simulation_soccer_2d is a template for simulation to study AI. Basicaly, think in a physical 
robot: you need to implement a mind of this roboty only. In this simulation, you can get 
the sensors's information and move the robot. The rest of simulation already is implemented.

For use this simulation, you can clone this repository in your project, this is specified 
in the installation instructions.

![](https://user-images.githubusercontent.com/50979367/80439962-0ff43000-88de-11ea-84db-4bef850e44b3.png)

# Installation Guide

Open a terminal into your project and download this repository:

`git clone https://github.com/jonysalgado/simulation_soccer_2d.git`

then you need to install the basic dependencies to run the project on your system:

```
cd simulation_soccer_2d
pip install -r requirements.txt
```

# Testing file

For testing and learning about this simulation you go back to your project:

```bash
cd ..
```
to import the repository:

```python
import sys
sys.path.append('/simulation-soccer-2d')
```

for testing you can copy the code bellow:

```python
from simulation_soccer_2d.simulation2D import simulation2D, init_simulation
from simulation_soccer_2d.agents import Player, Pose, Sensors

simulation = simulation2D([
    Player(Pose(3, 3, 0), 2, 2, 0.15), 
    Player(Pose(4, 4, 3.14), 20, 20, 0.15),
    Player(Pose(5, 3, 0), 2, 2, 0.15),
    Player(Pose(7, 3, 3.14), 2, 2, 0.15)],
    shockable=False,
    full_vision=False)


while True:
    simulation.set_commands([(0, 0), (0, 0), (0,0), (0,0)])
    init_simulation(simulation)
    player_sensors = simulation.get_sensors()
```

