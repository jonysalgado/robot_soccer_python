# ______________________________________________________________________________
# importation
import random
import math
from simulation_soccer_2d.constants import *
from simulation_soccer_2d.utils import *

# ______________________________________________________________________________
# class FiniteStateMachineBall
class FiniteStateMachineBall:
    """
    A finite state machine.
    """
    def __init__(self, state):
        self.state = state

    def change_state(self, new_state):
        self.state = new_state

    def update(self, agent):
        
        self.state.check_transition(agent, self)
        self.state.execute(agent)
    
 

# ______________________________________________________________________________
# class State
class State:
    """
    Abstract state class.
    """
    def __init__(self, state_name):
        """
        Creates a state.

        :param state_name: the name of the state.
        :type state_name: str
        """
        self.state_name = state_name

    def check_transition(self, agent, fsm):
        """
        Checks conditions and execute a state transition if needed.

        :param agent: the agent where this state is being executed on.
        :param fsm: finite state machine associated to this state.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")

    def execute(self, agent):
        """
        Executes the state logic.

        :param agent: the agent where this state is being executed on.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")


class MoveForwardStateBall(State):

    def __init__(self, initial):
        super().__init__("MoveForward")
        self.initial = initial

    def check_transition(self, agent, state_machine):  
        if agent.get_bumper_state():
            state_machine.change_state(Reflection())

    def execute(self, agent):
        agent.set_cont_friction(self.initial, 1)
        self.initial = False
        
        velocity = math.fabs(agent.linear_speed)-math.fabs(FACTOR_FRICTION*(agent.cont_friction)**(1/3))
        if velocity < 0:
            velocity = 0
        agent.set_velocity(velocity,0)
        agent.move()

class Reflection(State):
    def __init__(self):
        super().__init__("Reflection")

    def check_transition(self, agent, state_machine):  
        self.rotation(agent)
        state_machine.change_state(MoveForwardStateBall(agent.get_bumper_state()))

    def rotation(self, agent):
        if agent.get_bumper_state():
            self.calculate_speed(agent)

    def calculate_speed(self, agent):
        collide = agent.get_collide
        coordinate = TransformCartesian(agent.linear_speed, agent.pose.rotation)
        if collide in ["left", "right"]:
            if math.fabs(agent.pose.rotation) < 1.0e-3:
                agent.pose.rotation = math.pi
            else:
                agent.pose.rotation = TransformPolar(-1 * coordinate.x, coordinate.y).rotation
        
        elif collide in ["top", "bottom"]:
            agent.pose.rotation = TransformPolar(coordinate.x, -1*coordinate.y).rotation

        elif collide != None:
            agent.linear_speed, agent.pose.rotation = agent.collision_player_speed
        
    def execute(self, agent):
        agent.move()

    

