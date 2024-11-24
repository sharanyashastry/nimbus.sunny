import numpy as np
from pydrake.trajectories import BezierCurve
from pydrake.all import LeafSystem
from pydrake.systems.framework import EventStatus
import pdb

class FsmStateMachine(LeafSystem):
    def __init__(self, plant, plant_context):
        print("Initializing FSM State Machine")
        LeafSystem.__init__(self)
        self._plant = plant
        self._plant_context = plant_context
        self._world_frame = plant.world_frame()
        self.step_duration = 1
        self.epsilon = 1e-6
        self.sim_dt = 0.001

        # Needs a constant vector sink for rotation velocity. 
        self.DeclareVectorInputPort("fsm_state_input : isLeftFoot, \
                                    curr_phase_input", 2)
        # self.DeclareVectorInputPort("nimbus.position_measured", 17)
        # Declare output ports
        self.DeclareVectorOutputPort("fsm_state_input : isLeftFoot, phase_ouput", \
                                     2, self.fsm_state)
        
        # Declare discrete state variables: [isLeftFoot, curr_phase]
        self.DeclareDiscreteState([0.0, 0.0])

        # This time step is the same as sim dt.
        self.DeclarePerStepDiscreteUpdateEvent(self.DoCalcDiscreteVariableUpdates)

    def DoCalcDiscreteVariableUpdates(self, context, discrete_state):
        vec = context.get_discrete_state_vector()
        _fsm_state = vec[0]
        _phase = vec[1]

        if _phase >= self.step_duration - self.epsilon:
            _fsm_state = 1 - _fsm_state
            _phase = 0.0
            print("Switching feet")
        else:
            _phase += self.sim_dt
        
        discrete_state.get_mutable_vector().SetFromVector([_fsm_state, _phase])
        return EventStatus.Succeeded()


    def fsm_state(self, context, output):
        # T is the step_duration parameter and phase represents what point along the
        # step you are in time.
        output_vec =  context.get_discrete_state_vector().get_value()
        output.SetFromVector(output_vec)

