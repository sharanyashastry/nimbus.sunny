
from pydrake.all import LeafSystem, BasicVector
from pydrake.systems.framework import EventStatus
import numpy as np

class PositionSetter(LeafSystem):
    def __init__(self, plant, plant_context):
        super().__init__()
        self.plant = plant
        self.plant_context = plant_context
        # These correspond to qdot not v!
        self.DeclareVectorInputPort("desired_velocities", plant.num_positions())
        self.DeclarePerStepDiscreteUpdateEvent(self.DoCalcDiscreteVariableUpdates)  # Adjust period if needed
        self.dt = 0.001

    def DoCalcDiscreteVariableUpdates(self, context, discrete_state):
        # Get the desired positions from the input port
        desired_positions = self.get_input_port(0).Eval(context)
        
        # Access the plant context and set positions
        # current_positions = self.plant.GetPositions(plant_context)
        # desired_positions = self.calc_desired_positions(desired_velocities, \
        #                                                 current_positions)
        self.plant.SetPositions(self.plant_context, desired_positions)

        return EventStatus.Succeeded()
    
    # def calc_desired_positions(self, desired_velocities, current_positions):
    #     # Calculate desired positions from desired velocities
    #     pelvis_current_orientation = current_positions[:4]
    #     # convert to quaternion
    #     # TODO: Figure out what goes in here
    #     pelvis_angular_velocity = desired_velocities[:3]
