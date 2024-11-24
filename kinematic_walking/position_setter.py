
from pydrake.all import LeafSystem, BasicVector
from pydrake.systems.framework import EventStatus
import numpy as np

class PositionSetter(LeafSystem):
    def __init__(self, plant, plant_context):
        super().__init__()
        self.plant = plant
        self.plant_context = plant_context
        self.DeclareVectorInputPort("current_state", plant.num_positions() + plant.num_velocities())
        print("plant.num_positions() ", plant.num_positions())
        print("plant.num_velocities() ", plant.num_velocities())
        # These correspond to qdot not v!
        self.DeclareVectorInputPort("desired_positions", plant.num_positions())
        # self.DeclareVectorOutputPort("gravity_compensation", plant.num_actuators(), self.calc_gravity_compensation)
        self.DeclareVectorOutputPort("desired_actuation", plant.num_actuators(), self.calc_desired_actuation)
        # self.DeclarePerStepDiscreteUpdateEvent(self.DoCalcDiscreteVariableUpdates)  # Adjust period if needed
        self.dt = 0.001
        self.Kp = 10000.0
        self.Kd = 10.0

    # def DoCalcDiscreteVariableUpdates(self, context, discrete_state):
    #     # Get the desired positions from the input port
    #     desired_positions = self.get_input_port(0).Eval(context)
        
    #     # Access the plant context and set positions
    #     # current_positions = self.plant.GetPositions(plant_context)
    #     # desired_positions = self.calc_desired_positions(desired_velocities, \
    #     #                                                 current_positions)
    #     self.plant.SetPositions(self.plant_context, desired_positions)

    #     return EventStatus.Succeeded()
    
    def calc_desired_actuation(self, context, desired_actuation):
        current_positions = self.get_input_port(0).Eval(context)[:self.plant.num_positions()]
        desired_positions = self.get_input_port(1).Eval(context)

        # print("current_positions ", current_positions)
        self.plant.SetPositions(self.plant_context, current_positions)

        u = self.Kp * (desired_positions - current_positions)
    #     # without pelvis since the pelvis is not actuated
        vec_desired_actuatiion_without_pelvis = np.zeros(self.plant.num_positions() - 7)
        vec_desired_actuatiion_without_pelvis = u[7:]



        # vec_desired_actuatiion_without_pelvis += self.plant.CalcGravityGeneralizedForces(self.plant_context)
        desired_actuation.SetFromVector(vec_desired_actuatiion_without_pelvis)

    # def calc_gravity_compensation(self, context, gravity_compensation):
    #     # current plant state
    #     current_positions = self.get_input_port(0).Eval(context)
    #     desired_positions = self.get_input_port(1).Eval(context)

    #     # current_positions = self.get_input_port(0).Eval(context)
    #     # Set the plant state to the current state
    #     self.plant.SetPositions(self.plant_context, current_positions)

    #     # Calculate the gravity compensation
    #     gravity_compensation.SetFromVector(self.plant.CalcGravityGeneralizedForces(self.plant_context))
