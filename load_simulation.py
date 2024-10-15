import numpy as np
from pydrake.all import (
    AddDefaultVisualization,
    DiscreteContactApproximation,
    PidController,
    RobotDiagramBuilder,
    Simulator,
    StartMeshcat,
)

from underactuated import ConfigureParser
from underactuated.multibody import MakePidStateProjectionMatrix

meshcat = StartMeshcat()

robot_builder = RobotDiagramBuilder(time_step=1e-4)

parser = robot_builder.parser()
ConfigureParser(parser)
parser.AddModelsFromUrl("file://drake_models/home/kyle/nimbus.sunny/lower_body.urdf")
parser.AddModelsFromUrl("package://underactuated/models/littledog/ground.urdf")
plant = robot_builder.plant()
plant.set_discrete_contact_approximation(DiscreteContactApproximation.kLagged)
plant.Finalize()

builder = robot_builder.builder()

# PD Controller from Random Example
num_u = plant.num_actuators()
kp = 1500 * np.ones(num_u)
ki = 0.0 * np.ones(num_u)
kd = 100.0 * np.ones(num_u)

S = MakePidStateProjectionMatrix(plant)

control = builder.AddSystem(
    PidController(
        kp=kp,
        ki=ki,
        kd=kd,
        state_projection=S,
        output_projection=plant.MakeActuationMatrix()[16:, :].T,
        # For When Boardwalk Arms Added
        # output_projection=plant.MakeActuationMatrix()[32:, :].T,
    )
)

builder.Connect(
    plant.get_state_output_port(), control.get_input_port_estimated_state()
)
builder.Connect(control.get_output_port(), plant.get_actuation_input_port())

AddDefaultVisualization(builder, meshcat=meshcat)

diagram = robot_builder.Build()
simulator = Simulator(diagram)
context = simulator.get_mutable_context() 
plant_context = plant.GetMyContextFromRoot(context)

q0 = np.zeros(plant.num_positions())
q0[0] = 1
q0[6] = 1
q0[7:12] = [-0.25, 0, 0, -0.65, 0.4]  # Left leg
q0[12:] = [0.25, 0, 0, -0.65, 0.4]  # Right leg

# For When Boardwalk Arms Added
# q0[16+7:16+12] = [0, 0, 0, -0.5, 1.0]  # Left leg
# q0[16+12:16+17] = [0, 0, 0, -0.5, 1.0]  # Right leg

plant.SetPositions(plant_context, q0)

x0 = S @ plant.get_state_output_port().Eval(plant_context)
control.get_input_port_desired_state().FixValue(
    control.GetMyContextFromRoot(context), x0
)

meshcat.StartRecording()
simulator.AdvanceTo(3.0)
meshcat.PublishRecording()

while True:
    pass
