import pdb
import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.geometry import StartMeshcat
from pydrake.geometry import Meshcat
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization
from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
from pydrake.trajectories import PiecewisePolynomial
from pydrake.common.eigen_geometry import Quaternion
from pydrake.all import Integrator, ConstantVectorSource
from pydrake.multibody.tree import PdControllerGains

from ground_reaction_model import add_ground
from swing_foot_trajectory_generator_system import SwingFootTrajectoryGeneratorSystem
from FSMStateMachine import FsmStateMachine
# from drake_ik import IK
from differential_ik_system import DifferentialIKSystem
from position_setter import PositionSetter
from util import DrawAndSaveDiagram

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def print_model_info(plant):
    print(f"Number of positions: {plant.num_positions()}")
    print(f"Number of velocities: {plant.num_velocities()}")
    print(f"Number of joints: {plant.num_joints()}")
    print(f"Number of bodies: {plant.num_bodies()}")
    print(f"Number of actuators: {plant.num_actuators()}")
    print(f"Number of frames: {plant.num_frames()}")

def create_initial_pose_trajectory(initial_height, duration):
    times = [0., duration]
    poses = np.array([
        [0., 0., initial_height, 1., 0., 0., 0.],  # Start at initial height
        [0., 0., initial_height, 1., 0., 0., 0.]  # End at initial height
    ]).T
    return PiecewisePolynomial.FirstOrderHold(times, poses)

def main():

    meshcat = StartMeshcat()

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    
    robot_url = "file://drake_models/home/sharanya/workspace/nimbus.sunny/lower_body.urdf"
    parser = Parser(plant)
    parser.AddModelsFromUrl(robot_url)

    # Add actuators to each joint    
    joint_names = [
        "R_ankle", "R_knee", "R_hip_yaw", "R_hip_roll", "R_hip_pitch",
        "L_ankle", "L_knee", "L_hip_yaw", "L_hip_roll", "L_hip_pitch"
    ]
    
    for joint_name in joint_names:
        try:
            joint = plant.GetJointByName(joint_name)
            actuator = plant.AddJointActuator(name=f"{joint_name}_actuator", joint=joint)
            # Pd = PdControllerGains()
            # Pd.p = 100.0
            # Pd.d = 10.0
            # actuator.set_controller_gains(Pd)
            print(f"Added actuator: {actuator.name()} for joint: {joint.name()}")
        except RuntimeError as e:
            print(f"Error adding actuator for joint {joint_name}: {e}")

    # pdb.set_trace()

    add_ground(plant, soft_contact=True)  # Implement soft contact in add_ground function
    plant.Finalize()
    plant_context = plant.CreateDefaultContext()

    print_model_info(plant)

    AddDefaultVisualization(builder=builder, meshcat=meshcat)

    fsm = builder.AddSystem(FsmStateMachine(plant, plant_context))
    swing_foot_trajectory_generator = \
        builder.AddSystem(SwingFootTrajectoryGeneratorSystem(plant, plant_context))
    controller = builder.AddSystem(DifferentialIKSystem(plant, plant_context))
    swing_foot_angular_velocity_source = \
        builder.AddSystem(ConstantVectorSource(np.array([0, 0, 0])))
    
    # plant_mutable_context = plant.GetMyMutableContextFromRoot()
    position_setter = builder.AddSystem(PositionSetter(plant, plant_context))

    # set up the diagram
    # send robot state to fsm
    # fix the initial state of the fsm to be right foot first with phase 0.
    # builder.Connect(plant.get_state_output_port(), fsm.get_input_port(1))
    builder.Connect(fsm.get_output_port(0), \
                    swing_foot_trajectory_generator.get_input_port(0))
    builder.Connect(swing_foot_angular_velocity_source.get_output_port(), \
                    swing_foot_trajectory_generator.get_input_port(1))
    # builder.Connect(plant.get_state_output_port(), \
    #                 swing_foot_trajectory_generator.get_input_port(2))

    builder.Connect(fsm.get_output_port(0), controller.get_input_port(0))
    builder.Connect(swing_foot_trajectory_generator.get_output_port(0), \
                    controller.get_input_port(1))
    builder.Connect(plant.get_state_output_port(), controller.get_input_port(2))
    
    integrator = builder.AddSystem(Integrator(17))
    # set update the sim plant positions to the controller output
    builder.Connect(plant.get_state_output_port(), position_setter.get_input_port(0))
    builder.Connect(controller.get_output_port(0), \
                    integrator.get_input_port(0))
    builder.Connect(integrator.get_output_port(0), \
                    position_setter.get_input_port(1))
    
    builder.Connect(position_setter.get_output_port(0), plant.get_actuation_input_port())
    
    diagram = builder.Build()
    diagram.set_name("Foundation_bot_system_diagram")
    DrawAndSaveDiagram(diagram, "Foundation_bot_system_diagram")

    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1)
    context = simulator.get_mutable_context()

    fsm_context = fsm.GetMyMutableContextFromRoot(context)
    fsm.get_input_port(0).FixValue(fsm_context, np.array([0, 0]))

    plant_context = plant.GetMyContextFromRoot(context)

    initial_height = 0.9
    initial_pose_trajectory = create_initial_pose_trajectory( \
                                initial_height, duration = 2.0)
    # Set initial joint positions
    q0 = plant.GetPositions(plant_context)
    print(plant.GetPositionNames())
    # Adjust these values based on your robot's joint configuration
    q0[7:12] = [-0.25, 0, 0, -0.65, 0.4]  # Right leg
    q0[12:] = [0.25, 0, 0, -0.65, 0.4]  # Left leg
    plant.SetPositions(plant_context, q0)

    # Gradual initial pose setting
    pelvis = plant.GetBodyByName("pelvis")
    t = 0
    dt = 0.001
    pose = initial_pose_trajectory.value(t)
    position = pose[:3].flatten()  # Flatten to ensure it's a 1D array
    quat_wxyz = pose[3:].flatten()  # Flatten and reorder for Quaternion constructor
    quaternion = Quaternion(quat_wxyz[0], quat_wxyz[1], quat_wxyz[2], \
                            quat_wxyz[3])
    # print("quaternion ", quaternion)
    R = RotationMatrix(quaternion)
    X_WP = RigidTransform(R, position)
    # print("this is X_WP ", X_WP)
    plant.SetFreeBodyPose(plant_context, pelvis, X_WP)

    integrator.set_integral_value(
        integrator.GetMyMutableContextFromRoot(context),
        plant.GetPositions(plant_context))

    meshcat.StartRecording()
    simulator.AdvanceTo(100)
    print("Simulation complete. Meshcat URL:")
    print(meshcat.web_url())
    meshcat.PublishRecording()
    # simulator.Initialize()

if __name__ == "__main__":
    main()