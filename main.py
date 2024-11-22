import pdb
import numpy as np
import os
from util import DrawAndSaveDiagram
from util import MeshcatUtils

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

from gait_planner import GaitPlanner
from inverse_kinematics import InverseKinematicsSolver
from global_ik import GlobalIK
from balance_controller import BalanceController
from com_calculator import COMCalculator
from ground_reaction_model import add_ground
from swing_foot_trajectory_generator import swing_foot_traj_generator
from swing_foot_trajectory_generator import fsm_state

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

def plot_swing_foot_traj(swing_foot_traj):
    '''This function takes in the bezier curve and plots it in the world frames
        using matplotlib.'''
    curve_points = []

    # Loop over t values and evaluate the Bézier curve at each t
    t_values = np.linspace(0, 1, 100)
    for i in t_values:
        point = swing_foot_traj.value(i).flatten()  # Flatten to make it 1D
        curve_points.append(point)

    # # Convert the list to a numpy array
    curve_points = np.array(curve_points)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(curve_points[:, 0], curve_points[:, 1], curve_points[:, 2], label="Swing Foot Trajectory")
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    ax.grid(True)
    ax.set_title("Swing Foot Trajectory (3D Bézier Curve)")

    plt.show()

    return


def main():

    # meshcat_utils = MeshcatUtils(port=7000)
    # meshcat = meshcat_utils.meshcat
    meshcat = StartMeshcat()
#
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0)  # Continuous model

    robot_url = "file://drake_models/home/sharanya/workspace/nimbus.sunny/lower_body.urdf"
    parser = Parser(plant)
    parser.AddModelsFromUrl(robot_url)

    add_ground(plant, soft_contact=True)  # Implement soft contact in add_ground function
    plant.Finalize()

    print_model_info(plant)

    AddDefaultVisualization(builder=builder, meshcat=meshcat)

    diagram = builder.Build()
    diagram.set_name("Foundation_bot")

    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()
    plant_context = plant.GetMyContextFromRoot(context)

    # balance_controller = BalanceController(Kp=0.5, Kd=0.1, max_correction=0.005)  # Further reduced gains
    # com_calculator = COMCalculator(plant)

    # Create initial pose trajectory
    initial_height = 0.9  #Start robot on the ground
    initial_pose_trajectory = create_initial_pose_trajectory(initial_height, duration=2.0)

    # Set initial joint positions
    q0 = plant.GetPositions(plant_context)
    print(plant.GetPositionNames())
    # Adjust these values based on your robot's joint configuration
    q0[7:12] = [-0.25, 0, 0, -0.65, 0.4]  # Left leg
    q0[12:] = [0.25, 0, 0, -0.65, 0.4]  # Right leg
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

    simulator.set_target_realtime_rate(0.1)
    simulator.Initialize()
    # X_WB_right = plant.EvalBodyPoseInWorld(plant_context, plant.GetBodyByName("foot"))
    # print("This is where the right foot is = ", X_WB_right)

    # gait_planner = GaitPlanner(step_duration=5.0)  # Slower gait for stability
    # initializing with right foot.
    isLeftFoot = False
    phase = 0
    step_duration = 5

    # initializing swing foot by calling the fsm_state function. This should
    # return phase = 0 and isLeftFoot = False.
    phase, isLeftFoot = fsm_state(phase, step_duration, isLeftFoot)
    ik_solver = GlobalIK(plant, plant_context)
    ik_solver.set_body_position_costs([100, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
    ik_solver.set_body_orientation_costs([100, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
    fixed_foot_orientation_left = (plant.EvalBodyPoseInWorld(plant_context, plant.GetBodyByName("foot_2"))).rotation()
    fixed_foot_orientation_right = (plant.EvalBodyPoseInWorld(plant_context, plant.GetBodyByName("foot"))).rotation()
    # @sharanya there's something slightly fishy about the y values for the feet, they're not symmetric for some reason.
    # TODO: Look into this later.
    print("left foot position ", plant.EvalBodyPoseInWorld(plant_context, plant.GetBodyByName("foot_2")).translation())
    print("right foot position ", plant.EvalBodyPoseInWorld(plant_context, plant.GetBodyByName("foot")).translation())
    # pdb.set_trace()

    # Draw diagram
    DrawAndSaveDiagram(diagram)

    # # Main simulation loop
    simulation_time = 100  # Total simulation time including initial pose
    meshcat.StartRecording()

    # temp parameter to track the curve traversal time. This might need to change
    # based on motion smoothness.
    curve_dt = 1/step_duration
    while context.get_time() < simulation_time:
        t = context.get_time()
        print(t)

        # @sharanya look into why there is a -2.0 here.why are we shifting this
        # to the right in time by 2?
        # left_foot, right_foot = gait_planner.get_foot_positions(t - 2.0)

        # Check if the swing foot has changed. If it has, update the phase and
        # fsm state.
        prev_swing_foot = isLeftFoot
        phase, isLeftFoot = fsm_state(phase, step_duration, isLeftFoot)

        # Plan new swing foot trajectory only if the swing foot has changed.
        # else, query the current trajectory for the current phase.
        if isLeftFoot != prev_swing_foot or context.get_time() == 0:
            if isLeftFoot:
                X_WB = plant.EvalBodyPoseInWorld(plant_context, plant.GetBodyByName("foot_2"))
                target = RigidTransform(RotationMatrix(), [0, 0, 0])
                target.set_rotation(fixed_foot_orientation_left)
                target.set_translation(X_WB.translation() + np.array([0.25, 0, 0]))
            else:
                X_WB = plant.EvalBodyPoseInWorld(plant_context, plant.GetBodyByName("foot"))
                target = RigidTransform(RotationMatrix(), [0, 0, 0])
                target.set_rotation(fixed_foot_orientation_right)
                target.set_translation(X_WB.translation() + np.array([0.25, 0, 0]))

            swing_foot_traj = swing_foot_traj_generator(current_footstep = X_WB, target_footstep=target, clearance=0.07)


        # plot for debugging
        # plot_swing_foot_traj(swing_foot_traj)
        # pdb.set_trace()

        # # print(X_WB_right)
        # print("This is where the right foot is = ", X_WB_right.rpy())
        # X_WB_left = plant.EvalBodyPoseInWorld(plant_context, plant.GetBodyByName("foot_2"))
        print("current phase ", phase)
        print("Current pelvis pose ", plant.EvalBodyPoseInWorld(plant_context, plant.GetBodyByName("pelvis")))
        print("current swing foot pose ", X_WB)
        ik_target = RigidTransform()
        print("ik target before setting ", ik_target)
        # @sharanya TODO: Maybe this should set this to a fixed orientation equal to the initial orientation instead?
        # The orientation of the foot is currently not aligned with the world frame. The constraints are in the
        # world frame. This might be the issue.
        # set the translation to be some z height higher.
        ik_target.set_translation(X_WB.translation() + np.array([0, 0, 0.10]))
        # ik_target.set_translation(swing_foot_traj.value(phase))
        print("ik target final ", ik_target)
        q_sol = ik_solver.joint_position_command_generator(ik_target, isLeftFoot)
        pdb.set_trace()
        plant.SetPositions(plant_context, q_sol)
        phase += dt

        # pdb.set_trace()




        # print("This is where the left foot is = ", X_WB_left.translation())
        # @sharanya these angles make no sense
        # left_leg_angles = ik_solver.solve(*left_foot)
        # print("left leg angles ik = ", left_leg_angles)
        # right_leg_angles = ik_solver.solve(*right_foot)
        # print("right leg angles ik = ", right_leg_angles)

        # current_com = com_calculator.calculate_com(plant_context)
        # # @sharanya this initial height is wrong! It's where the simulation begins.
        # desired_com = np.array([0, 0, current_com])
        # com_correction = balance_controller.compute_correction(desired_com, \
        #                                                        current_com, dt)
        # # #
        # all_positions = plant.GetPositions(plant_context)
        # print("all positions ", all_positions)
        #
        # # @sharanya check to see if these angles make sense.
        # all_positions[7:11] = left_leg_angles
        # all_positions[11:15] = right_leg_angles
        # plant.SetPositions(plant_context, all_positions)
        #
        # X_WP = plant.GetFreeBodyPose(plant_context, pelvis)
        # new_translation = X_WP.translation() + com_correction
        # X_WP.set_translation(new_translation)
        # plant.SetFreeBodyPose(plant_context, pelvis, X_WP)

        simulator.AdvanceTo(t + dt)

    print("Simulation complete. Meshcat URL:")
    print(meshcat.web_url())

    meshcat.PublishRecording()
    input("Press Enter to exit...")

if __name__ == "__main__":
    main()
