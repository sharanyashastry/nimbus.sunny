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
from pydrake.all import Integrator

from ground_reaction_model import add_ground
from swing_foot_trajectory_generator import swing_foot_traj_generator
from swing_foot_trajectory_generator import fsm_state
from drake_ik import IK

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

    add_ground(plant, soft_contact=True)  # Implement soft contact in add_ground function
    plant.Finalize()

    print_model_info(plant)

    AddDefaultVisualization(builder=builder, meshcat=meshcat)
