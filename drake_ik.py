from pydrake.multibody.inverse_kinematics import InverseKinematics

from pydrake.math import RigidTransform
import numpy as np
from pydrake.solvers import Solve
# import gurobipy
# from pydrake.solvers import
from pydrake.solvers import MosekSolver

class IK:
    def __init__(self, plant, plant_context):
        self.ik_solver = InverseKinematics(plant)
        self.plant = plant
        self.plant_context = plant_context
        self.lock_hip_joints()
        self.q_body_position_costs = np.zeros(plant.num_bodies())
        self.q_body_orienation_costs = np.zeros(plant.num_bodies())
        # self.grbmodel = gurobipy.Model(outputflag=0)

    def set_body_position_costs(self, body_positions):
        self.q_body_position_costs = body_positions
        return

    def set_body_orientation_costs(self, body_orientations):
        self.q_body_orienation_costs = body_orientations
        return

    def lock_hip_joints(self):
        # Lock the hip joints
        # Get the joint index for the hip joints
        # R_hip_roll_joint = self.plant.GetJointByName("R_hip_roll")
        # R_hip_roll_joint.Lock(self.plant_context)
        R_hip_yaw_joint = self.plant.GetJointByName("R_hip_yaw")
        R_hip_yaw_joint.Lock(self.plant_context)
        # L_hip_roll_joint = self.plant.GetJointByName("L_hip_roll")
        # L_hip_roll_joint.Lock(self.plant_context)
        L_hip_yaw_joint = self.plant.GetJointByName("L_hip_yaw")
        L_hip_yaw_joint.Lock(self.plant_context)
        # R_ankle_joint = self.plant.GetJointByName("R_ankle")
        # R_ankle_joint.Lock(self.plant_context)
        # L_ankle_joint = self.plant.GetJointByName("L_ankle")
        # L_ankle_joint.Lock(self.plant_context)
        print("Locked hip roll and yaw joints.")
        return

    def joint_position_command_generator(self, footstep_pose, isLeftFoot):
        # Lock the hip joints
        self.lock_hip_joints()

        # get the foot body index based on swing_foot
        if(not isLeftFoot):
            # get right foot body_index
            sw_foot_body = self.plant.GetBodyByName("foot")
            sw_foot_frame = sw_foot_body.body_frame()
            sw_foot_body_index = sw_foot_body.index()
            X_WB = self.plant.EvalBodyPoseInWorld(self.plant_context, self.plant.GetBodyByName("foot"))

            st_foot_body = self.plant.GetBodyByName("foot_2")
            st_foot_frame = st_foot_body.body_frame()
            st_foot_body_index = st_foot_body.index()
            X_WS = self.plant.EvalBodyPoseInWorld(self.plant_context, self.plant.GetBodyByName("foot_2"))

        else:
            # get left foot body_index
            sw_foot_body = self.plant.GetBodyByName("foot_2")
            sw_foot_frame = sw_foot_body.body_frame()
            sw_foot_body_index = sw_foot_body.index()
            X_WB = self.plant.EvalBodyPoseInWorld(self.plant_context, self.plant.GetBodyByName("foot_2"))

            st_foot_body = self.plant.GetBodyByName("foot")
            st_foot_frame = st_foot_body.body_frame()
            st_foot_body_index = st_foot_body.index()
            X_WS = self.plant.EvalBodyPoseInWorld(self.plant_context, self.plant.GetBodyByName("foot"))


        # Setting up the IK problem.
        world_frame = self.plant.world_frame()
        print("world frame ", world_frame)
        pelvis_frame = self.plant.GetBodyByName("pelvis").body_frame()
        # print("sw foot frame ",)
        # print("st foot frame ", st_foot_frame.rotation(), st_foot_frame.translation())

        self.ik_solver.AddPositionConstraint(sw_foot_frame, np.array([0, 0, 0]), 
                                             world_frame, footstep_pose.translation() - np.array([0.01, 0.01, 0.0]), 
                                             footstep_pose.translation() + np.array([0.01, 0.01, 0.0]))
        # Add stance foot constraint. The first constraints only enforces a point contact constraint.
        self.ik_solver.AddPositionConstraint(st_foot_frame, np.array([0, 0, 0]), 
                                             world_frame, X_WS.translation(), 
                                             X_WS.translation())
        # This second constraint is meant to enforce a planar geometric contact constraint.
        # self.ik_solver.AddPositionConstraint(st_foot_frame, X_WS.translation() + np.array([-0.04, 0, 0]),
        #                                      world_frame, X_WS.translation() + np.array([0.04, 0, 0]),
        #                                      X_WS.translation() + np.array([0.04, 0, 0]))

        # Add pelvis cost
        

        # TODO: Add switching logic here as well and add stance foot constraint.

        # prog = self.ik_solver.get_mutable_prog()
        result = Solve(self.ik_solver.prog())
        solution = result.GetSolution()
        return solution