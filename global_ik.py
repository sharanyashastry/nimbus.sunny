from pydrake.multibody.inverse_kinematics import GlobalInverseKinematics
from pydrake.math import RigidTransform
import numpy as np
from pydrake.solvers import Solve
# import gurobipy
# from pydrake.solvers import
from pydrake.solvers import MosekSolver

class GlobalIK:
    def __init__(self, plant, plant_context):
        self.global_ik_solver = GlobalInverseKinematics(plant)
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
        R_hip_roll_joint = self.plant.GetJointByName("R_hip_roll")
        R_hip_roll_joint.Lock(self.plant_context)
        R_hip_yaw_joint = self.plant.GetJointByName("R_hip_yaw")
        R_hip_yaw_joint.Lock(self.plant_context)
        L_hip_roll_joint = self.plant.GetJointByName("L_hip_roll")
        L_hip_roll_joint.Lock(self.plant_context)
        L_hip_yaw_joint = self.plant.GetJointByName("L_hip_yaw")
        L_hip_yaw_joint.Lock(self.plant_context)
        print("Locked hip roll and yaw joints.")
        return

    def joint_position_command_generator(self, footstep_pose, isLeftFoot):
        # Lock the hip joints
        self.lock_hip_joints()

        # get the foot body index based on swing_foot
        if(not isLeftFoot):
            # get right foot body_index
            sw_foot_body_index = self.plant.GetBodyByName("foot").index()
            X_WB = self.plant.EvalBodyPoseInWorld(self.plant_context, self.plant.GetBodyByName("foot"))

            st_foot_body_index = self.plant.GetBodyByName("foot_2").index()
            X_WS = self.plant.EvalBodyPoseInWorld(self.plant_context, self.plant.GetBodyByName("foot_2"))

        else:
            sw_foot_body_index = self.plant.GetBodyByName("foot_2").index()
            X_WB = self.plant.EvalBodyPoseInWorld(self.plant_context, self.plant.GetBodyByName("foot_2"))

            st_foot_body_index = self.plant.GetBodyByName("foot").index()
            X_WS = self.plant.EvalBodyPoseInWorld(self.plant_context, self.plant.GetBodyByName("foot"))



        # Set up mathematical program
        # TODO: Constraint needs to be fixed.
        # p_BQ is point Q in body frame {B} which as per my understanding is axis aligned with the world frame.
        print("Footstep pose in IK ", footstep_pose)
        self.global_ik_solver.AddWorldPositionConstraint(body_index = sw_foot_body_index,
                                                    p_BQ = np.array([0,0,0]),
                                                    X_WF = RigidTransform(),
                                                    box_lb_F = footstep_pose.translation(),
                                                    box_ub_F = footstep_pose.translation())

        pelvis_index = self.plant.GetBodyByName("pelvis").index()
        X_WP = self.plant.EvalBodyPoseInWorld(self.plant_context, self.plant.GetBodyByName("pelvis"))
        print("Pelvis pose in IK ", X_WP)

        # Adding constraint on stance foot to be at the same position.
        self.global_ik_solver.AddWorldPositionConstraint(body_index = st_foot_body_index,
                                                        p_BQ = np.array([0,0,0]),
                                                        X_WF = RigidTransform(),
                                                        box_lb_F = X_WS.translation(),
                                                        box_ub_F = X_WS.translation())
        # self.global_ik_solver.AddPostureCost(self.plant.GetPositions(self.plant_context),
        #                                      self.q_body_position_costs, self.q_body_orienation_costs)

        # TODO: Can try to do better here by using similar principle to inverted pendulum dynamics
        # or simply a bounding box centered at halfway distancy in x-y plane and z set at 0.9
        self.global_ik_solver.AddWorldPositionConstraint(body_index = pelvis_index,
                                                    p_BQ = np.array([0,0,0]),
                                                    X_WF = RigidTransform(),
                                                    box_lb_F = X_WP.translation() + np.array([-0.1, -0.1, -0.1]),
                                                    box_ub_F = X_WP.translation() + np.array([0.1, 0.1, 0.1]))

        # get pelvis pose in world as quaternion because the orientation constraint is in quaternion.
        # # TODO: Use a fixed orientaion for this.
        # quat_wxyz = X_WP.rotation().ToQuaternion()
        # print("Pelvis orientaion constraint in IK ", quat_wxyz)
        # self.global_ik_solver.AddWorldOrientationConstraint(body_index = pelvis_index,
        #                                             desired_orientation = quat_wxyz,
        #                                             angle_tol = 0.2)

        # TODO: Add switching logic here as well and add stance foot constraint.

        prog = self.global_ik_solver.get_mutable_prog()
        # gurobi_solver = GurobiSolver()
        mosek_solver = MosekSolver()
        # print(mosek_solver.is_available())
        # prog.SetSolverOption(gurobi_solver.solver_id(), "OutputFlag", 0)
        result = mosek_solver.Solve(prog)
        # result = Solve(prog)
        solution = self.global_ik_solver.ReconstructGeneralizedPositionSolution(result)
        # prog = self.global_ik_solver.get_mutable_prog()
        # Solve(prog)
        # solution = prog.GetSolution()
        return solution
