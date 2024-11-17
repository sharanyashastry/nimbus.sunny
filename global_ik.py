from pydrake.multibody.inverse_kinematics import GlobalInverseKinematics
import numpy as np
from pydrake.solvers import Solve
import gurobipy
# from pydrake.solvers import 
from pydrake.solvers import MosekSolver

class GlobalIK:
    def __init__(self, plant, plant_context, isLeftFoot):
        self.global_ik_solver = GlobalInverseKinematics(plant)
        self.plant = plant
        self.plant_context = plant_context
        self.isLeftFoot = isLeftFoot
        # self.grbmodel = gurobipy.Model(outputflag=0)

    def joint_position_command_generator(self, footstep_pose):
        # get the foot body index based on swing_foot
        if(not self.isLeftFoot):
            # get right foot body_index
            foot_body_index = self.plant.GetBodyByName("foot").index()
            X_WB = self.plant.EvalBodyPoseInWorld(self.plant_context, self.plant.GetBodyByName("foot"))
        else:
            foot_body_index = self.plant.GetBodyByName("foot_2").index()
            X_WB = self.plant.EvalBodyPoseInWorld(self.plant_context, self.plant.GetBodyByName("foot_2"))

        

        # Set up mathematical program
        # TODO: Constraint needs to be fixed.
        # p_BQ is point Q in body frame {B} which as per my understanding iss axis aligned with the world frame.
        self.global_ik_solver.AddWorldPositionConstraint(body_index = foot_body_index,
                                                    p_BQ = np.array([0,0,0]),
                                                    X_WF = footstep_pose,
                                                    box_lb_F = [0.01, 0.01, 0.01],
                                                    box_ub_F = [0.01, 0.01, 0.01])
        
        pelvis_index = self.plant.GetBodyByName("pelvis").index()
        X_WP = self.plant.EvalBodyPoseInWorld(self.plant_context, self.plant.GetBodyByName("pelvis"))
        self.global_ik_solver.AddWorldPositionConstraint(body_index = pelvis_index,
                                                    p_BQ = np.array([0,0,0]),
                                                    X_WF = X_WP,
                                                    box_lb_F = [1, 1, 0.05],
                                                    box_ub_F = [1, 1, 0.05])
        
        # get pelvis pose in world as quaternion because the orientation constraint is in quaternion.
        quat_wxyz = X_WP.rotation().ToQuaternion()
        self.global_ik_solver.AddWorldOrientationConstraint(body_index = pelvis_index,
                                                    desired_orientation = quat_wxyz,
                                                    angle_tol = 0.2)
        
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
