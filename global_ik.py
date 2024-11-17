from pydrake.multibody.inverse_kinematics import GlobalInverseKinematics

class GlobalIK:
    def __init__(self, plant, isLeftFoot):
        self.global_ik_solver_ = GlobalInverseKinematics()
        self.plant_ = plant
        self.isLeftFoot = isLeftFoot

    def joint_position_command_generator(self):
        # get the foot body index based on swing_foot
        if(not isLeftFoot):
            # get right foot body_index
            body_index = plant.GetBodyByName("foot").index()
            X_WB = plant.EvalBodyPoseInWorld(plant, plant.get_context(), plant.GetBodyByName("foot"))
        else:
            body_index = plant.GetBodyByName("foot_2").index()
            X_WB = plant.EvalBodyPoseInWorld(plant, plant.get_context(), plant.GetBodyByName("foot_2"))


        # Set up mathematical program
        # TODO: Constraint needs to be fixed.
        # p_BQ is point Q in body frame {B} which as per my understanding iss axis aligned with the world frame.
        global_ik_solver.AddWorldPositionConstraint(body_index = body_index,
                                                    p_BQ = np.array([0,0,-0.04]),
                                                    X_WF = X_WB,
                                                    box_lb_F = [0.01, 0.01, 0.01],
                                                    box_ub_F = [0.01, 0.01, 0.01])

        result = global_ik_solver.get_mutable_prog().solve()
        solution = result.GetSolution()
        return solution
