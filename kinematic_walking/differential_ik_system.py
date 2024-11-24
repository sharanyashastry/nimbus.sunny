
from pydrake.all import (
    JacobianWrtVariable,
    LeafSystem,
    MathematicalProgram,
    SnoptSolver)
import numpy as np

class DifferentialIKSystem(LeafSystem):

    def __init__(self, plant, plant_context):
        print("Initializing DifferentialIKSystem")
        LeafSystem.__init__(self)
        self._plant = plant
        self._plant_context = plant_context
        # self._plant_context = plant.CreateDefaultContext()
        self._world_frame = plant.world_frame()

        # Needs a constant vector sink for rotation velocity. 
        self.DeclareVectorInputPort("fsm_state_input : isLeftFoot, phase_input", 2)
        self.DeclareVectorInputPort("foot_velocity", 6)
        # self.DeclareVectorInputPort("nimbus.position_measured", 17)
        # self.DeclareVectorInputPort("nimbus.velocity_measured", 16)
        self.DeclareVectorInputPort("state [q,v]", plant.num_positions() + plant.num_velocities())
        self.DeclareVectorOutputPort("nimbus_velocity_command", 17, self.CalcOutput)

    def CalcOutput(self, context, output):
        fsm_state = self.get_input_port(0).Eval(context)[0]
        sw_foot_velocity_desired = self.get_input_port(1).Eval(context)
        # q_now = self.get_input_port(2).Eval(context)
        # qdot_now = self.get_input_port(3).Eval(context)
        num_positions = self._plant.num_positions()
        num_velocities = self._plant.num_velocities()
        q_now = self.get_input_port(2).Eval(context)[:num_positions]
        qdot_now = self.get_input_port(2).Eval(context)[num_positions:]

        self._plant.SetPositions(self._plant_context, q_now)
        if(fsm_state == 1):
            # This is the right foot
            print("Right foot in IK")
            sw_foot_body = self._plant.GetBodyByName("foot")
        else:
            print("Left foot in IK")
            sw_foot_body = self._plant.GetBodyByName("foot_2")


        sw_foot_frame = sw_foot_body.body_frame()
        # This might need to be kQV since we use quaternions for orientation.
        J_sw = self._plant.CalcJacobianSpatialVelocity(
            self._plant_context, JacobianWrtVariable.kQDot, 
            sw_foot_frame, [0, 0, 0], self._world_frame, self._world_frame)
        print("J_sw ", J_sw)
        
        # This is just representing the swingfoot body frame wrt world frame.
        X_WB = self._plant.CalcRelativeTransform(
            self._plant_context, self._world_frame, sw_foot_body.body_frame())
        foot_position_in_world = X_WB.translation()

        qdot_next = self.DiffIKQP(q_now, qdot_now, 
                                      sw_foot_velocity_desired, 
                                      foot_position_in_world, J_sw)
        # print("qdot_next ", qdot_next)
        output.SetFromVector(qdot_next)
        
    def DiffIKQP(self, q_now, qdot_now, sw_foot_velocity_desired, foot_position_in_world, J_sw):
        # This is the differential IK QP. 
        prog = MathematicalProgram()
        qdot = prog.NewContinuousVariables(17, "qdot")
        # TODO: Set this to max joint velocities as in urdf.
        v_max = 10

        for i in range(16):
            prog.AddBoundingBoxConstraint(-v_max, v_max, qdot[i])

        Q = np.dot(J_sw.T, J_sw)
        b = -np.dot(J_sw.T, sw_foot_velocity_desired)
        prog.AddQuadraticCost(Q, b, qdot)

        solver = SnoptSolver()
        result = solver.Solve(prog)

        if not (result.is_success()):
            raise ValueError("Could not find the optimal solution.")

        v_solution = result.GetSolution(qdot)

        return v_solution
