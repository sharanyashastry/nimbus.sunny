from pydrake.trajectories import BezierCurve
import pdb
from pydrake.all import LeafSystem
import numpy as np

class SwingFootTrajectoryGeneratorSystem(LeafSystem):
    def __init__(self, plant, plant_context):
        print("Initializing Swing Foot Trajectory Generator System")
        LeafSystem.__init__(self)
        self._plant = plant
        self._plant_context = plant_context
        self._world_frame = plant.world_frame()
        self.clearance = 0.07
        self.step_duration = 12

        self.DeclareVectorInputPort("fsm_state_input : isLeftFoot, phase_input", 2)
        # should be 17 + 16 for nimbus
        # self.DeclareVectorInputPort("nimbus.position_velocity_measured", plant.num_positions() + plant.num_velocities())
        # Needs to be set to 0 from a constant vector source.
        self.DeclareVectorInputPort("foot_angular_velocity", 3)
        self.DeclareVectorOutputPort("Swing foot velocity command", 6, self.CalcVelocityOutput)

        self._prev_fsm_state = None
        self._swing_foot_traj_position = None
        self._swing_foot_traj_velocity = None  

    def _generate_new_trajectory(self, fsm_state):
        # This function generates a new swing foot trajectory.
        # This is a simple linear trajectory for now.
        # The trajectory is a straight line from the current foot position to the target foot position.
        if fsm_state == 1.0:
            # This is the right foot
            X_WB = self._plant.EvalBodyPoseInWorld(self._plant_context, \
                                                   self._plant.GetBodyByName("foot_2"))
        else:
            X_WB = self._plant.EvalBodyPoseInWorld(self._plant_context, \
                                                   self._plant.GetBodyByName("foot"))
            
        current_footstep_position = X_WB.translation()
        # hardcoded relative fotstep position
        target_footstep_position = X_WB.translation() + np.array([0.25, 0, 0])

        # print("current footstep position ", current_footstep_position)
        # print("target footstep position ", target_footstep_position)
        # xyz locations of footstep. This is currently only generating way points for moving in the y direction.
        p1 = current_footstep_position + 0.25*(target_footstep_position - current_footstep_position)
        p2 = current_footstep_position + 0.75*(target_footstep_position - current_footstep_position)

        # clearance in the z direction.
        p1[2] = current_footstep_position[2] + self.clearance
        p2[2] = current_footstep_position[2] + self.clearance

        control_points = np.array([current_footstep_position, p1, p2, target_footstep_position]).T
        # print("control points ", control_points.shape)
        # print("control points ", control_points)

        # Spherically interpolate the pose. TODO: Implement the orientation way points for this later.
        # current_footstep_orientation = current_footstep.rotation()
        # target_footstep_orientation = target_footstep.rotation()

        # instantiate a bezier curve that interpolates the position between them. This is in the s parameter space.
        self._swing_foot_traj_position = BezierCurve(start_time = 0, end_time = 1, control_points = control_points)
        self._swing_foot_traj_velocity = self._swing_foot_traj_position.MakeDerivative()
        return

    def CalcVelocityOutput(self, context, output):
        fsm_state = self.get_input_port(0).Eval(context)[0]

        if fsm_state != self._prev_fsm_state or self._swing_foot_traj_position is None:
            self._generate_new_trajectory(fsm_state)
            self._prev_fsm_state = fsm_state
        
        curr_phase = self.get_input_port(0).Eval(context)[1]
        s = curr_phase/self.step_duration

        angular_velocity = self.get_input_port(1).Eval(context)
        linear_velocity = self._swing_foot_traj_velocity.value(s)
        target_velocity = np.zeros(6)
        target_velocity[:3] = angular_velocity
        target_velocity[3:] = linear_velocity.flatten()
        output.SetFromVector(target_velocity)    