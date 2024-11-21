import numpy as np
from pydrake.trajectories import BezierCurve
import pdb


def fsm_state(phase, T, isLeftFoot):
    # T is the step_duration parameter and phase represents what point along the
    # step you are in time.
    if phase >= T:
        isLeftFoot = not isLeftFoot
        phase = 0
    return phase, isLeftFoot

def swing_foot_traj_generator(current_footstep, target_footstep, clearance):
    ''' This function generates a pose trajectory for the swing foot to track.
        It takes in the pose of the current and target footsteps as expressed in
        the world frame and returns a trajectory.'''
    current_footstep_position = np.array(current_footstep.translation())
    target_footstep_position = np.array(target_footstep.translation())
    # print("current footstep position ", current_footstep_position)
    # print("target footstep position ", target_footstep_position)
    # xyz locations of footstep. This is currently only generating way points for moving in the y direction.
    p1 = current_footstep_position + 0.25*(target_footstep_position - current_footstep_position)
    p2 = current_footstep_position + 0.75*(target_footstep_position - current_footstep_position)

    # clearance in the z direction.
    p1[2] = current_footstep_position[2] + clearance
    p2[2] = current_footstep_position[2] + clearance

    control_points = np.array([current_footstep_position, p1, p2, target_footstep_position]).T
    # print("control points ", control_points.shape)
    # print("control points ", control_points)

    # instantiate a bezier curve that interpolates the position between them. This is in the s parameter space.
    swing_foot_traj = BezierCurve(start_time = 0, end_time = 1, control_points = control_points)
    # pdb.set_trace()

    # Spherically interpolate the pose. TODO: Implement the orientation way points for this later.
    # current_footstep_orientation = current_footstep.rotation()
    # target_footstep_orientation = target_footstep.rotation()

    return swing_foot_traj
