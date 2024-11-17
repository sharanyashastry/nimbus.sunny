import numpy as np
from pydrake.trajectories import BezierCurve

def swing_foot_traj_generator(current_footstep, target_footstep, clearance):
    ''' This function generates a pose trajectory for the swing foot to track.
        It takes in the pose of the current and target footsteps as expressed in
        the world frame and returns a trajectory.'''
    current_footstep_position = np.array(current_footstep.translation())
    target_footstep_position = np.array(target_footstep.translation())
    # xyz locations of footstep. This is currently only generating way points for moving in the y direction.
    p1 = current_footstep_position + 0.25*(target_footstep_position - current_footstep_position)
    p2 = current_footstep_position + 0.75*(target_footstep_position - current_footstep_position)

    # clearance in the z direction.
    p1[2] = clearance
    p2[2] = clearance

    control_points = np.array([current_footstep_position, p1, p2, target_footstep_position])

    # instantiate a bezier curve that interpolates the position between them. This is in the s parameter space.
    swing_foot_traj = BezierCurve(start_time = 0, end_time = 1, control_points)

    # Spherically interpolate the pose. TODO: Implement the orientation way points for this later.
    # current_footstep_orientation = current_footstep.rotation()
    # target_footstep_orientation = target_footstep.rotation()

    return swing_foot_traj
