import numpy as np

class InverseKinematicsSolver:
    def __init__(self, hip_offset=0.1, upper_leg_length=0.4, lower_leg_length=0.4):
        self.hip_offset = hip_offset
        self.l1 = upper_leg_length
        self.l2 = lower_leg_length

    def solve(self, x, y, z):
        # Adjust for hip offset
        y -= self.hip_offset if y > 0 else -self.hip_offset
        
        # Calculate hip yaw
        hip_yaw = np.arctan2(y, x)
        
        # Transform to 2D problem in the YZ plane
        r = np.sqrt(x**2 + y**2)
        
        # Calculate hip pitch and knee angles using cosine law
        d = np.sqrt(r**2 + z**2)
        cos_knee = (self.l1**2 + self.l2**2 - d**2) / (2 * self.l1 * self.l2)
        knee_angle = np.arccos(np.clip(cos_knee, -1.0, 1.0))
        
        alpha = np.arctan2(z, r)
        beta = np.arccos((self.l1**2 + d**2 - self.l2**2) / (2 * self.l1 * d))
        hip_pitch = alpha + beta
        
        # Calculate hip roll
        hip_roll = np.arcsin(y / r)
        
        return np.array([hip_yaw, hip_roll, hip_pitch, knee_angle])