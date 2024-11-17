import numpy as np

class InverseKinematicsSolver:
    def __init__(self, hip_offset=0.1, upper_leg_length=0.4, lower_leg_length=0.4):
        self.hip_offset = hip_offset
        self.l1 = upper_leg_length
        self.l2 = lower_leg_length

    # def solve(self, x, y, z):
    #     # @sharanya look into this and make sure this is correct.
    #     # Adjust for hip offset
    #     y -= self.hip_offset if y > 0 else -self.hip_offset
    #
    #     # Calculate hip yaw
    #     hip_yaw = np.arctan2(y, x)
    #
    #     # Transform to 2D problem in the YZ plane
    #     r = np.sqrt(x**2 + y**2)
    #     print("xsq + ysq ",x**2 + y**2)
    #     print("r value ", r)
    #
    #     # Calculate hip pitch and knee angles using cosine law
    #     d = np.sqrt(r**2 + z**2)
    #     print("d value ", d)
    #     cos_knee = (self.l1**2 + self.l2**2 - d**2) / (2 * self.l1 * self.l2)
    #     # print("printing cosine value in IK = ", cos_knee)
    #     knee_angle = np.arccos(np.clip(cos_knee, -1.0, 1.0))
    #     # print("printing knee angle value in IK = ", knee_angle)
    #
    #     alpha = np.arctan2(z, r)
    #     print("alpha ", alpha)
    #     beta = np.arccos((self.l1**2 + d**2 - self.l2**2) / (2 * self.l1 * d))
    #     print("beta ", beta)
    #     hip_pitch = alpha + beta
    #     # print("hip pitch angle in IK ", hip_pitch)
    #
    #     # Calculate hip roll
    #     hip_roll = np.arcsin(y / r)
    #     # @sharanya is this order right? The positions seem to be in order hip pitch, roll, yaw
    #     return np.array([hip_pitch, hip_roll, hip_yaw, knee_angle])

    def solve(self, x, y, z):
        # @sharanya look into this and make sure this is correct.
        # Adjust for hip offset
        y -= self.hip_offset if y > 0 else -self.hip_offset

        # Calculate hip yaw
        hip_yaw = np.arctan2(y, x)

        # Transform to 2D problem in the YZ plane
        # r = np.sqrt(x**2 + y**2)
        # print("xsq + ysq ",x**2 + y**2)
        # print("r value ", r)

        # Calculate hip pitch and knee angles using cosine law
        d = np.sqrt(x**2 + z**2)
        print("d value ", d)
        cos_knee = (x**2 + z**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        # print("printing cosine value in IK = ", cos_knee)
        knee_angle = np.arccos(np.clip(cos_knee, -1.0, 1.0))
        # print("printing knee angle value in IK = ", knee_angle)

        alpha = np.arctan2(z, x)
        print("alpha ", alpha)
        beta = np.arccos((self.l1**2 + d**2 - self.l2**2) / (2 * self.l1 * d))
        print("beta ", beta)
        hip_pitch = alpha - beta
        # print("hip pitch angle in IK ", hip_pitch)

        # Calculate hip roll
        hip_roll = np.arcsin(z / y)
        # @sharanya is this order right? The positions seem to be in order hip pitch, roll, yaw
        return np.array([hip_pitch, hip_roll, hip_yaw, knee_angle])
