import numpy as np

class GaitPlanner:
    def __init__(self, step_length=0.2, step_height=0.05, step_duration=1.0):
        self.step_length = step_length
        self.step_height = step_height
        self.step_duration = step_duration

    def generate_foot_trajectory(self, t, is_left):
        print("time in gait planner ", t)
        # Normalize time within the step cycle
        t_norm = (t % self.step_duration) / self.step_duration
        print("normalized time in gait planner ", t_norm)

        # Phase shift for right foot
        if not is_left:
            t_norm = (t_norm + 0.5) % 1.0

        # X position
        x = self.step_length * (t_norm - 0.5)

        # Y position (constant, but different for each foot)
        y = 0.1 if is_left else -0.1

        # Z position (simple sine wave for foot lift)
        z = self.step_height * np.sin(np.pi * t_norm) if t_norm < 0.5 else 0

        return np.array([x, y, z])

    def get_foot_positions(self, t):
        left_foot = self.generate_foot_trajectory(t, is_left=True)
        right_foot = self.generate_foot_trajectory(t, is_left=False)
        return left_foot, right_foot
