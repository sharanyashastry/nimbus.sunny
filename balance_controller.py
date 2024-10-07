import numpy as np

class BalanceController:
    def __init__(self, Kp=1.0, Kd=0.5, Ki=0.1, max_correction=0.01):
        self.Kp = Kp  # Proportional gain
        self.Kd = Kd  # Derivative gain
        self.Ki = Ki  # Integral gain
        self.prev_error = np.zeros(3)
        self.integral_error = np.zeros(3)
        self.max_correction = max_correction

    def compute_correction(self, desired_com, actual_com, dt):
        # Compute error
        error = desired_com - actual_com
        
        # Update integral error
        self.integral_error += error * dt
        
        # Compute error derivative
        error_derivative = (error - self.prev_error) / dt
        
        # Compute correction
        correction = (self.Kp * error + 
                      self.Kd * error_derivative + 
                      self.Ki * self.integral_error)
        
        # Limit the magnitude of the correction
        correction_magnitude = np.linalg.norm(correction)
        if correction_magnitude > self.max_correction:
            correction = correction * (self.max_correction / correction_magnitude)
        
        # Update previous error
        self.prev_error = error
        
        return correction