# HA-iDRAC/ha-idrac-controller-dev/app/pid_controller.py
import time

class PIDController:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, setpoint=0):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.setpoint = setpoint # The target temperature

        self.integral = 0
        self.last_error = 0
        self.last_time = time.time()

        # Define output limits to prevent extreme fan speeds
        self.output_min = 15  # Minimum safe fan speed
        self.output_max = 95  # Maximum safe fan speed

    def update(self, current_value):
        """Calculates the new output value based on the current temperature."""
        current_time = time.time()
        delta_time = current_time - self.last_time
        
        if delta_time == 0:
            return None # Avoid division by zero

        error = self.setpoint - current_value
        
        # Proportional term
        P_out = self.Kp * error
        
        # Integral term
        self.integral += error * delta_time
        # Add anti-windup: clamp the integral term to prevent it from growing too large
        self.integral = max(min(self.integral, 20), -20)
        I_out = self.Ki * self.integral
        
        # Derivative term
        derivative = (error - self.last_error) / delta_time
        D_out = self.Kd * derivative
        
        # Calculate the total output
        output = P_out + I_out + D_out
        
        # Clamp the output to our defined limits
        output = max(min(output, self.output_max), self.output_min)
        
        # Update state for next iteration
        self.last_error = error
        self.last_time = current_time
        
        return int(output)

    def set_gains(self, Kp, Ki, Kd):
        """Allows tuning the controller's responsiveness."""
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def load_state(self, state):
        """Loads the integral from a saved state to provide persistence."""
        self.integral = state.get('integral', 0)

    def get_state(self):
        """Returns the current state for saving."""
        return {'integral': self.integral}