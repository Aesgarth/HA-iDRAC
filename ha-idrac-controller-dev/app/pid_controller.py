# HA-iDRAC/ha-idrac-controller-dev/app/pid_controller.py
import time

class PIDController:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint

        self.integral = 0
        self.last_error = 0
        self.last_time = time.time()

        self.output_min = 15
        self.output_max = 95

    def update(self, current_value, base_fan_speed):
        """Calculates the new fan speed based on current temp and a base speed."""
        current_time = time.time()
        delta_time = current_time - self.last_time
        
        if delta_time == 0:
            return None

        # Calculate the error (how far we are from the target)
        error = current_value - self.setpoint
        
        # Proportional term: Reacts to the current error
        P_out = self.Kp * error
        
        # Integral term: Accumulates past errors to eliminate steady-state error
        self.integral += error * delta_time
        self.integral = max(min(self.integral, 20), -20) # Anti-windup
        I_out = self.Ki * self.integral
        
        # Derivative term: Predicts future error
        derivative = (error - self.last_error) / delta_time
        D_out = self.Kd * derivative
        
        # The final output is the base speed plus the PID adjustment
        output = base_fan_speed + P_out + I_out + D_out
        
        # Clamp the output to safe fan speed limits
        output = max(min(output, self.output_max), self.output_min)
        
        # Update state for the next loop
        self.last_error = error
        self.last_time = current_time
        
        return int(output)

    def set_gains(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def load_state(self, state):
        self.integral = state.get('integral', 0)

    def get_state(self):
        return {'integral': self.integral}