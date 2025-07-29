class PID_Ctrl():
    def __init__(self,kp=0.0000028, ki=0.000000003, kd=0.000000, set_point=640):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # Setpoint (target position)
        self.setpoint = set_point  # Center point [Yaw, Pitch]
        
        self.error = 0
        self.last_error = 0
        self.integral = 0
        self.output = None

    def calculate(self, process_variable):
        self.output = 0
        self.error = (self.setpoint - process_variable) * -1
        self.integral += self.error
        derivative_0 = self.error - self.last_error
        self.output = (self.kp * self.error) + (self.ki * self.integral) + (self.kd * derivative_0)
        self.last_error = self.error

        return self.output

    def pid_run(self, val):
        """Returns output degrees and errors

        Returns:
            output: PID calculation result
            error : error values
        """
        self.output = self.calculate(val)
        return self.output