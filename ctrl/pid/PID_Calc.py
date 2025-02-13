import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from utils import JsonHandler, check_file
json_file = check_file("config.json")
json_config = JsonHandler(json_file)
VIDEO_WIDTH = json_config.get(["video_resolutions", "default", "width"])
VIDEO_HEIGHT = json_config.get(["video_resolutions", "default", "height"])

class PID_Ctrl():
    def __init__(self,kp=0.0147, ki=0.00000036, kd=0.00000033, set_point=(VIDEO_WIDTH/2, VIDEO_HEIGHT/2)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # Setpoint (target position)
        self.setpoint = list(set_point)  # Center point [Yaw, Pitch]
        
        self.error = [0, 0]
        self.last_error = [0, 0]
        self.integral = [0, 0]
        self.output = [None, None]

    def calculate(self, process_variable):
        self.output = [0, 0]
        ### yaw ###
        self.error[0] = (self.setpoint[0] - process_variable[0]) * -1
        self.integral[0] += self.error[0]
        derivative_0 = self.error[0] - self.last_error[0]
        self.output[0] = (self.kp * self.error[0]) + (self.ki * self.integral[0]) + (self.kd * derivative_0)
        self.last_error[0] = self.error[0]

        ### pitch ###
        self.error[1] = (self.setpoint[1] - process_variable[1]) * -1
        self.integral[1] += self.error[1]
        derivative_1 = self.error[1] - self.last_error[1]
        self.output[1] = (self.kp * self.error[1]) + (self.ki * self.integral[1]) + (self.kd * derivative_1)
        self.last_error[1] = self.error[1]

        return self.output[0], self.output[1]

    def pid_run(self, *args):
        """Returns output degrees and errors

        Returns:
            output: PID calculation result
            error : error values
        """
        self.output = self.calculate(args)
        return self.output, self.error