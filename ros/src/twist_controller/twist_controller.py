import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

# Global Constants
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_radius, wheel_base, steer_ratio, max_lat_accel,
                 max_steer_angle, min_speed=0.1):
        # TODO: Implement
        # Initialise Yaw Control
        self.yaw_control = YawController(wheel_base, steer_ratio, min_speed,
                                         max_lat_accel, max_steer_angle)
        # Initialise PID Control
        self.pid_control = PID(kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM)

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return 1., 0., 0.

    def reset_PID(self):
        '''
        Resets the PID Controller
        '''
        self.pid_control.reset()
