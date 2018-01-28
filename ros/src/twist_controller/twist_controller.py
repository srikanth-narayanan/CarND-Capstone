import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

# Global Constants
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, wheel_radius, wheel_base, steer_ratio,
                 max_lat_accel, max_steer_angle, decel_limit, accel_limit,
                 brake_deadband, fuel_capacity, min_speed=1):
        '''
        Initialise all controller as needed for calculations
        '''
        # Initialise Yaw Control
        self.yaw_control = YawController(wheel_base, steer_ratio, min_speed,
                                         max_lat_accel, max_steer_angle)
        # Initialise PID Control
        self.pid_accel = PID(kp=1.5, ki=0.0, kd=0.01, mn=decel_limit, mx=accel_limit)
        self.pid_steer = PID(kp=0.5, ki=0.0, kd=0.2, mn=-max_steer_angle, mx=max_steer_angle)
        # Lowpass filter for Steering
        self.lowpass_steering = LowPassFilter(0.00, 0.02)
        #Low pass filter for throttle
        self.lowpass_throttle = LowPassFilter(0.00, 0.02)
        # Initialise constants
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity= fuel_capacity
        self.brake_deadband = brake_deadband
        self.wheel_radius = wheel_radius
        self.DEBUG_STAT = False

    def control(self, twist_cmd, current_velocity, target_velocity, delta_time, cte):
        '''
        Run controller based on current values to determine optimal steering,
        brake and throttle.
        '''
        linear_velocity = abs(twist_cmd.twist.linear.x)
        angular_velocity = twist_cmd.twist.angular.z

        # Using Yaw Controller get steering values
        if current_velocity.twist.linear.x > 0.05:
            steering_angle = self.yaw_control.get_steering(linear_velocity,
                                                           angular_velocity,
                                                           current_velocity.twist.linear.x)

        else:
            steering_angle = 0.0

        ### Corrective Steering
        correct_steer = self.pid_steer.step(cte, delta_time)
        steering_angle_filtered = 1.0 * steering_angle + 0.2 * correct_steer
        steering_angle_filtered = self.lowpass_steering.filt(steering_angle)

        # Using PID for Throttle control
        linear_velocity_error = linear_velocity - current_velocity.twist.linear.x
        pid_acceleration = self.pid_accel.step(linear_velocity_error, delta_time)
        a_ego_filtered = self.lowpass_throttle.filt(pid_acceleration)

        # calculate brake force needed if acceleration is not positive and target speed is not (close to) zero
        if a_ego_filtered > 0.0 and target_velocity > 0.05:
            #throttle = a_ego_filtered
            throttle = pid_acceleration
            brake_torque = 0.0 # Ensure brakes are not applied when accelerating
        else:
            throttle = 0.0 # Ensure throttle is not applied when braking
            deccel_request = abs(a_ego_filtered)
            # check if the deceleration needed is lesser than brake dead band
            # (This might just be priming force for the callipers or drums)
            if deccel_request < self.brake_deadband:
                deccel_request = 0.0

            # Calculate Brake force as brake torque in Nm
            brake_torque = (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * deccel_request * self.wheel_radius

        if self.DEBUG_STAT:
            rospy.loginfo("Steering_Filt : %f, Accel_Unfilt : %f, Accel_Filt : %f, Brake : %f" % (steering_angle,
                                                                                                  pid_acceleration,
                                                                                                  a_ego_filtered,
                                                                                                  brake_torque))

        # Return throttle, brake, steer
        return throttle, brake_torque, steering_angle_filtered

    def reset_PID(self):
        '''
        Resets the PID Controller
        '''
        self.pid_accel.reset()
        self.pid_steer.reset()
