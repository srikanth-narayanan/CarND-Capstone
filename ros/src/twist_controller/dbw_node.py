#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane, Waypoint
import math

from twist_controller import Controller
import calc_steer_cte

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # Other variables needed for the DBW node By default set it True.
        # if a human driver takes over the subscribed node will change this status
        self.DBW_ENABLED = True
        self.current_twist_cmd = None
        self.current_velocity = None
        self.previous_time = rospy.get_time() # Get the time during instantiantion
        self.PID_RESET = True
        self.final_waypoints = None
        self.current_position = None


        # Create `TwistController` object
        self.controller = Controller(vehicle_mass, wheel_radius, wheel_base,
                                     steer_ratio, max_lat_accel,
                                     max_steer_angle, decel_limit,
                                     accel_limit, brake_deadband, fuel_capacity)

        # Subscribe to all the topics you need to
        # Get Drive By Wire Status (Human or Auto)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.on_receive_dbw_stat, queue_size=1)

        # Get twist command messages
        rospy.Subscriber('/twist_cmd', TwistStamped, self.on_receive_twist_cmd, queue_size=1)

        # Get current velocity status
        rospy.Subscriber('/current_velocity', TwistStamped, self.on_receive_current_vel, queue_size=1)

        # Get Current Position here for CTE calcs
        rospy.Subscriber('/current_pose', PoseStamped, self.on_receive_current_pose, queue_size=1)

        # Get Current Waypoints
        rospy.Subscriber('/final_waypoints', Lane, self.on_receive_waypoints, queue_size=1)

        self.loop()

    def on_receive_current_pose(self, msg):
        '''
        Call Back Function when Current Positions is received
        '''
        self.current_position = msg.pose # Get Message

    def on_receive_waypoints(self, message):
        '''
        Call back function when final waypoints array is received
        '''
        self.final_waypoints = message.waypoints

    def on_receive_dbw_stat(self, dbw_enabled):
        '''
        A callback function to set the status of current drive by wire
        '''
        try:
            self.DBW_ENABLED = bool(dbw_enabled.data) # cast it to python type
        except:
            self.DBW_ENABLED = dbw_enabled

    def on_receive_twist_cmd(self, twist_cmd):
        '''
        A callback function to deal when twist_cmd data is received from the
        waypoint follower.
        '''
        self.current_twist_cmd = twist_cmd

    def on_receive_current_vel(self, current_velocity):
        '''
        A calback function to deal when current velocity is received
        '''
        self.current_velocity = current_velocity

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():

            # Get predicted throttle, brake, and steering using `twist_controller`
            # Calculate Delta time for twist controller is called since instantiantion
            current_time = rospy.get_time()
            delta_time = current_time - self.previous_time
            self.previous_time = current_time

            # Predict values if DBW is enabled
            if self.DBW_ENABLED and self.current_velocity is not None and self.current_twist_cmd is not None and self.final_waypoints is not None:
                if self.PID_RESET:
                    self.controller.reset_PID()
                    self.PID_RESET = False
                    cte = calc_steer_cte.get_cte(self.current_position, self.final_waypoints)
                    throttle, brake, steering = self.controller.control(twist_cmd = self.current_twist_cmd,
                                                                        current_velocity = self.current_velocity,
                                                                        target_velocity = self.final_waypoints[0].twist.twist.linear.x,
                                                                        delta_time = delta_time, cte=cte)
                    # You should only publish the control commands if dbw is enabled
                    self.publish(throttle, brake, steering)
                else:
                    self.PID_RESET = True
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
