#!/usr/bin/env python

from math import sqrt, sin, cos
import numpy as np
import tf


'''
This module contains helper functions to calculate the cross track error for
steering angle from the waypoins
'''

def transform_waypoints(current_position, waypoints, num_points=None):
    '''
    the waypoints are transformed to match the position and orientation of the
    object to calculate x and y cordinates
    '''
    x_values = []
    y_values = []

    roll, pitch, yaw = tf.transformations.euler_from_quaternion((current_position.orientation.x,
                                                                 current_position.orientation.y,
                                                                 current_position.orientation.z,
                                                                 current_position.orientation.w))
    if num_points is None:
        num_points = len(waypoints)
    for i in range(num_points):
        shift_x = waypoints[i].pose.pose.position.x - current_position.position.x
        shift_y = waypoints[i].pose.pose.position.y - current_position.position.y

        x = shift_x * cos(yaw) - shift_y * sin(yaw)
        y = shift_x * sin(yaw) + shift_y * cos(yaw)

        x_values.append(x)
        y_values.append(y)

    return x_values, y_values


def get_cte(current_position, waypoints):
    '''
    Return the CTE from the current position to the desired waypoint collection
    '''
    x_values, y_values = transform_waypoints(current_position, waypoints, num_points=20)
    poly_coeff = np.polynomial.polynomial.polyfit(x_values, y_values, 3)
    cte = poly_coeff[0] # kind of center

    return cte
