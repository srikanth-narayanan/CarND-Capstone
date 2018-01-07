#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import math
import tf

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        # Initialise basic variables
        self.current_position = None
        self.waypoints = None

        rospy.spin()

    def pose_cb(self, msg):
        '''
        Call Back Function when Current Positions is received
        '''
        # TODO: Implement
        self.current_position = msg.pose # Get Message
        if self.waypoints is not None:
            self.publish_waypoints() # Push New Calculated Waypoints ahead of the vehicle

    def waypoints_cb(self, waypoints):
        '''
        Call Back Function when waypoints are received. This function is called
        only once as the entire waypoints are sent
        '''
        # TODO: Implement
        if self.waypoints is None:
            self.waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def _calc_dist(self, pos1, pos2):
        '''
        A helper method to return distance between two waypoints
        '''
        dist = math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2)
        return dist

    def get_closet_waypoint(self):
        '''
        This method determines the closet waypoint from the received current
        position to the waypoint list.
        '''
        closest_distance = 999999 # Initialise very high value
        closest_waypoint_idx = 0
        for idx, waypoint in enumerate(self.waypoints):
            temp_dist = self._calc_dist(self.current_position.position, waypoint.pose.pose.position)
            if (temp_dist < closest_distance):
                closest_distance = temp_dist
                closet_waypoint_idx = idx

        # Calculate Heading
        pos_map_x = self.waypoints[closet_waypoint_idx].pose.pose.position.x
        pos_map_y = self.waypoints[closet_waypoint_idx].pose.pose.position.y

        map_heading = math.atan2((pos_map_y - self.current_position.position.y), (pos_map_x - self.current_position.position.x))
        # Get Yaw angle by transforming cartesian co-ordinates to pitch, roll and yaw
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((self.current_position.orientation.x,
                                                                     self.current_position.orientation.y,
                                                                     self.current_position.orientation.z,
                                                                     self.current_position.orientation.w))
        delta_angle = abs(yaw - map_heading)
        if delta_angle > (math.pi / 4):
            closet_waypoint_idx += 1 # move it to the next point
        
        return closet_waypoint_idx

    def publish_waypoints(self):
        '''
        a main method where every thing is dealt. here after all calcs waypoints
        are pushed to publish
        '''
        if self.current_position is not None:
            closet_waypoint_idx = self.get_closet_waypoint()
            forward_waypoints = self.waypoints[closet_waypoint_idx : closet_waypoint_idx + LOOKAHEAD_WPS]

            # Create a data type to publish forward lane points
            # creating a same way to publish waypoint as done in Waypoint Loader file
            drive_path = Lane()
            drive_path.header.frame_id = '/world' # set the header name for waypoints
            drive_path.header.stamp = rospy.Time(0) # Time Stamp for the published message
            drive_path.waypoints = forward_waypoints
            # Publish waypoints
            self.final_waypoints_pub.publish(drive_path)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
