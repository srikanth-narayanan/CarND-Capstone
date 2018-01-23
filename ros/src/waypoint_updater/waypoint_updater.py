#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
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
MIN_TARGET_V_MPH = 3 # minimum speed for car to approach a red light while coasting (MPH)
ACCELERATION_MPHS = 3 # Rate of change in speed when accelerating (MPH per second)
DECELERATION_MPHS = 5 # Rate of change in speed when decelerating (MPH per second)
STOP_DIST_TRAFFIC_LIGHT = 6 # distance in front of stop line center of car should stop


def kmph2mps(velocity_kmph):
    "Km per hour to meters per second"
    return (velocity_kmph * 1000.) / (60. * 60.)

def mph2mps(velocity_mph):
    "Miles per hour to meters per second"
    return velocity_mph * 1.60934 / 3.6

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.target_velocity_mps = kmph2mps(rospy.get_param('/waypoint_loader/velocity'))

        # TODO: Add other member variables you need below

        # Initialise basic variables
        self.current_position = None
        self.waypoints = None
        self.red_traffic_light = None
        self.last_speed = 0
        self.last_timestamp = None
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_waypoints()
            rate.sleep()

    def pose_cb(self, msg):
        '''
        Call Back Function when Current Positions is received
        '''
        # TODO: Implement
        self.current_position = msg.pose # Get Message
        #if self.waypoints is not None:
            #self.publish_waypoints() # Push New Calculated Waypoints ahead of the vehicle

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
        self.red_traffic_light = msg.data

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
        # If we don't the position and traffic_waypoint channels are not yet broadcasting, the car should not move
        if self.current_position is not None and self.red_traffic_light is not None:
            if self.red_traffic_light and self.red_traffic_light > 0 and self.waypoints:
                traffic_light_pose = self.waypoints[self.red_traffic_light].pose.pose
                dist_to_traffic_light = self._calc_dist(self.current_position.position, traffic_light_pose.position)
            else:
                dist_to_traffic_light = 999999

            closet_waypoint_idx = self.get_closet_waypoint()
            forward_waypoints = self.waypoints[closet_waypoint_idx : closet_waypoint_idx + LOOKAHEAD_WPS]

            # Set speed for waypoints
            acceleration_mps2 = mph2mps(ACCELERATION_MPHS)
            deceleration_mps2 = mph2mps(DECELERATION_MPHS)
            min_speed_mps = mph2mps(MIN_TARGET_V_MPH)
            elapsed_time_s = (rospy.get_time() - self.last_timestamp) if self.last_timestamp else 1
            change_in_v_during_acc = acceleration_mps2 * elapsed_time_s
            change_in_v_during_dec = deceleration_mps2 * elapsed_time_s

            # multiplied by 1.1 as a buffer to get a slightly longer breaking distance than what is physically needed
            breaking_dist_m = self.last_speed * self.last_speed / deceleration_mps2 * 1.1

            # Note that the way speed is increased or decreased here is too simplistic since it does not take into
            # account the actual time duration since last update, and the acceleration/deceleration is therefore
            # dependent on the rospy.Rate
            # SDG TODO: ensure change in speed is constant and works with different ropsy.Rate values
            for i in range(len(forward_waypoints) - 1):
                if dist_to_traffic_light > (STOP_DIST_TRAFFIC_LIGHT + breaking_dist_m):
                    # If the car is very far from the traffic light, there is no need to start slowing down
                    # but car may have to accelerate, so increase speed since last speed until target speed is reached
                    forward_waypoints[i].twist.twist.linear.x = min(self.last_speed + change_in_v_during_acc,
                                                                    self.target_velocity_mps)
                elif (dist_to_traffic_light > STOP_DIST_TRAFFIC_LIGHT):
                    # If the car is within 75m of a traffic light, it should start slowing down
                    forward_waypoints[i].twist.twist.linear.x = max(self.last_speed - change_in_v_during_dec,
                                                                    min_speed_mps)
                else:
                    # car should stop 30 m (i.e., STOP_DIST_TRAFFIC_LIGHT) before traffic light
                    forward_waypoints[i].twist.twist.linear.x = 0

            self.last_speed = forward_waypoints[0].twist.twist.linear.x

            # Create a data type to publish forward lane points
            # creating a same way to publish waypoint as done in Waypoint Loader file
            drive_path = Lane()
            drive_path.header.frame_id = '/world' # set the header name for waypoints
            drive_path.header.stamp = rospy.Time(0) # Time Stamp for the published message
            self.last_timestamp = rospy.get_time()
            drive_path.waypoints = forward_waypoints
            # Publish waypoints
            self.final_waypoints_pub.publish(drive_path)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
