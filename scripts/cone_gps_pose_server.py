#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
from copy import deepcopy
import rospy
import actionlib
import tf
from math import radians, cos, sin, asin, sqrt, pow, pi, atan2
from actionlib_msgs.msg import GoalID, GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# TODO: Implement a particle filter for pose

class ConeGPSPoseServer(object):
    def __init__(self):
        self.gps_topic = rospy.get_param('~gps_topic', default='fix')
        self.odom_topic = rospy.get_param('~odom_topic', default='odom')
        self.cone_coords = rospy.get_param('~cone_coords')

        self.last_gps = NavSatFix()
        self.last_odom = PoseWithCovarianceStamped()
        self.earth_radius = 6371000.0  # Metres
        
        self.gps_sub = rospy.Subscriber(self.gps_topic, NavSatFix, self._gps_cb)
        self.odom_sub = rospy.Subscriber(self.odom_topic, PoseWithCovarianceStamped, self._odom_cb)

        s = rospy.Service('cone_gps_pose', ConeGPSPose, self._handle_cone_pose, buff_size=1)
        s.spin()

    def _gps_cb(self, data):
        self.last_gps = deepcopy(data)

    def _odom_cb(self, data):
        self.last_odom = deepcopy(data)

    def _handle_transform(self, cone, x_cord, y_cord):
        t = tf.Transformer()
        w2c_pos, w2c_quat = t.lookupTransform(camera_frame, 'map', t.getLatestCommonTime('map', camera_frame))

        br = tf.TransformBroadcaster()
        br.sendTransform((x_cord + w2c_pos[0], y_cord + w2c_pos[1], 0),
                    w2c_quat,
                    rospy.Time.now(),
                    'cone_loc',
                    'map')

    def _handle_cone_pose(self, req):
        cone_pose_stamped = PoseStamped()

        cone_pose_stamped.header = Header()
        cone_pose_stamped.header.stamp = rospy.get_rostime()

        cone_pose_stamped.pose = self.get_cone_pose(req.cone_lat, req.cone_lon)

        return ConeGPSPoseResponse(cone_pose_stamped)
    
    def get_cone_pose(self, lat_c, lon_c):
        # Store the positions locally so they don't change when using them
        cur_gps = self.last_gps
        cur_odom = self.last_odom

        lat_cur = cur_gps.latitude
        lon_cur = cur_gps.longitude

        pos_x = cur_odom.pose.pose.position.x
        pos_y = cur_odom.pose.pose.position.y

        dist_to_cone = self.haversine_distance(lat_cur, lon_cur, lat_c, lon_c)
        bearing_to_cone = self.bearing(lat_cur, lon_cur, lat_c, lon_c)

        cone_pose = Pose()
        
        cone_pose.position.x = pos_x + (dist_to_cone * cos(bearing_to_cone))  # Convert distance and angle to waypoint from Polar to Cartesian co-ordinates then add current position of robot odometry 
        cone_pose.position.y = pos_y + (dist_to_cone * sin(bearing_to_cone))
        cone_pose.position.z = 0

        cone_pose.orientation = quaternion_from_euler(0.0, 0.0, 0.0)

        return cone_pose
    
    def haversine_distance(self, lat_cur, lon_cur, lat_c, lon_c): #Returns distance to waypoint in Metres
	    latWP, lonWP, latCur, lonCur = map(radians, [lat_c, lon_c, lat_cur, lon_cur]) #Convert into Radians to perform math
	    a = pow(sin((latWP - latCur)/2),2) + cos(latCur) * cos(latWP) * pow(sin((lonWP - lonCur)/2),2)
	    return self.earth_radius * 2.0 * asin(sqrt(a))  #Return calculated distance to waypoint in Metres
	
    def bearing(self, lat_cur, lon_cur, lat_c, lon_c): #Bearing to waypoint (degrees)
        latWP, lonWP, latCur, lonCur = map(radians, [lat_c, lon_c, lat_cur, lon_cur]) #Convert into Radians to perform math
        dLon = lonWP - lonCur
        return atan2(sin(dLon) * cos(latWP), cos(latCur) * sin(latWP) - (sin(latCur) * cos(latWP) * cos(dLon)))

if __name__ == '__main__':
    rospy.init_node('cone_gps_pose_server')

    try:
        cp = ConeGPSPoseServer()
    except rospy.ROSInterruptException:
        pass
