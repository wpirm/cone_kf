#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
from copy import deepcopy
import rospy
import actionlib
import tf
from math import pi, sin, cos
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2DArray
from actionlib_msgs.msg import GoalID, GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class ConeVisionPose(object):
    def __init__(self):
        self.camera_frame = rospy.get_param('~camera_frame')
        self.detection_topic = rospy.get_param('~detection_topic', default='yolo_predict/detection')
        self.odom_topic = rospy.get_param('~odom_topic', default='odom')
        self.horiz_fov = rospy.get_param('~horiz_fov', default=96.0)

        self.image_width = None
        
        self.last_detection = Detection2DArray()
        self.last_odom = PoseWithCovarianceStamped()
        self.bridge = CvBridge()
        
        self.detection_sub = rospy.Subscriber(self.detection_topic, NavSatFix, self._detection_cb)
        self.odom_sub = rospy.Subscriber(self.odom_topic, PoseWithCovarianceStamped, self._odom_cb)

        rate = rospy.Rate(10)
        last_detection = Detection2DArray()
        while not rospy.is_shutdown():
            cur_detection = self.last_detection
            if cur_detection.header.stamp != last_detection.stamp:
                self.get_cone_pose()
            last_detection = cur_detection
            rate.sleep()

    def _detection_cb(self, data):
        self.last_detection = deepcopy(data)
        if self.image_width == None:
            self.image_width = data.detections[0].source_img.width

    def _odom_cb(self, data):
        self.last_odom = deepcopy(data)

    def _handle_transform(self, camera_frame, x_cord, y_cord):
        t = tf.Transformer()
        w2c_pos, w2c_quat = t.lookupTransform(camera_frame, 'map', t.getLatestCommonTime('map', camera_frame))

        br = tf.TransformBroadcaster()
        br.sendTransform((x_cord + w2c_pos[0], y_cord + w2c_pos[1], 0),
                    w2c_quat,
                    rospy.Time.now(),
                    'cone_vis_loc',
                    'map')
    
    def get_cone_pose(self, detection):
        cone_poses = []

        for i, detect in enumerate(detection.detections):
            depth_image = detect.source_img
            
            x_center = detect.bbox.center.x
            y_center = detect.bbox.center.y

            try:
                cv_depth_image = self.bridge.imgmsg_to_cv2(depth_image, "16UC1")
            except CvBridgeError as e:
                rospy.logerr(e)

            center_pixel_depth = cv_depth_image[x_center, y_center]
            # dist_avg = self.depth_region(cv_depth_image, detect)
            distance = float(center_pixel_depth)
            
            bearing = self.calculate_bearing(x_center, distance)

            cone_x = cos(bearing) * distance
            cone_y = sin(bearing) * distance

            rospy.loginfo('Cone X: {} Cone Y: {}'.format(cone_x, cone_y))
    
    def depth_region(self, depth_map, detection):
        # grab depths along a strip and take average
        # go half way
        box = detection.bbox

        y_center = box.center.y
        x_center = box.center.x
        w = box.size_x
        h = box.size_y

        starting_width = w/4
        end_width = w - starting_width
        x_center = x_center - starting_width
        pixie_avg = 0.0

        for i in range(starting_width, end_width):
            assert (depth_map.shape[1] > end_width)
            assert (depth_map.shape[1] > x_center)
            pixel_depth = depth_map[y_center, x_center]
            pixie_avg += pixel_depth
            x_center += 1

        pixie_avg = (pixie_avg/(end_width - starting_width))
        return float(pixie_avg)

    def calculate_bearing(self, object_x, object_depth):
        # only consider horizontal FOV.
        # Bearing is only in 2D

        # Calculate Horizontal Resolution
        horiz_res = self.horiz_fov/self.image_width

        # location of object in pixels.
        # Measured from center of image.
        # Positive x is to the left, positive y is upwards
        obj_x = self.image_width/2.0 - object_x

        # Calculate angle of object in relation to center of image
        bearing = obj_x*horiz_res        # degrees
        bearing = bearing*pi/180.0  # radians

        # Calculate true range, using measured bearing value.
        # Defined as depth divided by cosine of bearing angle
        if np.cos(bearing) != 0.0:
            object_range = object_depth/np.cos(bearing)
        else:
            object_range = object_depth

        return bearing, object_range

if __name__ == '__main__':
    rospy.init_node('cone_vision_poser')

    try:
        cp = ConeVisionPose()
    except rospy.ROSInterruptException:
        pass
