#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
from copy import deepcopy
import rospy
import actionlib
import tf
from math import pi, sin, cos, radians
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry


class ConeVisionPose(object):
    def __init__(self):
        self.camera_frame = rospy.get_param('~camera_frame')
        self.detection_topic = rospy.get_param('~detection_topic', default='yolo_predict/detection')
        self.odom_topic = rospy.get_param('~odom_topic', default='odom')
        self.horiz_fov = rospy.get_param('~horiz_fov', default=96.0)

        self.image_width = 1
        
        self.last_detection = Detection2DArray()
        self.last_odom = Odometry()
        self.bridge = CvBridge()
        
        self.detection_sub = rospy.Subscriber(self.detection_topic, Detection2DArray, self._detection_cb)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb)

        rate = rospy.Rate(10)
        last_detection = Detection2DArray()
        while not rospy.is_shutdown():
            cur_detection = self.last_detection
            if cur_detection.header.stamp != last_detection.header.stamp:
                x, y = self.get_cone_pose(cur_detection)
                self._handle_transform(self.camera_frame, x, y)
            last_detection = cur_detection
            rate.sleep()

    def _detection_cb(self, data):
        self.last_detection = data
        if self.image_width == None:
            self.image_width = data.detections[0].source_img.width

    def _odom_cb(self, data):
        self.last_odom = data

    def _handle_transform(self, camera_frame, x_cord, y_cord):
        br = tf.TransformBroadcaster()
        br.sendTransform((x_cord, y_cord, 0),
                    quaternion_from_euler(0,0,0),
                    rospy.Time.now(),
                    'cone_vis_loc',
                    camera_frame)
    
    def get_cone_pose(self, detection):
        cone_poses = []

        cone_x = None
        cone_y = None

        for i, detect in enumerate(detection.detections):
            depth_image = detect.source_img
            
            x_center = int(detect.bbox.center.x)
            y_center = int(detect.bbox.center.y)
            try:
                cv_depth_image = self.bridge.imgmsg_to_cv2(depth_image, "32FC1")
                img_height, img_width = cv_depth_image.shape
                center_pixel_depth = cv_depth_image[y_center, x_center]
                # dist_avg = self.depth_region(cv_depth_image, detect)
                distance = float(center_pixel_depth)
                
                bearing, object_range = self.calculate_bearing(img_width, x_center, distance)

                cone_y = cos(radians(bearing)) * center_pixel_depth
                cone_x = sin(radians(bearing)) * center_pixel_depth
                
                # rospy.loginfo('Bearing: {} Depth: {}'.format(bearing, center_pixel_depth))
            except CvBridgeError as e:
                rospy.logerr(e)

            return cone_x, cone_y
            
    
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

    def calculate_bearing(self, img_width, object_x, object_depth):
        # only consider horizontal FOV.
        # Bearing is only in 2D

        # Calculate Horizontal Resolution
        horiz_res = self.horiz_fov / img_width

        # location of object in pixels.
        # Measured from center of image.
        # Positive x is to the right, positive y is upwards
        obj_x = ((img_width / 2.0) - object_x) * -1
        # rospy.loginfo('Object X: {}'.format(img_width))

        # Calculate angle of object in relation to center of image
        bearing = obj_x*horiz_res        # degrees
        # bearing = bearing*pi/180.0  # radians

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
