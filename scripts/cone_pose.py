#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
import rospy
import tf
from vision_msgs.msg import *

class ConeTfBroadcaster(object):
    def __init__(self):
        self.camera_frame = rospy.get_param('~camera_frame')
        self.detection_topic = rospy.get_param('~detection_topic', default='yolo_predict/detection')
        self.cone_coords = rospy.get_param('~cone_coords')

        self.detection_sub = rospy.Subscriber(self.detection_topic, Detection2DArray, self._detection_cb)

        rospy.spin()

    def _detection_cb(self, data):
        pass

    def _handle_transform(self, camera_frame, x_cord, y_cord):
        t = tf.Transformer()
        w2c_pos, w2c_quat = t.lookupTransform(camera_frame, 'map', t.getLatestCommonTime('map', camera_frame))

        br = tf.TransformBroadcaster()
        br.sendTransform((x_cord + w2c_pos[0], y_cord + w2c_pos[1], 0),
                    w2c_quat,
                    rospy.Time.now(),
                    'cone_loc',
                    'map')
    
    def lowpass_filter(self, cone_detected, cone_gps):
        pass

if __name__ == '__main__':
    rospy.init_node('cone_tf_broadcaster')

    try:
        cp = ConeTfBroadcaster()
    except rospy.ROSInterruptException:
        pass
