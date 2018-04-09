#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
import rospy
import tf
from vision_msgs.msg import *

class ConePose(object):
    def __init__(self):
        self.camera_frame = rospy.get_param('~camera_frame')
        self.detection_topic = rospy.get_param('~detection_topic', default='yolo_predict/detection')

        self.detection_sub = rospy.Subscriber(self.detection_topic, Detection2DArray, self._detection_cb)

        rospy.spin()

    def _detection_cb(self, data):
        pass

    def _handle_transform(self, camera_frame, x_cord, y_cord):
        br = tf.TransformBroadcaster()
        br.sendTransform((x_cord, y_cord, 0),
                    tf.transformations.quaternion_from_euler(0, 0, 0),
                    rospy.Time.now(),
                    'cone_loc',
                    camera_frame)

if __name__ == '__main__':
    rospy.init_node('cone_tf_broadcaster')

    try:
        cp = ConePose()
    except rospy.ROSInterruptException:
        pass
