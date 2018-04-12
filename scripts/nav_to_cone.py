#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
from copy import deepcopy
import rospy
import actionlib
import tf
from actionlib_msgs.msg import GoalID, GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class NavToCone(object):
    def __init__(self):

        self.at_goal = False
        
        # Navigation setup
        self.cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        self.move = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move.wait_for_server(rospy.Duration(1))

        self.move_base_status = rospy.Subscriber('/move_base/status', GoalStatusArray, self._update_status)

        rospy.spin()

    def end_nav(self):
		# end the navigation
		c = GoalID()
		self.cancel.publish(c)
    
    def _update_status(self, data):
		# update the goal status
		for goal in data.status_list:
			if goal.status < 2:
				self.last_goal = goal
				self.at_goal = False
		if len(data.status_list) > 0 and not self.at_goal:
			for goal in data.status_list:
				# get next goal, at current goal
				if goal.goal_id.id == self.last_goal.goal_id.id:
					if 2 <= goal.status <= 3:
						self.at_goal = True
						self.nav_next_centroid()
					elif 4 <= goal.status <= 5:  # Goal unreachable or rejected
						self.at_goal = True
						self.unreachable = True
						self.end_nav()
						self.nav_next_centroid()

if __name__ == '__main__':
    rospy.init_node('nav_to_cone')

    try:
        cp = NavToCone()
    except rospy.ROSInterruptException:
        pass
