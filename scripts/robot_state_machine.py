#!/usr/bin/env python

import threading
import rospy
import actionlib

from smach import State,StateMachine
from math import sqrt
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Empty
from wpirm_navigation.srv import *

# TODO: Setup
class GPSFix(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.gps_topic = rospy.get_param('~gps_topic', default='fix')

        self.last_gps = NavSatFix()

        self.gps_sub = rospy.Subscriber(self.gps_topic, NavSatFix, self._gps_cb)
    
    def execute(self, userdata):
        while self.last_gps.status.status <= self.last_gps.status.STATUS_FIX:
            rospy.sleep(0.1)
        return 'success'

    def _gps_cb(self, data):
        self.last_gps = data

class NavToCone(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints', 'cur_waypoint_in'])
        self.odom_topic = rospy.get_param('~odom_topic', default='odom')
        self.gps_topic = rospy.get_param('~gps_topic', default='fix')
        
        self.pose = PoseWithCovarianceStamped()
        self.last_gps = NavSatFix()
        
        self.odom_sub = rospy.Subscriber(self.odom_topic, PoseWithCovarianceStamped, self._odom_cb)
        self.gps_sub = rospy.Subscriber(self.gps_topic, NavSatFix, self._gps_cb)
        
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
    
    def execute(self, userdata):
        rate = rospy.Rate(10)  # Set the rate at which new nav goals are given
        at_waypoint = False

        lat = userdata.waypoints[userdata.cur_waypoint_in][0]
        lon = userdata.waypoints[userdata.cur_waypoint_in][1]

        cone_gps_srv = rospy.ServiceProxy('cone_gps_pose', ConeGPSPose)
        goal = cone_gps_srv(ConeGPSPoseRequest(lat, lon)).cone_loc

        rospy.loginfo('Executing move_base goal to position (x,y): %s, %s' % (goal.pose.position.x, goal.pose.position.y))
        rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}".format(userdata.cur_waypoint_in))
        self.client.send_goal(goal)
        self.client.wait_for_result()
        return 'success'

        # while not at_waypoint:
        #     # Test to see if we have a gps fix or not
        #     if self.last_gps.status.status >= self.last_gps.status.STATUS_FIX:
        #         cone_gps_srv = rospy.ServiceProxy('cone_gps_pose', ConeGPSPose)
        #         goal = cone_gps_srv(ConeGPSPoseRequest(lat, lon)).cone_loc

        #         rospy.loginfo('Executing move_base goal to position (x,y): %s, %s' %
        #                 (goal.pose.position.x, goal.pose.position.y))
        #         rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}".format(userdata.cur_waypoint_in))
        #         self.client.send_goal(goal)
        #         self.client.wait_for_result()
            
        #     rate.sleep()
    
    def _odom_cb(self, data):
        self.pose = data
    
    def _gps_cb(self, data):
        self.last_gps = data
    
    def euclidean_distance(self, start_pose, goal_pose):
        """Euclidean distance between a start pose and the goal."""
        return sqrt(pow((goal_pose.pose.position.x - start_pose.pose.position.x), 2) +
                    pow((goal_pose.position.y - start_pose.pose.position.y), 2))

class TouchCone(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'fail', 'last_cone'], input_keys=['waypoints', 'cur_waypoint_in', 'last_waypoint'])
    
    def execute(self, userdata):
        pass

class FailTouch(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
    
    def execute(self, userdata):
        pass

class ResetCone(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['cur_waypoint_in'], output_keys=['cur_waypoint_out'])
    
    def execute(self, userdata):
        pass

def main():
    rospy.init_node('wpirm_state_machine')

    sm = StateMachine(outcomes=['stop'])
    sm.userdata.waypoints = rospy.get_param('~waypoints')
    sm.userdata.cur_waypoint = 0
    sm.userdata.last_waypoint = rospy.get_param('~last_waypoint', default=3)

    with sm:
        StateMachine.add('GPS_FIX', GPSFix(),
                           transitions={'success':'NAV_TO_CONE'})
        StateMachine.add('NAV_TO_CONE', NavToCone(),
                           transitions={'success':'TOUCH_CONE'},
                           remapping={'waypoints':'waypoints', 'cur_waypoint_in':'cur_waypoint'})
        StateMachine.add('TOUCH_CONE', TouchCone(),
                           transitions={'success':'RESET_CONE', 'fail':'FAIL_TOUCH', 'last_cone':'stop'},
                           remapping={'cur_waypoint_in':'cur_waypoint', 'last_waypoint':'last_waypoint'})
        StateMachine.add('FAIL_TOUCH', FailTouch(),
                           transitions={'success':'TOUCH_CONE'})
        StateMachine.add('RESET_CONE', ResetCone(),
                           transitions={'success':'NAV_TO_CONE'},
                           remapping={'cur_waypoint_in':'cur_waypoint', 'cur_waypoint_out':'cur_waypoint'})

    outcome = sm.execute()

if __name__ == '__main__':
    main()