#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
import rospy
import tf

def haversineDistance(latCur, lonCur, latWP, lonWP): #Returns distance to waypoint in Metres
	latWP, lonWP, latCur, lonCur = map(radians, [latWP, lonWP, latCur, lonCur]) #Convert into Radians to perform math
	a = pow(sin((latWP - latCur)/2),2) + cos(latCur) * cos(latWP) * pow(sin((lonWP - lonCur)/2),2)
	return earthRadius * 2.0 * asin(sqrt(a))  #Return calculated distance to waypoint in Metres
	
def bearing(latCur, lonCur, latWP, lonWP): #Bearing to waypoint (degrees)
	latWP, lonWP, latCur, lonCur = map(radians, [latWP, lonWP, latCur, lonCur]) #Convert into Radians to perform math
	dLon = lonWP - lonCur
	return atan2(sin(dLon) * cos(latWP), cos(latCur) * sin(latWP) - (sin(latCur) * cos(latWP) * cos(dLon)))