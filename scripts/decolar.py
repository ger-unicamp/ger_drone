#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger


rospy.wait_for_service('/uav1/uav_manager/takeoff')
dois = rospy.ServiceProxy('/uav1/uav_manager/takeoff', Trigger)
reqb = Trigger._request_class()
dois(reqb)