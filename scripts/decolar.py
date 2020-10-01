#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger


t = 0

rospy.init_node('decolar_cliente')

rospy.wait_for_service('/uav1/uav_manager/takeoff')
  
val = rospy.ServiceProxy('/uav1/uav_manager/takeoff', Trigger)
req = Trigger._request_class()
disp = val(req)
   
while t < 10:   
   disp
   t = t + 1
   print('pt1')
   rospy.sleep(1)


if t >= 9:
    print('pt2')
    rospy.wait_for_service('/uav1/uav_manager/land')

    vel = rospy.ServiceProxy('/uav1/uav_manager/land', Trigger)
    reqq = Trigger._request_class()
    dispo= vel(reqq)

