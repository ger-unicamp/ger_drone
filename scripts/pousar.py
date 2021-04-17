#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger

"""!
    Chama o serviço para efetuar o pouso
"""
rospy.init_node('pouso')
rospy.wait_for_service('/uav1/uav_manager/land')
oi = rospy.ServiceProxy('/uav1/uav_manager/land', Trigger)
req = Trigger._request_class()
oi(req)

while not rospy.is_shutdown:
    oi()
    rospy.loginfo('tentando pousar')
    rospy.sleep(1)
    
     