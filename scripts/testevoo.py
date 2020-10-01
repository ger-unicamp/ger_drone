#!/usr/bin/env python

import rospy
from mrs_msgs.srv import ReferenceSrv
from geometry_msgs.msg import Point


if __name__ == '__main__':
    try:
        rospy.init_node('voar')

        rospy.wait_for_service('/uav1/control_manager/reference')

       

        req = ReferenceSrv._request_class()
        srv = rospy.ServiceProxy('/uav1/control_manager/reference',ReferenceSrv)
        
        req.reference.position.x = 5.0
        req.reference.position.y = 5.0
        req.reference.position.z = 2.0
        
     
        srv(req)


    except rospy.ROSInternalException:
        pass    
      