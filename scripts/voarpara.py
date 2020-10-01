#!/usr/bin/env python
#manda pra uma posição marcada
import rospy
from mrs_msgs.srv import Vec4



if __name__ == '__main__':
    try:
        rospy.init_node('ir_para')

        rospy.wait_for_service('/uav1/control_manager/goto')

        req = Vec4._request_class()

        srv = rospy.ServiceProxy('/uav1/control_manager/goto', Vec4)

        req.goal[0] = 5
        req.goal[1] = 5
        req.goal[2] =2
        req.goal[3] = 0
        srv(req)
        rospy.sleep(5)
        
        

    except rospy.ROSInternalException:
        pass    
      