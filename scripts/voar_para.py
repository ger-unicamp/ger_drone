#!/usr/bin/env python
#chama de volta pra base
import rospy
from mrs_msgs.srv import Vec4
from std_srvs.srv import Trigger



if __name__ == '__main__':
    try:
        rospy.init_node('ir_para')

        rospy.wait_for_service('/uav1/control_manager/goto')

        req = Vec4._request_class()

        srv = rospy.ServiceProxy('/uav1/control_manager/goto', Vec4)
## pontas [4.4,8.2] [5.8,1.8]        ponto de partida:[8.1,1.8,1.5]
        req.goal[0] = 0 #(2.65 + 1.2)
        req.goal[1] = 0 #-6.3
        req.goal[2] = 1.5
        req.goal[3] = 0
        srv(req)
        rospy.sleep(10)
        

        '''
        rospy.wait_for_service('/uav1/uav_manager/land')
        oi = rospy.ServiceProxy('/uav1/uav_manager/land', Trigger)
        reqq = Trigger._request_class()
        oi(reqq)
        '''

    except rospy.ROSInternalException:
        pass    
     # y = -0.22x +3,1

    
