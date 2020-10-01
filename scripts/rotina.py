#!/usr/bin/env python

# pontos iniciais ---- [5,1.8,-0.8] -- [7.5,2,2] -- [7.5,7.5,2] --- [2,7.5,2]----- [2,2,2] ---- [5,5,2]

import rospy
from std_srvs.srv import Trigger
from mrs_msgs.srv import Vec4

rospy.init_node('procedimento')

def pousar():
   print('P')
   rospy.wait_for_service('/uav1/uav_manager/land')
   um = rospy.ServiceProxy('/uav1/uav_manager/land', Trigger)
   reqa = Trigger._request_class()
   um(reqa)
   

def decolar():
    print('D')
    rospy.wait_for_service('/uav1/uav_manager/takeoff')
    dois = rospy.ServiceProxy('/uav1/uav_manager/takeoff', Trigger)
    reqb = Trigger._request_class()
    dois(reqb)

def rotina():
    pousar()
    rospy.sleep(12)
    decolar()
    rospy.sleep(12)
    

    
if __name__ == '__main__':
    try:
        
        rotina()
        print('indo')

        rospy.wait_for_service('/uav1/control_manager/goto')

        req = Vec4._request_class()

        srv = rospy.ServiceProxy('/uav1/control_manager/goto', Vec4)

        req.goal[0] = 4
        req.goal[1] = 4
        req.goal[2] = 2
        req.goal[3] = 0

        srv(req)
        rospy.sleep(12)
        rotina()
        
        rospy.sleep(12)
        
        print('indo')
        req.goal[0] = 5
        req.goal[1] = 1.8
        req.goal[2] = 2
        req.goal[3] = 0
        srv(req)

        rospy.sleep(12)
        pousar()

    except rospy.ROSInternalException:
        pass    
      


