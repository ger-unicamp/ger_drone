#!/usr/bin/env python
import rospy
from mrs_msgs.srv import ReferenceStampedSrv
from mrs_msgs.msg import PositionCommand
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger

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


if __name__ == '__main__':
    try:
        rospy.init_node('voarr')

        rospy.wait_for_service('/uav1/control_manager/reference')

       

        req = ReferenceStampedSrv._request_class()
        srv = rospy.ServiceProxy('/uav1/control_manager/reference', ReferenceStampedSrv)
        
        req.reference.position.x = 8.1
        req.reference.position.y = 1.5
        req.reference.position.z =  2
        
        srv(req)
        
        
        
        
        
        
        '''
        req.reference.position.x = 3.8
        req.reference.position.y = 1.8
        req.reference.position.z =  2
        
        srv(req)

        
        rospy.sleep(8)
        pousar()
        rospy.sleep(6)
        decolar()
        
        rospy.sleep(6)

        req.reference.position.x = 5.1
        req.reference.position.y =  1.8
        req.reference.position.z = 2.5
        srv(req)
        rospy.sleep(8)

        pousar()
        '''





    except rospy.ROSInternalException:
        pass    
      

     # 8.1 e 1.5