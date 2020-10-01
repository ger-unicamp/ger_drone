#!/usr/bin/env python

# pontos iniciais ---- [5,1.8,-0.8] -- [7.5,2,2] -- [7.5,7.5,2] --- [2,7.5,2]----- [2,2,2] ---- [5,5,2]

import rospy
from std_srvs.srv import Trigger
from mrs_msgs.srv import Vec4
from mrs_msgs.srv import ReferenceStampedSrv
from geometry_msgs.msg import Point




rospy.init_node('caminho')

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
    

def voar(a):
    rospy.wait_for_service('/uav1/control_manager/reference')
    tres = rospy.ServiceProxy('/uav1/control_manager/reference',ReferenceStampedSrv)
    
    reqc = ReferenceStampedSrv._request_class()
    
    x =a[0]
    y = a[1]
    z = a[2]
    reqc.reference.position.x = x
    reqc.reference.position.y = y
    reqc.reference.position.z = z
    reqc.reference.heading = 0
    tres(reqc)







    
if __name__ == '__main__':
    try:
        
        pontos = [[2,2,2.5],[2,3.5,2.5],[7.5,3.5,2],[7.5,5.5,2],[2,5,2],[2,7.5,2],[7.5,7.5,2],[8.1,1.2,2]]
        
        decolar()

        rospy.sleep(8)




        for i in pontos:
            rospy.sleep(2)
            voar(i)
            rospy.sleep(6)



        pousar()    
        

    except rospy.ROSInternalException:
        pass    
      


