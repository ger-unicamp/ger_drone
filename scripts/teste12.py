#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger
from mrs_msgs.srv import Vec4
from mrs_msgs.srv import ReferenceStampedSrv
from geometry_msgs.msg import Point
from mrs_msgs.srv import String 
from mrs_msgs.msg import PositionCommand

rospy.init_node('caminho2')
check = False
check2 = 0
mudanca = 0

def decolar():
    print('D')
    rospy.wait_for_service('/uav1/uav_manager/takeoff')
    dois = rospy.ServiceProxy('/uav1/uav_manager/takeoff', Trigger)
    reqb = Trigger._request_class()
    dois(reqb)


def voar(a):
    rospy.wait_for_service('/uav1/control_manager/reference')
    tres = rospy.ServiceProxy('/uav1/control_manager/reference',ReferenceStampedSrv)
    
    reqc = ReferenceStampedSrv._request_class()
    
    x = a[0]
    y = a[1]
    z = a[2]
    lista = [x,y,z]
    reqc.reference.position.x = x
    reqc.reference.position.y = y
    reqc.reference.position.z = z
    reqc.reference.heading = 0
    tres(reqc)
   

def velocidade():
    rospy.wait_for_service('/uav1/constraint_manager/set_constraints')
    quatro = rospy.ServiceProxy('/uav1/constraint_manager/set_constraints',String)
    reqd = String._request_class()
    reqd.value = 'slow'
        
    quatro(reqd)

def pousar():
   print('P')
   rospy.wait_for_service('/uav1/uav_manager/land')
   um = rospy.ServiceProxy('/uav1/uav_manager/land', Trigger)
   reqa = Trigger._request_class()
   um(reqa)




if __name__ == '__main__':
    try:
        velocidade()
        decolar()
        rospy.sleep(10)
        voar([5.8,1.8,0.6])
        rospy.sleep(4)
        pontos = [4.4000, 4.4467,   4.4933,   4.5400,   4.5867,   4.6333,   4.6800,   4.7267, 4.7733,   4.8200,   4.8667,   4.9133,   4.9600 ,  5.0067,   5.0533,   5.1000,5.1467 ,  5.1933,   5.2400 ,  5.2867,   5.3333,   5.3800,   5.4267,   5.4733,5.5200 ,  5.5667 ,  5.6133 ,  5.6600,   5.7067 ,  5.7533,   5.8000]
       
        poses = []
        z = 0.6
        for i in pontos:
            y = -4.5714*i + 28.314
            
            poses.append([i,y,z])

       
        poses.reverse()
        

        for j in poses[0:30:3]:
            voar(j)
            rospy.sleep(5)
            #if tem vermelho:
                #dar sinal
            rospy.sleep(5)
                

        rospy.sleep(10)
        

    except rospy.ROSInternalException:
        pass        


            