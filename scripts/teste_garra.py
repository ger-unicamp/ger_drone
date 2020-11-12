#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger
from mrs_msgs.srv import Vec4
from mrs_msgs.srv import ReferenceStampedSrv
from geometry_msgs.msg import Point
from mrs_msgs.srv import String 
from mrs_msgs.msg import PositionCommand
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
rospy.init_node('agarrando')



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

def ativa_garra():
    rospy.wait_for_service('/link_attacher_node/attach')
    garra = rospy.ServiceProxy('/link_attacher_node/attach',Attach)

    req = AttachRequest()
    req.model_name_1 = "uav1"
    req.link_name_1 = "base_link"
    req.model_name_2 = "equipmentA"
    req.link_name_2 = "link_A"

    garra(req)                               

def desativa_garra():
    rospy.wait_for_service('/link_attacher_node/detach')
    garra = rospy.ServiceProxy('/link_attacher_node/detach',Attach)

    req = AttachRequest()
    req.model_name_1 = "uav1"
    req.link_name_1 = "base_link"
    req.model_name_2 = "equipmentA"
    req.link_name_2 = "link_A"

    garra(req)

#[2.65,0,0.8]
#[3,-1.8375]

if __name__ == '__main__':
    try:
    
        p = [4.23,-2.03,2]
        voar(p)
        rospy.sleep(7)
        q = [4.23,-2.03,0.6]
        voar(q)
        rospy.sleep(7)
        ativa_garra()
        rospy.sleep(4)

        base = [0,0,2]
        voar(base)
        rospy.sleep(6)


        p = [4.23,-2.03,2]
        voar(p)
        rospy.sleep(7)
        q = [4.23,-2.03,0.8]
        voar(q)
        rospy.sleep(7)
        desativa_garra()




        

        

    except rospy.ROSInternalException:
        pass        