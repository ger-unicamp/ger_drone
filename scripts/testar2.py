#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger
from mrs_msgs.srv import Vec4
from mrs_msgs.srv import ReferenceStampedSrv
from geometry_msgs.msg import Point
from mrs_msgs.srv import String 
from mrs_msgs.msg import PositionCommand


from ger_drone.msg import Object, Identifier

from ger_drone.srv import GetObject, GetObjectResponse

def publicabase(msg):
    #msg = Object()
    
   
    pubBase.publish(msg)



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
    rospy.sleep(8)
    decolar()
    rospy.sleep(8)
    

def getListObject():
    rospy.wait_for_service('get_object')
    cinco = rospy.ServiceProxy('get_object', GetObject)
    reqc = GetObject._request_class()

    reqc.identifier.state.data = Identifier.STATE_NOPROCESSADO
    reqc.identifier.type.data = Identifier.TYPE_BASE

    response = cinco(reqc)
    lista =  response.list
    print('oi')
    getPose(lista)

def getPose(lista):
    listaPoses = []
    for i in lista:
        a = [i.pose.position.x, i.pose.position.y, i.pose.position.z]
        listaPoses.append(a)
        #print(i.identifier.state.data)
        i.identifier.state.data = 1
        #print(i.identifier.state.data)
        publicabase(i)    

  

if __name__ == '__main__':
    try:  
        
        rospy.init_node('testar2')
        pubBase = rospy.Publisher('objeto_detectado',Object, queue_size=10)
       
        getListObject()

    except rospy.ROSInternalException:
        pass