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

rospy.init_node('publisher')
pubBase = rospy.Publisher('objeto_detectado',Object, queue_size=10)

def getListObject():
    rospy.wait_for_service('get_object')
    cinco = rospy.ServiceProxy('get_object', GetObject)
    reqc = GetObject._request_class()

    reqc.identifier.state.data = Identifier.STATE_NOPROCESSADO
    reqc.identifier.type.data = Identifier.TYPE_BASE

    response = cinco(reqc)
    lista =  response.list
    return lista


while not rospy.is_shutdown():
    lista = getListObject()
    for i in lista:
        print(i)
        rospy.sleep(0.5)

