#!/usr/bin/env python2

# TO DO
# identificador de bases?
# objetos diferentes?
# como funciona subscriber + service?



import rospy

import numpy as np
#from scipy.spatial.transform import Rotation as R
#from matplotlib import pyplot as plt

from mrs_msgs.msg import UavState
from ger_drone.msg import Object, Identifier

from ger_drone.srv import GetObject, GetObjectResponse



def entregaListaObjetos(req):

    lista = []

    obj = Object()

    obj.pose.position.x = 3
    obj.pose.position.y = 3
    obj.pose.position.z = 2.5

    lista.append(obj) 

    obj = Object()

    obj.pose.position.x = 5
    obj.pose.position.y = 1.8
    obj.pose.position.z = 2.5

    lista.append(obj)

    obj = Object()

    obj.pose.position.x = 2
    obj.pose.position.y = 2
    obj.pose.position.z = 2.5

    lista.append(obj)

    obj = Object()

    obj.pose.position.x = 4
    obj.pose.position.y = 4
    obj.pose.position.z = 2.5

    lista.append(obj)

    for obj in lista:
        obj.identifier.state.data = Identifier.STATE_NOPROCESSADO
        obj.identifier.type.data = Identifier.TYPE_BASE

    return GetObjectResponse(lista)


if __name__ == '__main__':
    try:

       
        rospy.init_node('mapa', anonymous="True")
        
        rospy.Service('get_object', GetObject, entregaListaObjetos)

        pub = rospy.Publisher('info',Identifier,queue_size=10)


        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():

            rate.sleep()

    except rospy.ROSInternalException:
        pass