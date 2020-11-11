#!/usr/bin/env python

import rospy
from mrs_msgs.srv import String 
from mrs_msgs.msg import PositionCommand
from ger_drone.msg import Identifier
from ger_drone.srv import GetObject
from geometry_msgs.msg import Point
from mrs_msgs.srv import ReferenceStampedSrv
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from std_srvs.srv import SetBool

rospy.init_node('fase4')
pkg = False
check = False

def ReqPontos():
    rospy.wait_for_service('get_object')
    a = rospy.ServiceProxy('get_object', GetObject)
    requ = GetObject._request_class()
    
    requ.identifier.state.data = Identifier.STATE_NOPROCESSADO
    requ.identifier.type.data = Identifier.TYPE_BASE

    response = a(requ)
    lista = responde.lista

    rospy.wait_for_service('/ger_drone/set_atualiza_mapa')
    proxy = rospy.ServiceProxy('/ger_drone/set_atualiza_mapa', SetBool)
    req = SetBool._request_class()
    
    req.data = True

    proxy(req)

    return(lista)

def voar(a):
    print(a)
    rospy.wait_for_service('/uav1/control_manager/reference')
    tres = rospy.ServiceProxy('/uav1/control_manager/reference',ReferenceStampedSrv)
    
    reqc = ReferenceStampedSrv._request_class()
    
    x = a[0]
    y = a[1]
    z = 3
    lista = [x,y,z]
    reqc.reference.position.x = x
    reqc.reference.position.y = y
    reqc.reference.position.z = z
    reqc.reference.heading = 0
    tres(reqc)
   
    checar(lista)

def checar(a):
    global check 
    while check == False:
        pt = rospy.wait_for_message('/uav1/control_manager/position_cmd',PositionCommand)
        compara(pt,a)
        #print(check)
    check = False    

def compara(msg,w):
    global check 
    posx = msg.position.x
    posy = msg.position.y
    posz = msg.position.z
    if (posx < w[0] + 0.01 and posx > w[0] - 0.01) and (posy < w[1] + 0.01 and posy > w[1] - 0.01):
        print('pronto')
        check = True
    else:
        check = False
    return()    


if __name__ == '__main__':
    try:
        velocidade()
        l = ReqPontos()
        A = l[0]
        B = l[1]
        C = l[2]
        D = l[3]
        E = l[4]
        poses = [A,B,C,D,E]

        i = 0
        while pkg == False:
            voar(l[i])






