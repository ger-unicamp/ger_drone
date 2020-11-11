#!/usr/bin/env python



import rospy
from std_srvs.srv import Trigger
from mrs_msgs.srv import Vec4
from mrs_msgs.srv import ReferenceStampedSrv
from geometry_msgs.msg import Point
from mrs_msgs.srv import String 
from mrs_msgs.msg import PositionCommand, ControlManagerDiagnostics
from std_srvs.srv import SetBool

from ger_drone.msg import Identifier
from ger_drone.srv import GetObject

import numpy as np

check = False
chegou = False

rospy.init_node('fase1')

def pousar():
    print('P')
    rospy.wait_for_service('/uav1/uav_manager/land')
    um = rospy.ServiceProxy('/uav1/uav_manager/land', Trigger)
    reqa = Trigger._request_class()
    um(reqa)

    rospy.sleep(1)
    while(chegou == False):
        pass
   

def decolar():
    print('D')
    rospy.wait_for_service('/uav1/uav_manager/takeoff')
    dois = rospy.ServiceProxy('/uav1/uav_manager/takeoff', Trigger)
    reqb = Trigger._request_class()
    dois(reqb)

    rospy.sleep(1)

    while(chegou == False):
        pass

def rotina():
    pousar()
    rospy.sleep(2)
    decolar()
    rospy.sleep(2)
    

def voar(a):
    print(a)
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
   
    checar(lista)

def velocidade():
    rospy.wait_for_service('/uav1/constraint_manager/set_constraints')
    quatro = rospy.ServiceProxy('/uav1/constraint_manager/set_constraints',String)
    reqd = String._request_class()
    reqd.value = 'fast'
        
    quatro(reqd)

def checar(a):
    global check 
    while (check == False) or (chegou == False):
        pt = rospy.wait_for_message('/uav1/control_manager/position_cmd',PositionCommand)
        compara(pt,a)
        #print(check)
    check = False
    rospy.sleep(5)    

def compara(msg,w):
    global check 
    posx = msg.position.x
    posy = msg.position.y
    posz = msg.position.z

    tBase = np.array([w[0],w[1]], np.float32)
    tDrone = np.array([posx, posy], np.float32)

    if (np.linalg.norm(tBase-tDrone)<0.01):
        print('pronto')
        check = True
    else:
        check = False
    
    return


def getListObject():
    rospy.wait_for_service('get_object')
    cinco = rospy.ServiceProxy('get_object', GetObject)
    reqc = GetObject._request_class()

    reqc.identifier.state.data = Identifier.STATE_NOPROCESSADO
    reqc.identifier.type.data = Identifier.TYPE_BASE

    response = cinco(reqc)
    lista =  response.list

    rospy.wait_for_service('/ger_drone/set_atualiza_mapa')
    proxy = rospy.ServiceProxy('/ger_drone/set_atualiza_mapa', SetBool)
    req = SetBool._request_class()
    
    req.data = False

    proxy(req)
    
    getPose(lista)

def getPose(lista):
    listaPoses = []
    for i in lista:
        #a = [i.pose.position.x, i.pose.position.y, i.pose.position.z]
        a = [i.pose.position.x, i.pose.position.y, 2]
        listaPoses.append(a)
    print(listaPoses)
    for j in listaPoses:
        print(j)
        voar(j)
        preparapouso(j)
        rotina()

def preparapouso(j):
    a = [j[0],j[1],1]
    voar(a)
    ajustaPonto(a)

def recebeDiagnostico(msg):
    global chegou
    if msg.tracker_status.have_goal == False:
        chegou = True
    else:
        chegou = False

def ajustaPonto(ponto):
    rospy.wait_for_service('/uav1/control_manager/switch_controller')
    proxy = rospy.ServiceProxy('/uav1/control_manager/switch_controller', String)
    req = String._request_class()
    
    req.value = "Se3Controller"
    proxy(req)

    rospy.sleep(1)

    voar(ponto)

    rospy.sleep(3)

    req.value = "MpcController"
    proxy(req)

    rospy.sleep(1)

if __name__ == '__main__':
    
    rospy.Subscriber('/uav1/control_manager/diagnostics/', ControlManagerDiagnostics, recebeDiagnostico)
    
    try:
        
        velocidade()
        #pontos = [[2,0,3],[4,0,3], [6,0,3],[6,-1.5,3] ,[4,-1.5,3] ,[2,-1.5,3],[0,-1.5,3],[0,-3,3], [2,-3,3], [4,-3,3], [6,-3,3],[6,-4.5,3], [4,-4.5,3], [2,-4.5,3],[0,-4.5,3],[0,-6,3],[2,-6,3], [4,-6,3], [6,-6,3]]
        pontos = [[2,0,3],[4,0,3], [6,0,3],[6,-1.2,3] ,[4,-1.2,3] ,[2,-1.2,3],[0,-1.2,3],[0,-2.4,3], [2,-2.4,3], [4,-2.4,3], [6,-2.4,3],[6,-3.6,3], [4,-3.6,3], [2,-3.6,3],[0,-3.6,3],[0,-4.8,3],[2,-4.8,3], [4,-4.8,3], [6,-4.8,3],[6,-6,3],[4,-6,3],[2,-6,3],[0,-6,3]]
        #pontos = [[2,0,2.5],[3,0,2.5],[4,0,2.5], [6,0,3],[6,-1.2,3] ,[4,-1.2,3] ,[2,-1.2,3],[0,-1.2,3],[0,-2.4,3], [2,-2.4,3], [4,-2.4,3], [6,-2.4,3]] 
        
        for i in pontos:
            voar(i)
        rospy.sleep(2)
        getListObject()  

        #voltar e finalizar
        base = [0,0,2]
        voar(base)
        base = [0,0,1]
        ajustaPonto(base)
        rospy.sleep(2)
        pousar()
        

    except rospy.ROSInternalException:
        pass    
      





