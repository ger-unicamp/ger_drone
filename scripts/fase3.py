#!/usr/bin/env python



import rospy
from std_srvs.srv import Trigger
from mrs_msgs.srv import Vec4
from mrs_msgs.srv import ReferenceStampedSrv
from geometry_msgs.msg import Point
from mrs_msgs.srv import String
from mrs_msgs.msg import PositionCommand, ControlManagerDiagnostics
from std_srvs.srv import SetBool

from ger_drone.msg import Identifier, object
from ger_drone.srv import GetObject

import numpy as np

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

    listaPontos = []
    for i in lista:
        x = i.pose.position.x
        y = i.pose.position.y
        z = i.pose.position.z
        listaPontos.append([x,y,z])
    return(listaPontos)

def ReqMostrador():
    rospy.wait_for_service('get_object')
    a = rospy.ServiceProxy('get_object', GetObject)
    requ = GetObject._request_class()

    requ.identifier.type.data = Identifier.TYPE_MOSTRADOR

    response = requ(a)
    obj = response.list

    return obj


def Voar(a):
    print(a)
    rospy.wait_for_service('/uav1/control_manager/reference')
    tres = rospy.ServiceProxy('/uav1/control_manager/reference',ReferenceStampedSrv)

    reqc = ReferenceStampedSrv._request_class()

    x = a[0]
    y = a[1]
    z = 2.0
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

def Velocidade():
    rospy.wait_for_service('/uav1/constraint_manager/set_constraints')
    quatro = rospy.ServiceProxy('/uav1/constraint_manager/set_constraints',String)
    reqd = String._request_class()
    reqd.value = 'fast'

    quatro(reqd)

def PreparaPouso(j):
    a = [j[0],j[1],0.5]
    Voar(a)
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

    Voar(ponto)

    rospy.sleep(3)

    req.value = "MpcController"
    proxy(req)

    rospy.sleep(1)

def ativaLed(cor):

    msg = LedColor()
    msg.r = cor[0]
    msg.g = cor[1]
    msg.b = cor[2]
    pub.publish(msg)
    rospy.sleep(15)
    
    msg = LedColor()
    msg.r = 0
    msg.g = 0
    msg.b = 0
    pub.publish(msg)

def pousar():
   print('P')
   rospy.wait_for_service('/uav1/uav_manager/land')
   um = rospy.ServiceProxy('/uav1/uav_manager/land', Trigger)
   reqa = Trigger._request_class()
   um(reqa)

def Sinal(gas,ajuste):
    if (gas<=45) or (gas>=55):
        ativaLed([255,0,0])
    else:
        ativaLed([0,255,0])
    
    rospy.sleep(45)
    if (ajuste<=-5) or (ajuste>=5):
        ativaLed([255,0,0])
    else:
        ativaLed([0,255,0])


def publicaProcessado(obj):
    obj.identifier.state.data = Identifier.STATE_PROCESSADO
    pubObj.publish(obj)

if __name__ == '__main__':

    rospy.init_node('fase3', anonymous="True")

    pubObj = rospy.Publisher('objeto_detectado',Object, queue_size=10)

    try:
        Velocidade()
        pontos = ReqPontos()

        for i in pontos:
            Voar([i[0],i[1],2.0])
            lista = ReqMostrador()
            obj = lista[-1]
            stri = obj.identifier.data
            val = stri.split(" ")
            sinal(val[0],val[1])
            publicaProcessado(obj)
            rospy.sleep(5)

        Voar([0.0,0.0,1.0])
        pousar()


            
            
            


    except rospy.ROSInternalException:
        pass
