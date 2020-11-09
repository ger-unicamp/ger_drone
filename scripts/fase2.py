#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger
from mrs_msgs.srv import Vec4
from mrs_msgs.srv import ReferenceStampedSrv
from geometry_msgs.msg import Point
from mrs_msgs.srv import String 
from mrs_msgs.msg import PositionCommand
from ger_drone.msg import Identifier
from ger_drone.srv import GetObject
from ger_drone.msg import LedColor

rospy.init_node('caminho2')
check = False

def decolar():
    print('D')
    rospy.wait_for_service('/uav1/uav_manager/takeoff')
    dois = rospy.ServiceProxy('/uav1/uav_manager/takeoff', Trigger)
    reqb = Trigger._request_class()
    dois(reqb)

def pousar():
   print('P')
   rospy.wait_for_service('/uav1/uav_manager/land')
   um = rospy.ServiceProxy('/uav1/uav_manager/land', Trigger)
   reqa = Trigger._request_class()
   um(reqa)


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
    reqc.reference.heading = 0.2
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


def velocidade():
    rospy.wait_for_service('/uav1/constraint_manager/set_constraints')
    quatro = rospy.ServiceProxy('/uav1/constraint_manager/set_constraints',String)
    reqd = String._request_class()
    reqd.value = 'slow'
        
    quatro(reqd)

def getSensor():
    print('entrou')
    rospy.wait_for_service('get_object')
    a = rospy.ServiceProxy('get_object', GetObject)
    reqf = GetObject._request_class()
    
    reqf.identifier.type.data = Identifier.TYPE_SENSOR_VERMELHO

    response = a(reqf)
    lista = response.list
    print(len(lista))
    voarSensor(lista)

def voarSensor(lista):
    PoseSensores = []
    for i in lista:
        print(i)
        if i.pose.position.y < -3.4:
            p = [(i.pose.position.x)+0.4,i.pose.position.y,0.6]
        else:
            p = [i.pose.position.x,i.pose.position.y,0.6]
        PoseSensores.append(p)
    for j in PoseSensores:
        print(j)
        voar(j)
        rospy.sleep(1)
        ativaLed()
        rospy.sleep(2)

def ativaLed():
    msg = LedColor()
    msg.r = 255
    msg.g = 0
    msg.b = 0
    pub.publish(msg)
    rospy.sleep(4)
    
    msg = LedColor()
    msg.r = 0
    msg.g = 0
    msg.b = 0
    pub.publish(msg)

   

if __name__ == '__main__':
    try:
        velocidade()
        pub = rospy.Publisher('led_color', LedColor, queue_size=10)
        #decolar()
        #rospy.sleep(8)
        # eq do caminho y = -5.25x + 13.9125   entre x = 2.65 e x =3.85
        partida = [2.65,0,0.8]
        voar(partida)
        rospy.sleep(3)
    

        px =[]
        n = 0
        poses =[]
        z = 0.34
        for i in range(0,11):
            p = 2.65 + n*(1.2/11)
            n = n+1
            px.append(p)
        
        px.append(3.85)
        for j in px:
            y = -5.25*j + 13.9125
            poses.append([j,y,z])    

        for h in poses:
            voar(h)
            rospy.sleep(8)
        rospy.sleep(5)
        getSensor()

        rospy.sleep(5)
        voar([0,0,1.5])
        rospy.sleep(2)
        pousar()    

    except rospy.ROSInternalException:
        pass    
