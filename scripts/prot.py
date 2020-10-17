#!/usr/bin/env python



import rospy
from std_srvs.srv import Trigger
from mrs_msgs.srv import Vec4
from mrs_msgs.srv import ReferenceStampedSrv
from geometry_msgs.msg import Point
from mrs_msgs.srv import String 
from mrs_msgs.msg import PositionCommand
from geometry_msgs.msg import Point

'''
from ger_drone.msg import Identifier
from ger_drone.srv import GetObject
'''


check = False

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
    rospy.sleep(8)
    decolar()
    rospy.sleep(8)
    

def voar(a):
    rospy.wait_for_service('/uav1/control_manager/reference')
    tres = rospy.ServiceProxy('/uav1/control_manager/reference',ReferenceStampedSrv)
    
    reqc = ReferenceStampedSrv._request_class()
    
    x =a[0]
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
    reqd.value = 'slow'
        
    quatro(reqd)

def checar(a):
    global check 
    while check == False:
        pt = rospy.wait_for_message('/uav1/control_manager/position_cmd',PositionCommand)
        compara(pt,a)
        print(check)
    check = False    

def compara(msg,w):
    global check 
    posx = msg.position.x
    posy = msg.position.y
    posz = msg.position.z
    if (posx < w[0] + 0.1 and posx > w[0] - 0.1) and (posy < w[1] + 0.1 and posy > w[1] - 0.1):
        print('pronto')
        check = True
    else:
        check = False
    return()    

'''
def getListObject():
    rospy.wait_for_service('get_object')
    cinco = rospy.ServiceProxy('get_object', Identifier)
    reqc = Identifier._request_class()

    reqc.identifier.state = Identifier.STATE_NOPROCESSADO
    reqc.identifier.type = Identifier.TYPE_BASE

    response = cinco(reqc)
    lista =  response.list
    getPose(lista)

def getPose(lista):
    listaPoses = []
    for i in lista:
        a = [i.pose.position.x, i.pose.position.y, i.pose.position.z]
        listaPoses.append(a)
    for j in listaPoses:
        voar(j)
        rotina()
'''

if __name__ == '__main__':
    try:
        velocidade()
        pontos = [[6,2,2.5],[4,2,2.5],[2,2,2.5],[2,3.5,2.5],[4,3.5,2],[6,3.5,2],[7.5,3.5,2],[7.5,5,2],[6,5,2],[4,5,2],[2,5,2],[2,7.5,2],[4,7.5,2],[6,7.5,2],[7.5,7.5,2],[8.1,2,2]]
        
        decolar()

        rospy.sleep(8)

        for i in pontos:
            voar(i)
        
        rospy.sleep(4)
        pousar()  

        #getListObject()  
        

    except rospy.ROSInternalException:
        pass    
      





