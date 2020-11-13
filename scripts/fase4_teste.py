#!/usr/bin/env python

import rospy
from mrs_msgs.srv import String 
from mrs_msgs.msg import PositionCommand
from ger_drone.msg import Identifier, Object
from ger_drone.srv import GetObject
from geometry_msgs.msg import Point
from mrs_msgs.srv import ReferenceStampedSrv
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from std_srvs.srv import SetBool


rospy.init_node('fase4')
check = False

def velocidade():
    rospy.wait_for_service('/uav1/constraint_manager/set_constraints')
    quatro = rospy.ServiceProxy('/uav1/constraint_manager/set_constraints',String)
    reqd = String._request_class()
    reqd.value = 'fast'
        
    quatro(reqd)

def ReqPontos():
    rospy.wait_for_service('get_object')
    a = rospy.ServiceProxy('get_object', GetObject)
    requ = GetObject._request_class()
    
    requ.identifier.state.data = Identifier.STATE_NOPROCESSADO        #mudar para processado quando visitar
    requ.identifier.type.data = Identifier.TYPE_BASE

    response = a(requ)
    lista = response.list
    #print(lista)

    #lista[i].identifier.data == "A"

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
    z = a[2]
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

def separa_lista(lista):
    rotina = []
    for i in lista:
        px = i.pose.position.x
        py = i.pose.position.y
        pz = 2.5
        base = [px,py,pz]
        voar(base)
        rospy.sleep(3)
        voar([px,py,0.5])
        rospy.sleep(4) 
        cubo_info = base_cubo(base)
        rospy.sleep(3)
        publicaProcessado(i)
        voar([px,py,2])
        infos = [base,cubo_info[1],cubo_info[0]]
        rotina.append(infos)
        print(infos)
    print(rotina) 
    return(rotina) 

def separa_lista2(lista):
    base1 = [lista[0].pose.position.x, lista[0].pose.position.y,2.5]
    voar(base1)
    rospy.sleep(3)
    base1[2] = 0.5
    voar(base1)
    rospy.sleep(5)

    cubo = base_cubo()
    proximo = cubo.identifier.data


def proximo(s)



def base_cubo():
    rospy.wait_for_service('get_object')
    a = rospy.ServiceProxy('get_object', GetObject)
    rospy.sleep(5)
    req = GetObject._request_class()
    
    req.identifier.type.data = Identifier.TYPE_PACOTE
    response = a(req)
    lista = response.list
    #print(lista)
    #bases_atuais = []
    for i in lista:
        cuboLetra = i.identifier.data
        #print(cuboLetra)
        cuboPose = ChecaPose(cuboLetra)
        info = [cuboLetra,cuboPose]
        #print('info',info)
        #bases_atuais.append(cuboLetra)

    #print(bases_atuais,'bases')  
    return lista[0] 

def ChecaPose(i):
    destino =[]
    C = [4.25,-2,2.5]
    B = [5.25,0,2.5]
    A = [1.25,-3,2.5]
    E = [3.25,-0.08,2.5]
    D = [0.25,-6,2.5]

    if i == 'A':
        destino = A
    elif i == 'B':
        destino = B    
    elif i == 'C':
        destino = C 
    elif i == 'D':
        destino = D 
    elif i == 'E':
        destino = E 
    #print('pacote',i,destino)    

    return(destino)

def publicaProcessado(obj):
    obj.identifier.state.data = Identifier.STATE_PROCESSADO
    pubObj.publish(obj)

def carrega_cubo(caminho):
    for i in caminho:
        base = i[0]
        letra = i[2]
        destino = i[1]
        voar(base)
        rospy.sleep(3)
        altura_safe = [base[0]+0.2,base[1]+0.2,0.6]
        voar(altura_safe)
        rospy.sleep(8)
        ativa_garra(letra)
        rospy.sleep(8)
        voar(destino)
        altura_safe2 = [destino[0]+0.2,destino[1]+0.2,0.8]
        voar(altura_safe2)
        rospy.sleep(8)
        desativa_garra(letra)
        rospy.sleep(8)
        
def desativa_garra(letra):
    rospy.wait_for_service('/link_attacher_node/detach')
    garra = rospy.ServiceProxy('/link_attacher_node/detach',Attach)

    req = AttachRequest()
    req.model_name_1 = "uav1"
    req.link_name_1 = "base_link"
    req.model_name_2 = "equipment"+letra
    req.link_name_2 = "link_"+letra

    garra(req)




def ativa_garra(letra):
    print(letra)
    rospy.wait_for_service('/link_attacher_node/attach')
    garra = rospy.ServiceProxy('/link_attacher_node/attach',Attach)

    req = AttachRequest()
    req.model_name_1 = "uav1"
    req.link_name_1 = "base_link"
    req.model_name_2 = "equipment"+letra
    req.link_name_2 = "link_"+letra

    garra(req) 

def pousar():
    print('P')
    rospy.wait_for_service('/uav1/uav_manager/land')
    um = rospy.ServiceProxy('/uav1/uav_manager/land', Trigger)
    reqa = Trigger._request_class()
    um(reqa)

    rospy.sleep(1)







if __name__ == '__main__':
    try:
        pubObj = rospy.Publisher('objeto_detectado',Object, queue_size=10)
        velocidade()

        lista = ReqPontos() 
        caminho = separa_lista(lista)
        carrega_cubo(caminho)
        base_costeira = [0,0,2.5]
        voar(base_costeira)
        rospy.sleep(3)
        pousar()


     
       
    except rospy.ROSInternalException:
        pass    
      





