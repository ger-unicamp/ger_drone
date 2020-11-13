#!/usr/bin/env python

import rospy
from mrs_msgs.srv import String 
from mrs_msgs.msg import PositionCommand, ControlManagerDiagnostics
from ger_drone.msg import Identifier, Object
from ger_drone.srv import GetObject
from geometry_msgs.msg import Point
from mrs_msgs.srv import ReferenceStampedSrv
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger
import numpy as np

rospy.init_node('fase4')
check = False
chegou = False

def velocidade():
    """!
        Altera o perfil de velocidade do drone para faset
    """
    rospy.wait_for_service('/uav1/constraint_manager/set_constraints')
    quatro = rospy.ServiceProxy('/uav1/constraint_manager/set_constraints',String)
    reqd = String._request_class()
    reqd.value = 'fast'
        
    quatro(reqd)

def ReqPontos():
    """!
        Requisita os pontos das bases ao mapa

        Retorno:
            @return Listas de Object com as bases
    """
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
    """!
        Voa ate uma posicao, verificando se chegou nela
        
        Parametros:
            @param a: lista [x,y,z] com a posicao desejada
    """
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
    """!
        Checa se o drone ja chegou na posicao desejada
        
        Parametros:
            @param a: lista [x,y,z] com a posicao desejada
    """
    global check, chegou
    while (check == False) or (chegou == False):
        pt = rospy.wait_for_message('/uav1/control_manager/position_cmd',PositionCommand)
        compara(pt,a)
        #print(check)
    check = False    
    print('CHEGOU')

def recebeDiagnostico(msg):
    global chegou
    if msg.tracker_status.have_goal == False:
        chegou = True
    else:
        chegou = False

def compara(msg,w):
    global check 
    posx = msg.position.x
    posy = msg.position.y
    posz = msg.position.z

    tMsg = np.array([w[0],w[1]], np.float32)
    tDrone = np.array([posx, posy], np.float32)

    if (np.linalg.norm(tMsg-tDrone)<0.01):
        check = True
    else:
        check = False
    
    return  
 

def separa_lista2(lista):
    """!
        Voa ate as bases da lista, e em cada uma solicita o cubo ao mapa, pega ela, voa e deixa o cubo

        Parametros:
            @param lista de Object contendo as bases
    """
    base1 = [lista[0].pose.position.x, lista[0].pose.position.y,2.5]
    voar(base1)
    rospy.sleep(3)
    base1[2] = 0.5
    voar([base1[0]+0.2, base1[1]+0.2, base1[2]])
    del(lista[0])
    rospy.sleep(5)

    for i in range(4):
        cubo = base_cubo()
        publicaProcessado(cubo)
        proximo = ChecaPose(cubo.identifier.data)
        rospy.sleep(3)
        ativa_garra(cubo.identifier.data)
        rospy.sleep(3)
        voar(proximo)
        rospy.sleep(4)
        voar([proximo[0],proximo[1],0.8])
        rospy.sleep(5)
        desativa_garra(cubo.identifier.data)
        rospy.sleep(5)
        voar([proximo[0]+0.2,proximo[1]+0.2,0.5])
        

def separa_lista3(lista):
    bases = ["A","B","C","D","E"]
    baseOrigem= bases[0]

    origem = ChecaPose(bases[0])
    voar(origem)
    voar([origem[0]+0.2,origem[1]+0.2,0.5])
    #ajustaPonto([origem[0]+0.2,origem[1]+0.2,0.5])

    cubo = base_cubo()

    while len(bases) > 0: 
        bases.remove(baseOrigem)

        baseDestino = cubo.identifier.data
        destino = ChecaPose(cubo.identifier.data)
        
        
        print("Posicao do Cubo "+cubo.identifier.data+": "+ str(cubo.pose.position.x)+ " "
            +str(cubo.pose.position.y)+" "+str(cubo.pose.position.z))

        ativa_garra(cubo.identifier.data)
        rospy.sleep(3)

        #Vai ate o destino
        voar([origem[0], origem[1], 2.0])
        rospy.sleep(5)
        voar([destino[0], destino[1], 2.5])
        rospy.sleep(3)
        voar([destino[0],destino[1],0.8])
        rospy.sleep(5)
        #ajustaPonto([destino[0],destino[1],0.8])


        #Solta o cubo
        desativa_garra(cubo.identifier.data)
        rospy.sleep(3)
        voar([destino[0]+0.2,destino[1]+0.2,0.5])

        publicaProcessado(cubo)

        cubo = base_cubo()
        if(cubo is None):
            if(len(bases) == 0):
                break

            baseOrigem = bases[0]
            origem = ChecaPose(bases[0])
            voar(origem)
            voar([origem[0]+0.2,origem[1]+0.2,0.5])
            #ajustaPonto([origem[0]+0.2,origem[1]+0.2,0.5])
            cubo = base_cubo()
        else:
            origem = destino
            baseOrigem = baseDestino
            

        
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

         



def base_cubo():
    """!
        Solicita e retorna o primeiro cubo nao processado detectado

        Retorno:
            @return Object do cubo detectado
    """
    rospy.wait_for_service('get_object')
    a = rospy.ServiceProxy('get_object', GetObject)
    rospy.sleep(5)
    req = GetObject._request_class()
    
    
    req.identifier.type.data = Identifier.TYPE_PACOTE
    req.identifier.state.data = Identifier.STATE_NOPROCESSADO

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

    if(len(lista) != 0):
        return lista[0]
    else:
        return None 


def ChecaPose(i):
    """!
        Checa qual pose o cubo deve ir

        Parametros:
            @param i: A letra do cubo
        
        Retorno:
            @return destino: Lista com a posicao [x,y,z] para onde o cubo deve ir
    """
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
    """!
        Publica ao mapa indicando que um objeto foi processado
        
        Parametros:
            @param obj: Objecto com o objeto a ser alterado no mapa
    """
    obj.identifier.state.data = Identifier.STATE_PROCESSADO
    pubObj.publish(obj)
        
def desativa_garra(letra):
    """!
        Desfaz o link entre o drone e o pacote

        Parametros:
            letra: letra do pacote
    """
    rospy.wait_for_service('/link_attacher_node/detach')
    garra = rospy.ServiceProxy('/link_attacher_node/detach',Attach)

    req = AttachRequest()
    req.model_name_1 = "uav1"
    req.link_name_1 = "base_link"
    req.model_name_2 = "equipment"+letra
    req.link_name_2 = "link_"+letra

    garra(req)




def ativa_garra(letra):
    """!
        Cria o link entre o drone e o pacote

        Parametros:
            letra: letra do pacote
    """
    rospy.wait_for_service('/link_attacher_node/attach')
    garra = rospy.ServiceProxy('/link_attacher_node/attach',Attach)

    req = AttachRequest()
    req.model_name_1 = "uav1"
    req.link_name_1 = "base_link"
    req.model_name_2 = "equipment"+letra
    req.link_name_2 = "link_"+letra

    garra(req) 

def pousar():
    """!
       Pousa o drone na posicao atual
    """
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

        rospy.Subscriber('/uav1/control_manager/diagnostics/', ControlManagerDiagnostics, recebeDiagnostico)

        #Se o codigo falhar, finaliza voltando ate a base costeira
        try:
            lista = ReqPontos() 
            caminho = separa_lista3(lista)
            #carrega_cubo(caminho)
        except:
            pass

        base_costeira = [0,0,2.5]
        voar(base_costeira)
        rospy.sleep(3)
        pousar()


     
       
    except rospy.ROSInternalException:
        pass    
      








