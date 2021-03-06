#!/usr/bin/env python



import rospy
from std_srvs.srv import Trigger
from mrs_msgs.srv import Vec4
from mrs_msgs.srv import ReferenceStampedSrv
from geometry_msgs.msg import Point
from mrs_msgs.srv import String
from mrs_msgs.msg import PositionCommand, ControlManagerDiagnostics
from std_srvs.srv import SetBool

from ger_drone.msg import Identifier, Object, LedColor
from ger_drone.srv import GetObject

import numpy as np
check = False

def ReqPontos():
    """
    Pede as informaçoes das bases que ainda nao foram processadas

    Retorno:
    @return: lista com as posiçoes de todos as bases nao processadas dadas em [x,y,z]
    """
    rospy.wait_for_service('get_object')
    a = rospy.ServiceProxy('get_object', GetObject)
    requ = GetObject._request_class()

    requ.identifier.state.data = Identifier.STATE_NOPROCESSADO
    requ.identifier.type.data = Identifier.TYPE_BASE

    response = a(requ)
    lista = response.list

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
    """
    Pede as informacoes dos mostradores

    Retorno:
    @return obj: objeto com os valores dos mostradores, gas e ajuste
    """
    rospy.wait_for_service('get_object')
    proxy = rospy.ServiceProxy('get_object', GetObject)
    requ = GetObject._request_class()

    requ.identifier.type.data = Identifier.TYPE_MOSTRADOR
    requ.identifier.state.data = Identifier.STATE_NOPROCESSADO

    response = proxy(requ)
    obj = response.list

    return obj


def Voar(a):
    """!
        Voa ate uma posicao, verificando se chegou nela

        Parametros:
            @param a: lista [x,y,z] com a posicao desejada
    """
    print(a)
    rospy.wait_for_service('/uav1/control_manager/reference')
    tres = rospy.ServiceProxy('/uav1/control_manager/reference',ReferenceStampedSrv)

    reqc = ReferenceStampedSrv._request_class()

    reqc.reference.position.x = a[0]
    reqc.reference.position.y = a[1]
    reqc.reference.position.z = a[2]
    reqc.reference.heading = 0
    tres(reqc)

    checar(a)


def checar(a):
    """!
       Checa se o drone ja chegou na posicao desejada

       Parametros:
           @param a: lista [x,y,z] com a posicao desejada
   """
    global check
    while check == False:
        pt = rospy.wait_for_message('/uav1/control_manager/position_cmd',PositionCommand)
        compara(pt,a)
        #print(check)
    check = False

def Velocidade():
    """!
        Altera o perfil de velocidade do drone para fast
    """
    rospy.wait_for_service('/uav1/constraint_manager/set_constraints')
    quatro = rospy.ServiceProxy('/uav1/constraint_manager/set_constraints',String)
    reqd = String._request_class()
    reqd.value = 'fast'

    quatro(reqd)

def PreparaPouso(j):
    """!
    Prepara o drone para pousar de forma precisa na base inicial
    """
    a = [j[0],j[1],0.5]
    Voar(a)
    ajustaPonto(a)
    lista = [x,y,z]


def recebeDiagnostico(msg):
    """!
        Verfica se chegou no ponto de destino
        Solucao mais precisa e pratica que a funcao checar

        @param msg: objeto que contém dados como a posicao atual
    """
    global chegou
    if msg.tracker_status.have_goal == False:
        chegou = True
    else:
        chegou = False

def compara(msg,w):
    """!
        Compara posicao do drone com o seu destino

        Parametros:
        @param msg: objeto que contém dados como a posicao atual
        @param w: lista [x,y,x] com a posicao desejada
    """
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

def ajustaPonto(ponto):
    """!
    Muda o controlador do Drone por um breve momento, somente para realizar a aproximação do Drone

    Mesma funcionalidade de preparapouso porem mais precisa
    """
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

def ativaLed(cor, tempo):
    """
    Altera a cor do led por um determinado tempo e para uma determinada corresponde

    @param cor: lista [R,G,B] com a cor do led
    @param tempo: tempo que o drone ficara com o led acesso
    """

    msg = LedColor()
    msg.r = cor[0]
    msg.g = cor[1]
    msg.b = cor[2]
    pubLed.publish(msg)
    rospy.sleep(tempo)

    msg = LedColor()
    msg.r = 0
    msg.g = 0
    msg.b = 0
    pubLed.publish(msg)

def pousar():
    """!
        Pousa o drone na posicao atual
     """
   print('P')
   rospy.wait_for_service('/uav1/uav_manager/land')
   um = rospy.ServiceProxy('/uav1/uav_manager/land', Trigger)
   reqa = Trigger._request_class()
   um(reqa)

def Sinal(gas,ajuste):
    """
    Define a cor do led se o valor do gas estiver entre 45 e 55 para verde, senao vermelho
    Define a cor do led se o valor do ajuste estiver entre -5 e 5 para verde, senao vermelho

    @param gas: valor do gas
    @param ajuste: valor do ajuste
    """
    ativaLed([255,0,255], 10)

    if (gas>=45) and (gas<=55):
        ativaLed([0,255,0], 10)
    else:
        ativaLed([255,0,0], 10)

    rospy.sleep(32)

    if (ajuste>=-5) and (ajuste<=5):
        ativaLed([0,255,0], 10)
    else:
        ativaLed([255,0,0], 10)


def publicaProcessado(obj):
    """
    Publica se a base ja foi processadas

    @param obj: base que acabou de ser processada
    """
    obj.identifier.state.data = Identifier.STATE_PROCESSADO
    pubObj.publish(obj)

if __name__ == '__main__':

    rospy.init_node('fase3', anonymous="True")

    pubObj = rospy.Publisher('objeto_detectado',Object, queue_size=10)
    pubLed = rospy.Publisher('led_color', LedColor, queue_size=10)

    try:
        Velocidade()
        pontos = ReqPontos()

        for i in pontos:
            Voar([i[0],i[1],2.0])
            rospy.sleep(2)
            Voar([i[0],i[1],0.5])
            ajustaPonto([i[0],i[1],0.5])
            rospy.sleep(10)


            lista = ReqMostrador()

            for obj in lista:
                stri = obj.identifier.data
                print(stri+" |Pose: "+": "+ str(obj.pose.position.x)+ " "
                    +str(obj.pose.position.y)+" "+str(obj.pose.position.z))
                val = stri.split(" ")
                Sinal(int(val[0]),int(val[1]))
                publicaProcessado(obj)
                rospy.sleep(5)

        Voar([0.0,0.0,1.0])
        pousar()







    except rospy.ROSInternalException:
        pass
