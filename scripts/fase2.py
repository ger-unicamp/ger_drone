#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger
from mrs_msgs.srv import ReferenceStampedSrv, Vec4, String
from geometry_msgs.msg import Point
from mrs_msgs.msg import PositionCommand
from ger_drone.msg import Identifier, LedColor
from ger_drone.srv import GetObject
from std_srvs.srv import SetBool

import numpy as np

rospy.init_node('caminho2')
check = False

def decolar():
      """!
        O drone decola a partir desse momento
        Não é recomendado efetuar outros comandos enquanto ele decola
    """
    print('D')
    rospy.wait_for_service('/uav1/uav_manager/takeoff')
    dois = rospy.ServiceProxy('/uav1/uav_manager/takeoff', Trigger)
    reqb = Trigger._request_class()
    dois(reqb)

def pousar():
       """!
       Pousa o drone na posicao atual
    """
   print('P')
   rospy.wait_for_service('/uav1/uav_manager/land')
   um = rospy.ServiceProxy('/uav1/uav_manager/land', Trigger)
   reqa = Trigger._request_class()
   um(reqa)


def voar(a):
         """!
        Voa ate uma posicao, verificando se chegou nela

        Parametros:
            @param a: lista [x,y,z] com a posicao desejada
    """
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

    tBase = np.array([w[0],w[1]], np.float32)
    tDrone = np.array([posx, posy], np.float32)

    if (np.linalg.norm(tBase-tDrone)<0.01):
        print('pronto')
        check = True
    else:
        check = False
    
    return   


def velocidade():
      """!
        Altera o perfil de velocidade do drone para fast
    """
    rospy.wait_for_service('/uav1/constraint_manager/set_constraints')
    quatro = rospy.ServiceProxy('/uav1/constraint_manager/set_constraints',String)
    reqd = String._request_class()
    reqd.value = 'slow'
        
    quatro(reqd)

def getSensor():
    """!
        Requisita um objeto que contém quais foram os sensores vemelhos localizados e a sua posicao
        Adiciona as poses desses sensores em lista
    """
    print('entrou')
    rospy.wait_for_service('get_object')
    a = rospy.ServiceProxy('get_object', GetObject)
    reqf = GetObject._request_class()
    
    reqf.identifier.type.data = Identifier.TYPE_SENSOR_VERMELHO

    response = a(reqf)
    lista = response.list
    print(len(lista))


    rospy.wait_for_service('/ger_drone/set_atualiza_mapa')
    proxy = rospy.ServiceProxy('/ger_drone/set_atualiza_mapa', SetBool)
    req = SetBool._request_class()
    
    req.data = False

    proxy(req)

    voarSensor(lista)

def voarSensor(lista):
    """!
        Voa ate a posicao dos sensores vermelhos
    """    
    PoseSensores = []
    for i in lista:
        p = [(i.pose.position.x),i.pose.position.y,0.6]
       
        
        PoseSensores.append(p)
    for j in PoseSensores:
        print(j)
        voar(j)

        ajustaPonto(j)

        rospy.sleep(1)
        ativaLed()
        #rospy.sleep(2)

def ativaLed():
    """!
        Mantém o LED acesso por 15 segundos (especificacao da competicao)
    """
    msg = LedColor()
    msg.r = 255
    msg.g = 0
    msg.b = 0
    pub.publish(msg)
    rospy.sleep(15)
    
    msg = LedColor()
    msg.r = 0
    msg.g = 0
    msg.b = 0
    pub.publish(msg)

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

    voar(ponto)

    rospy.sleep(2)

    req.value = "MpcController"
    proxy(req)

    rospy.sleep(1)


if __name__ == '__main__':
    try:
        velocidade()
        pub = rospy.Publisher('led_color', LedColor, queue_size=10)
        #decolar()
        #rospy.sleep(8)
        # eq do caminho y = -5.2941x + 14.1882   entre x = 2.68 e x =3.87
        partida = [2.68,0,0.8]
        voar(partida)
        rospy.sleep(3)
    

        px =[]
        n = 0
        poses =[]
        z = 0.5
        for i in range(0,9):
            p = 2.65 + n*(1.2/9)
            n = n+1
            px.append(p)
        
        px.append(3.85)
        for j in px:
            y = -5.2941*j + 14.1882
            poses.append([j,y,z])    

        for h in poses:
            voar(h)
            rospy.sleep(3)
        getSensor()

        rospy.sleep(5)
        voar([0,0,1.5])
        ajustaPonto([0,0,1])
        rospy.sleep(2)
        pousar()    

    except rospy.ROSInternalException:
        pass    
