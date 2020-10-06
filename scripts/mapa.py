# @todo uma base detectada ja foi armazenada?
# @todo documentar o que o nó faz na wiki do git do drone

#!/usr/bin/env python

import rospy

import numpy as np
from scipy.spatial.transform import Rotation as R
from matplotlib import pyplot as plt

from mrs_msgs.msg import UavState
from ger_drone.msg import Object, Identifier, ObjectState

from ger_drone.srv import GetObject, Identifier


# Classe objeto abstrata
# @todo fazer classe única, diferentes objetos diferentes atributos
class Objeto:

    def __init__(self, x, y):
        # Coordenadas globais
        self.x = x
        self.y = y


class Base(Objeto):
    # @todo atributo visitada


# Lista de bases inicialmente vazia
bases = []

# Vetores posição e rotação para cálculo da pose global em recebeOdometria e recebeObjeto
position = np.asarray([0,0,0])
rotation = np.asarray([[1,0,0],[0,1,0],[0,0,1]])


# Identifica, cria e guarda objeto recebido por mensagem
def recebeObjeto(msg):

    # Posição do objeto nas coordenadas da câmera
    cameraT = np.asarray([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    cameraR = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]).as_dcm()

    # @todo concatenar pose da camera no frame do drone

    # Concatena com a pose do drone para obter a pose global
    worldR = np.matmul(rotation, cameraR)
    worldT = position+np.matmul(rotation, cameraT)

    # Limites da pose global
    if(worldT[0] > 8 or worldT[0] < 0):
        return
    if(worldT[1] > 8 or worldT[1] < 0):
        return

    # Objeto é uma base? Insere na lista de bases com a pose global
    if msg.identifier == 0
        bases.append(Base(worldT[0], worldT[1]))
    
    r = R.from_dcm(worldR)
    theta = r.as_euler('xyz')[2]


# Recebe odometria por mensagem para calcular a pose global em recebeObjeto
def recebeOdometria(msg):

    global position, rotation

    position[0] = msg.pose.position.x
    position[1] = msg.pose.position.y
    position[2] = msg.pose.position.z

    r = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])
    rotation = r.as_dcm()


# Identifica e retorna lista de objetos requisitada pelo serviço GetObject
def entregaListaObjetos(req):

    # Objeto requisitado é uma base?
    if req.identifier == 0
        return GetObject(bases) # @todo ta certo isso? parece temos q definir o tipo de retorno do serviço (GetObject)


if __name__ == '__main__':
    try:

        # Inicia nó com nome 'mapa'
        rospy.init_node('mapa', anonymous="True")

        # Subscriber para receber objeto por mensagem
        rospy.Subscriber('objeto_detectado', Object, recebeObjeto)

        # Subscriber para receber odometria por mensagem
        rospy.Subscriber('/uav1/odometry/uav_state', UavState, recebeOdometria)

        # Serviço GetObject para prover objetos
        rospy.Service('get_object', GetObject, entregaListaObjetos)

        # Define a frequência de execução em Hz
        rate = rospy.Rate(10)

        #rospy.spin() substitui o rospy.is_shutdown abaixo?

        while not rospy.is_shutdown():

            # Espera o tempo para executar o programa na frequência definida
            rate.sleep()

    except rospy.ROSInternalException:
        pass