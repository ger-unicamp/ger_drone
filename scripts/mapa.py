#!/usr/bin/env python

import rospy

import numpy as np
from scipy.spatial.transform import Rotation as R
from matplotlib import pyplot as plt

from mrs_msgs.msg import UavState
from ger_drone.msg import Object, Identifier, ObjectState

from ger_drone.srv import GetObject, Identifier, GetObjectResponse


# Lista de bases inicialmente vazia
objetos = []

# Vetor de posições para cálculo de coordenadas
position = np.asarray([0,0,0])
rotation = np.asarray([[1,0,0],[0,1,0],[0,0,1]])

indiceMaximo = -1

# Cria e insere objeto da mensagem na lista
def recebeObjeto(msg):

    # Posicao do obj nas coordenadas da camera
    cameraT = np.asarray([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    cameraR = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]).as_dcm()

    # Concatena com a pose do drone para obter a pose no mundo
    worldR = np.matmul(rotation, cameraR)
    worldT = position+np.matmul(rotation, cameraT)

    if(worldT[0] > 8 or worldT[0] < 0):
        return
    if(worldT[1] > 8 or worldT[1] < 0):
        return

    # Verifica se um objeto com índice -1 é o mesmo que outro objeto que já está na lista
    if(msg.identifier.index.data == -1):
        # comparar a pose se está próxima (intervalo)
        # distancia euclidiana numpy.linalg.norm
        for i in objetos:
            if (i.identifier.type.data == msg.identifier.type.data):
                if (abs(i.pose.point.x - worldT[0]) < num and
                    abs(i.pose.point.y - worldT[1]) < num and
                    abs(i.pose.point.z - worldT[2]) < num)
                    return

    # Se objeto retornar com índice != -1, verificar se o tipo bate com o objeto já na lista, e atualizar o estado
    if(msg.identifier.index.data != -1):
        for i in objetos:
            if(i.identifier.index.data == msg.identifier.index.data):
                novoObjeto.identifier.state.data = msg.identifier.state.data

    r = R.from_dcm(worldR)
    quat = r.as_quat()

    if(msg.identifier.state.data == Identifier.STATE_DESCONHECIDO):
        estado = Identifier.STATE_NOPROCESSADO
    else:
        estado = msg.identifier.state.data

    novoObjeto = Object()
    
    rospy.loginfo('Objeto armazenado.')
    
    #tipo do objeto
    novoObjeto.identifier.type.data = msg.identifier.type.data
    
    #estado do objeto
    novoObjeto.identifier.state.data = estado
    
    #pose do objeto
    novoObjeto.pose.point.x = worldT[0]
    novoObjeto.pose.point.y = worldT[1]
    novoObjeto.pose.point.z = worldT[2]
    
    #quat do objeto
    novoObjeto.pose.orientation.x = quat[0]
    novoObjeto.pose.orientation.y = quat[1]
    novoObjeto.pose.orientation.z = quat[2]
    novoObjeto.pose.orientation.w = quat[3]

    objetos.append(novoObjeto)

def recebeOdometria(msg):
    global position, rotation

    position[0] = msg.pose.position.x
    position[1] = msg.pose.position.y
    position[2] = msg.pose.position.z

    r = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])
    rotation = r.as_dcm()


# Handler da função retorna serviço GetObject
def entregaListaObjetos(req):

    lista = [] #Lista com objetos que atendem o pedido

    # Monta lista com objetos com mesmo estado e tipo requisitados
    for i in objetos
        if (req.identifier.state.data == i.identifier.state.data and req.identifier.type.data == i.identifier.type.data):
            lista.append(i)

    # Tópico 3: http://wiki.ros.org/rospy/Overview/Services
    # Tópico 1: http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29

    return GetObjectResponse(lista)

if __name__ == '__main__':
    try:

        # Inicia nó com nome 'mapa'
        rospy.init_node('mapa', anonymous="True")

        # Subscriber para receber objeto por mensagem
        rospy.Subscriber('objeto_detectado', Object, recebeObjeto)

        # Subscriber para receber odometria
        rospy.Subscriber('/uav1/odometry/uav_state', UavState, recebeOdometria)

        # Serviço GetObject para entregar objetos
        rospy.Service('get_object', GetObject, entregaListaObjetos)

        # Define a frequência de execução em Hz
        rate = rospy.Rate(10)

        # Checa por mensagem ou serviço
        while not rospy.is_shutdown():

            # Espera o tempo para executar o programa na frequência definida
            rate.sleep()

    except rospy.ROSInternalException:
        pass