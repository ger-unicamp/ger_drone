#!/usr/bin/env python

import rospy
import rospkg

import numpy as np
from scipy.spatial.transform import Rotation as R
from matplotlib import pyplot as plt

from mrs_msgs.msg import UavState
from ger_drone.msg import Object, Identifier, ObjectState

from ger_drone.srv import GetObject, GetObjectResponse

import csv

# Lista de bases inicialmente vazia
objetos = []

# Vetor de posicoes para calculo de coordenadas
position = np.asarray([0,0,0])
rotation = np.asarray([[1,0,0],[0,1,0],[0,0,1]])

indiceMaximo = -1

num = 1

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

    # Verifica se um objeto com indice -1 e o mesmo que outro objeto que ja esta na lista
    if(msg.identifier.index.data == -1):
        # comparar a pose se esta proxima (intervalo)
        # @todo distancia euclidiana numpy.linalg.norm
        for i in objetos:
            if (i.identifier.type.data == msg.identifier.type.data):
                if (abs(i.pose.position.x - worldT[0]) < num and 
                abs(i.pose.position.y - worldT[1]) < num and 
                abs(i.pose.position.z - worldT[2]) < num):
                    return

    # Se objeto retornar com indice != -1, verificar se o tipo bate com o objeto ja na lista, e atualizar o estado
    if(msg.identifier.index.data != -1):
        for i in objetos:
            if(i.identifier.index.data == msg.identifier.index.data):
                if(i.identifier.type.data == msg.identifier.type.data):
                    i.identifier.state.data = msg.identifier.state.data
                return

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
    novoObjeto.pose.position.x = worldT[0]
    novoObjeto.pose.position.y = worldT[1]
    novoObjeto.pose.position.z = worldT[2]
    
    #quat do objeto
    novoObjeto.pose.orientation.x = quat[0]
    novoObjeto.pose.orientation.y = quat[1]
    novoObjeto.pose.orientation.z = quat[2]
    novoObjeto.pose.orientation.w = quat[3]

    objetos.append(novoObjeto)
    logObjetos()

def recebeOdometria(msg):
    global position, rotation

    position[0] = msg.pose.position.x
    position[1] = msg.pose.position.y
    position[2] = msg.pose.position.z

    r = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])
    rotation = r.as_dcm()

def logObjetos():
    log = str(len(objetos))
    log = log +" Objetos armazenados. \n"

    for obj in objetos:
        log = log + "["+str(obj.identifier.index.data)+"]: "+str(obj.identifier.type.data)
        log = log + " x: "+str(obj.pose.position.x)+" y: "+str(obj.pose.position.y)+" z: "+str(obj.pose.position.z)
        log = log + "\n"

    rospy.loginfo(log)

# Handler da funcao retorna servico GetObject
def entregaListaObjetos(req):

    lista = [] #Lista com objetos que atendem o pedido

    # Monta lista com objetos com mesmo estado e tipo requisitados
    for i in objetos:
        if (req.identifier.state.data == i.identifier.state.data and req.identifier.type.data == i.identifier.type.data):
            lista.append(i)

    # Topico 3: http://wiki.ros.org/rospy/Overview/Services
    # Topico 1: http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29

    return GetObjectResponse(lista)

def gerarArquivo():
    # Gera um arquivo com todos os objetos na lista.
    # Atencao! tudo que ouver no arquivo "mapa_gerado" sera substituido.
    
    with open(path, 'w') as arquivo:
        writer = csv.writer(arquivo, delimiter=',')

        for obj in objetos:
            row = geraLinhaCSV(obj)

            writer.writerow(row)


def geraLinhaCSV(objeto):
    row =[str(objeto.identifier.index.data),
            str(objeto.identifier.type.data ),
            str(objeto.identifier.state.data ),
            str(objeto.pose.position.x ),
            str(objeto.pose.position.y ),
            str(objeto.pose.position.z  ),
            str(objeto.pose.orientation.x ),
            str(objeto.pose.orientation.y ),
            str(objeto.pose.orientation.z ),
            str(objeto.pose.orientation.w  )]
    
    return row

def appendObjeto(objeto):
    with open(path, 'a') as arquivo:
        writer = csv.writer(arquivo, delimiter=',')

        row = geraLinhaCSV(objeto)

        writer.writerow(row)
    
            

def recuperaArquivo():
    # Gera uma lista de objetos com os dados de "mapa_gerado".
    
    try:
        with open(path, 'r') as arquivo:
            reader = csv.reader(arquivo, delimiter=',')

            arquivo.seek(0)

            for row in reader:
                novoObjeto = Object()

                novoObjeto.identifier.index.data = int(row[0])
                novoObjeto.identifier.type.data = int(row[1])
                novoObjeto.identifier.state.data = int(row[2])
                novoObjeto.pose.position.x = float(row[3])
                novoObjeto.pose.position.y = float(row[4])
                novoObjeto.pose.position.z = float(row[5])
                novoObjeto.pose.orientation.x = float(row[6])
                novoObjeto.pose.orientation.y = float(row[7])
                novoObjeto.pose.orientation.z = float(row[8])
                novoObjeto.pose.orientation.w = float(row[9])

                # Adiciona o objeto na lista global "objetos"
                objetos.append(novoObjeto)

        rospy.loginfo("Arquivo lido com sucesso, "+str(len(objetos))+" objetos armazenados")
    except:
        rospy.logwarn("Nao foi possivel ler o arquivo")
        return

if __name__ == '__main__':
    try:

        # Inicia no com nome 'mapa'
        rospy.init_node('mapa', anonymous="True")

        escreve = False
        le = False
        
        try:
            escreve = rospy.get_param("~escrever")
        except:
            pass

        try:
            le = rospy.get_param("~ler")
        except:
            pass

        rospy.loginfo("Escrita de arquivo definido para: "+str(escreve))
        rospy.loginfo("Leitura de arquivo definido para: "+str(le))

        path = rospkg.RosPack().get_path('ger_drone')

        path = path + "/data/mapa_gerado.csv"
        
        # Inicia a lista de objetos com os contidos no arquivo 
        
        if le == True:
            recuperaArquivo()

        # Subscriber para receber objeto por mensagem
        rospy.Subscriber('objeto_detectado', Object, recebeObjeto)

        # Subscriber para receber odometria
        rospy.Subscriber('uav_state', UavState, recebeOdometria)

        # Servico GetObject para entregar objetos
        rospy.Service('get_object', GetObject, entregaListaObjetos)

        # Define a frequencia de execucao em Hz
        rate = rospy.Rate(10)
        
        
        # Checa por mensagem ou servico
        while not rospy.is_shutdown():

            # Espera o tempo para executar o programa na frequencia definida
            rate.sleep()

        if(escreve == True):
            gerarArquivo()

    except rospy.ROSInternalException:
        if(escreve == True):
            gerarArquivo()

        pass
