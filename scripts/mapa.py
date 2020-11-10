#!/usr/bin/env python

import rospy
import rospkg

import numpy as np
from scipy.spatial.transform import Rotation
from matplotlib import pyplot as plt
import cv2 as cv

from mrs_msgs.msg import UavState, PositionCommand
from ger_drone.msg import Object, Identifier, ObjectState

from ger_drone.srv import GetObject, GetObjectResponse
from std_srvs.srv import SetBool, SetBoolResponse

import csv


num = 1.0
nBase = 5
nSensor = 5
nIteracao = 10

# Lista de bases inicialmente vazia
objetos = []

# Vetor de posicoes para calculo de coordenadas
tDroneWorld = np.asarray([0.0,0.0,0.0])
RDroneWorld = np.asarray([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
muitoRapido = True

indiceMaximo = -1


tCameraDrone = [0.050, 0.0, -0.093]
RCameraDrone = [0.707, -0.707, 0.0, -0.0]

RCameraDrone = Rotation.from_quat(RCameraDrone).as_dcm()



# Cria e insere objeto da mensagem na lista
def recebeObjeto(msg):

    if atualizaMapa == False:
        return

    if(muitoRapido == True):
        return
        

    # Posicao do obj nas coordenadas da camera
    tObjCamera = np.asarray([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    RObjCamera = Rotation.from_quat([msg.pose.orientation.x, msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]).as_dcm()

    # Concatena com a pose da camera no drone
    RObjDrone = np.matmul(RCameraDrone, RObjCamera)
    tObjDrone = tCameraDrone + np.matmul(RCameraDrone, tObjCamera)

    #Concatena com a pose do drone no mundo
    RObjWorld = np.matmul(RDroneWorld, RObjDrone) 
    tObjWorld = tDroneWorld + np.matmul(RDroneWorld, tObjDrone)

    #rospy.loginfo("Pose: "+str(tObjWorld[0])+" "+str(tObjWorld[1])+" "+str(tObjWorld[2]))

    #if(np.linalg.norm(tObjWorld[:2] - tDroneWorld[:2]) > 1.2):
     #   return
        

    if(tObjWorld[0] > 6.5 or tObjWorld[0] < -0.45):
        return
    if(tObjWorld[1] <-6.8 or tObjWorld[1] > 0.5):
        return
    
    if(tObjWorld[0]<1 and tObjWorld[1] > -1.5):
        return



    # Se objeto retornar com indice != -1, verificar se o tipo bate com o objeto ja na lista, e atualizar o estado
    if(msg.identifier.index.data != -1):
        for i in objetos:
            if(i.identifier.index.data == msg.identifier.index.data):
                if(i.identifier.type.data == msg.identifier.type.data):
                    i.identifier.state.data = msg.identifier.state.data
                return


    r = Rotation.from_dcm(RObjWorld)
    quat = r.as_quat()

    if(msg.identifier.state.data == Identifier.STATE_DESCONHECIDO):
        estado = Identifier.STATE_NOPROCESSADO
    else:
        estado = msg.identifier.state.data

    novoObjeto = Object()
    
    #rospy.loginfo('Objeto armazenado.')
    
    #tipo do objeto
    novoObjeto.identifier.type.data = msg.identifier.type.data
    
    #estado do objeto
    novoObjeto.identifier.state.data = estado
    
    #pose do objeto
    novoObjeto.pose.position.x = tObjWorld[0]
    novoObjeto.pose.position.y = tObjWorld[1]
    novoObjeto.pose.position.z = tObjWorld[2]
    
    #quat do objeto
    novoObjeto.pose.orientation.x = quat[0]
    novoObjeto.pose.orientation.y = quat[1]
    novoObjeto.pose.orientation.z = quat[2]
    novoObjeto.pose.orientation.w = quat[3]

    novoObjeto.identifier.data = msg.identifier.data

    objetos.append(novoObjeto)
    #logObjetos()


def recebeOdometria(msg):
    global tDroneWorld, RDroneWorld, muitoRapido


    tDroneWorld[0] = msg.pose.position.x
    tDroneWorld[1] = msg.pose.position.y
    tDroneWorld[2] = msg.pose.position.z

    r = Rotation.from_quat([msg.pose.orientation.x, msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])
    RDroneWorld = r.as_dcm()

    if((abs(msg.velocity.linear.x) > 0.05 or abs(msg.velocity.linear.y) > 0.05) or
        (abs(msg.acceleration.linear.x) > 0.4 or abs(msg.acceleration.linear.y) > 0.4) or
        (abs(msg.velocity.angular.x) > 0.07 or abs(msg.velocity.angular.y) > 0.07)):
        muitoRapido = True
    else:
        muitoRapido = False



def logObjetos():
    log = str(len(objetos))
    log = log +" Objetos armazenados. \n"

    for obj in objetos:
        log = log + "["+str(obj.identifier.index.data)+"]: "+str(obj.identifier.type.data)
        log = log + " x: "+str(obj.pose.position.x)+" y: "+str(obj.pose.position.y)+" z: "+str(obj.pose.position.z)
        log = log + "\n"

    rospy.loginfo(log)

def afinaLista():
    global objetos, atualizaMapa

    atualizaMapa = False

    rospy.loginfo("Refinando lista de objetos")

    novaLista = []

    bases = []
    sensores = []
    cubos = []

    for i in range(len(objetos)):
        tipo = objetos[i].identifier.type.data

        if tipo == Identifier.TYPE_BASE:
            bases.append(objetos[i])
        elif (tipo == Identifier.TYPE_SENSOR_VERDE or 
        tipo == Identifier.TYPE_SENSOR_VERMELHO):
            sensores.append(objetos[i])
        elif (tipo == Identifier.TYPE_PACOTE_A or tipo == Identifier.TYPE_PACOTE_B 
            or tipo == Identifier.TYPE_PACOTE_C or tipo == Identifier.TYPE_PACOTE_D 
            or Identifier.TYPE_PACOTE_E):
            cubos.append(objetos[i])

    if len(bases) != 0:

        for i in range(nBase):
            if len(bases) == 0:
                break

            nInlierMelhor = 0
            melhor = 0
            poseMediaMelhor = [0,0]

            for j in range(nIteracao):
                index = int(np.random.rand(1)[0]*len(bases))
                

                pose = np.array([bases[index].pose.position.x,bases[index].pose.position.y])
                nInlier =0
                poseMedia = [0,0]

                for k in range(len(bases)):
                    poseTeste = np.array([bases[k].pose.position.x,bases[k].pose.position.y])
                    if(np.linalg.norm(pose-poseTeste) < 1.0):
                        nInlier += 1
                        poseMedia[0] += poseTeste[0]
                        poseMedia[1] += poseTeste[1]

                if(nInlier > nInlierMelhor):
                    nInlierMelhor = nInlier
                    melhor = index

                    poseMediaMelhor[0] = poseMedia[0] / nInlier
                    poseMediaMelhor[1] = poseMedia[1] / nInlier


            pose = np.array([bases[melhor].pose.position.x,bases[melhor].pose.position.y])
            
            bases[melhor].pose.position.x = poseMediaMelhor[0]
            bases[melhor].pose.position.y = poseMediaMelhor[1]

            novaLista.append(bases[melhor])
            
            k = 0

            while(k < len(bases)):
                poseTeste = np.array([bases[k].pose.position.x,bases[k].pose.position.y])
                if(np.linalg.norm(pose- poseTeste) < 1.0):
                    del bases[k]
                else:
                    k += 1

    nSensor = len(sensores)
    nSensorEscolhido = 0

    if nSensor != 0:

        melhorErro = 999999999999
        pontos = [[2.68,0],[3.87,-6.3]]

        dx = pontos[0][0] - pontos[1][0]
        dy = pontos[0][1] - pontos[1][1]
        det = (dx*dx) + (dy*dy)
        
        for i in range(len(sensores)):
            a = (dy*(sensores[i].pose.position.y-pontos[0][1])+dx*(sensores[i].pose.position.x-pontos[0][0]))/det

            sensores[i].pose.position.x = (a*dx) + pontos[0][0]
            sensores[i].pose.position.y = (a*dy) + pontos[0][1]

        '''for i in range(10):
            index1 = int(np.random.rand(1)[0]*len(sensores))
            index2 = int(np.random.rand(1)[0]*len(sensores))

            p1 = np.array([sensores[index1].pose.position.x,sensores[index1].pose.position.y])
            p2 = np.array([sensores[index2].pose.position.x,sensores[index2].pose.position.y])

            erro = 0

            for j in range(len(sensores)):
                p3 = np.array([sensores[j].pose.position.x,sensores[j].pose.position.y])
                d = np.linalg.norm(np.cross(p2-p1, p1-p3))/np.linalg.norm(p2-p1)

                erro += d

            if(erro< melhorErro):
                melhorErro = erro
                pontos[0] = p1
                pontos[1] = p2
        '''


        while True:
        #for i in range(nSensor):
            if len(sensores) == 0:
                break

            nInlierMelhor = 0
            melhor = 0
            poseMediaMelhor = [0,0]

            for j in range(nIteracao):
                index = int(np.random.rand(1)[0]*len(sensores))
                

                pose = np.array([sensores[index].pose.position.x,sensores[index].pose.position.y])
                nInlier =0
                poseMedia = [0,0]

                for k in range(len(sensores)):
                    if (sensores[k].identifier.type.data != sensores[index].identifier.type.data):
                        continue
                    poseTeste = np.array([sensores[k].pose.position.x,sensores[k].pose.position.y])
                    if(np.linalg.norm(pose-poseTeste) < 0.05):
                        nInlier += 1
                        poseMedia[0] += poseTeste[0]
                        poseMedia[1] += poseTeste[1]

                if(nInlier > nInlierMelhor):
                    nInlierMelhor = nInlier
                    melhor = index

                    poseMediaMelhor[0] = poseMedia[0] / nInlier
                    poseMediaMelhor[1] = poseMedia[1] / nInlier

            if (nSensorEscolhido >= 5) and ((float(nInlierMelhor )/ float(nSensor)) < 0.28):
                rospy.loginfo(str(nInlierMelhor)+" "+str(len(sensores))+" "+str(nSensor))
                break
            
            nSensorEscolhido += 1


            pose = np.array([sensores[melhor].pose.position.x,sensores[melhor].pose.position.y])
            
            dx = pontos[0][0] - pontos[1][0]
            dy = pontos[0][1] - pontos[1][1]
            det = (dx*dx) + (dy*dy)
            a = (dy*(poseMediaMelhor[1]-pontos[0][1])+dx*(poseMediaMelhor[0]-pontos[0][0]))/det

            poseMediaMelhor[0] = (a*dx) + pontos[0][0]
            poseMediaMelhor[1] = (a*dy) + pontos[0][1]

            sensores[melhor].pose.position.x = poseMediaMelhor[0]
            sensores[melhor].pose.position.y = poseMediaMelhor[1]

            tipo = sensores[melhor].identifier.type.data

            novaLista.append(sensores[melhor])
            
            k = 0

            while(k < len(sensores)):
                if (tipo != sensores[k].identifier.type.data):
                    k+=1
                    continue
                poseTeste = np.array([sensores[k].pose.position.x,sensores[k].pose.position.y])
                if(np.linalg.norm(pose- poseTeste) < 0.05):
                    del sensores[k]
                else:
                    k += 1

    nCubos = len(cubos)

    if nCubos != 0:
        while True:
            if len(cubos) == 0:
                break

            nInlierMelhor = 0
            melhor = 0
            poseMediaMelhor = [0,0]

            for j in range(nIteracao):
                index = int(np.random.rand(1)[0]*len(cubos))
                

                pose = np.array([cubos[index].pose.position.x,cubos[index].pose.position.y])
                nInlier =0
                poseMedia = [0,0]

                for k in range(len(cubos)):
                    poseTeste = np.array([cubos[k].pose.position.x,cubos[k].pose.position.y])
                    if(np.linalg.norm(pose-poseTeste) < 1.0):
                        nInlier += 1
                        poseMedia[0] += poseTeste[0]
                        poseMedia[1] += poseTeste[1]

                if(nInlier > nInlierMelhor):
                    nInlierMelhor = nInlier
                    melhor = index

                    poseMediaMelhor[0] = poseMedia[0] / nInlier
                    poseMediaMelhor[1] = poseMedia[1] / nInlier

            if ((float(nInlierMelhor )/ float(nCubos)) < 0.2):
                break

            pose = np.array([cubos[melhor].pose.position.x,cubos[melhor].pose.position.y])
            
            cubos[melhor].pose.position.x = poseMediaMelhor[0]
            cubos[melhor].pose.position.y = poseMediaMelhor[1]

            novaLista.append(cubos[melhor])
            
            k = 0

            while(k < len(cubos)):
                poseTeste = np.array([cubos[k].pose.position.x,cubos[k].pose.position.y])
                if(np.linalg.norm(pose- poseTeste) < 1.0):
                    del cubos[k]
                else:
                    k += 1

    objetos = novaLista
    
    imprimeMapa()




# Handler da funcao retorna servico GetObject
def entregaListaObjetos(req):

    afinaLista()

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
    rospy.loginfo("Gerando arquivo")
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
            str(objeto.pose.orientation.w  ),
            str(objeto.identifier.data)]
    
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
                novoObjeto.identifier.data = row[10]

                # Adiciona o objeto na lista global "objetos"
                objetos.append(novoObjeto)

        rospy.loginfo("Arquivo lido com sucesso, "+str(len(objetos))+" objetos armazenados")
    except:
        rospy.logwarn("Nao foi possivel ler o arquivo")
        return

def setAtualizaMapa(req):
    global atualizaMapa
    if req.data == True:
        atualizaMapa = True
    else:
        atualizaMapa = False

    resp = SetBoolResponse()
    resp.success = True
    resp.message = "Alterado com sucesso"

def imprimeMapa():

    global imgMapa, atualizaMapa

    imgMapa = np.ones((800,800,3),np.uint8)*255

    for i in range(16):
        cv.line(imgMapa, (i*50,0), (i*50,800), (0,0,0), 1)
        cv.line(imgMapa, (0,i*50), (800, i*50), (0,0,0), 1)

    cv.line(imgMapa, (268+50,0+50),(387+50,630+50), (0,138,129),1)

    cv.circle(imgMapa, (int(tDroneWorld[0]*100)+50, int(-tDroneWorld[1]*100)+50), 4, (255,0,255), -1)

    for obj in objetos:

        cor = [0,0,0]

        posicao = [0, 0]

        if(obj.identifier.type.data == Identifier.TYPE_SENSOR_VERDE):
            cor[1] = 255
        elif(obj.identifier.type.data == Identifier.TYPE_SENSOR_VERMELHO):
            cor[2] = 255
        elif(obj.identifier.type.data == Identifier.TYPE_BASE):
            cor[0] = 255
        else:
            cor[2] = 255


        if obj.identifier.state.data == Identifier.STATE_PROCESSADO:
            cor[0] /= 2
            cor[1] /= 2
            cor[2] /= 2

        posicao[0] = (obj.pose.position.x *100)+50
        posicao[1] = (-obj.pose.position.y*100)+50

        cv.circle(imgMapa, (int(posicao[0]),int(posicao[1])), 3, (cor[0],cor[1],cor[2]), -1)

if __name__ == '__main__':
    try:

        # Inicia no com nome 'mapa'
        rospy.init_node('mapa', anonymous="True")

        escreve = False
        le = False
        imprime = False

        atualizaMapa = True
        
        try:
            escreve = rospy.get_param("~escrever")
        except:
            pass

        try:
            le = rospy.get_param("~ler")
        except:
            pass

        try:
            imprime = rospy.get_param("~imprimir")
        except:
            pass

        rospy.loginfo("Escrita de arquivo definido para: "+str(escreve))
        rospy.loginfo("Leitura de arquivo definido para: "+str(le))
        rospy.loginfo("Impressao definida para: "+str(imprime))

        path = rospkg.RosPack().get_path('ger_drone')

        path = path + "/data/mapa_gerado.csv"
        
        # Inicia a lista de objetos com os contidos no arquivo 
        
        if le == True:
            recuperaArquivo()

        if imprime == True:
            imprimeMapa()

        # Subscriber para receber objeto por mensagem
        rospy.Subscriber('objeto_detectado', Object, recebeObjeto)

        # Subscriber para receber odometria
        rospy.Subscriber('uav_state', UavState, recebeOdometria)
        #rospy.Subscriber('position_cmd', PositionCommand, recebeOdometria)

        # Servico GetObject para entregar objetos
        rospy.Service('get_object', GetObject, entregaListaObjetos)

        rospy.Service('set_atualiza_mapa', SetBool, setAtualizaMapa)


        # Define a frequencia de execucao em Hz
        rate = rospy.Rate(10)
        

        # Checa por mensagem ou servico
        while not rospy.is_shutdown():
            if imprime == True:
                imprimeMapa()
                cv.imshow("Mapa", imgMapa)
                cv.waitKey(1)
            # Espera o tempo para executar o programa na frequencia definida
            rate.sleep()

        #if(escreve == True):
        #    gerarArquivo()
        gerarArquivo()
        cv.destroyAllWindows()
    except rospy.ROSInternalException:
        #if(escreve == True):
            #gerarArquivo()
        gerarArquivo()
        cv.destroyAllWindows()
        pass
