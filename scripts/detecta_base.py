#!/usr/bin/env python

import rospy
import rospkg

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo


from ger_drone.msg import Object
from ger_drone.msg import Identifier

from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
from pyzbar.pyzbar import decode
import numpy as np
from scipy.spatial.transform import Rotation

K = [[0,0,0],[0,0,0],[0,0,0]]

pontoReal = np.array([[0.5,-0.5,0],
                    [0.5,0.5,0],
                    [-0.5,0.5,0],
                    [-0.5,-0.5,0]], dtype=np.float32)


imgDraw = np.zeros((3,3,1))


def imprimeTodosQuadrados(quadrados, img):
    """!
        Desenha em uma imagem previamente analisada os círculos indicando os
        quadrados observados.
        Parâmetros:
            @param quadrados(int32) - coordenadas dos vértices dos quadrados na imagem.
            @param img(np.array) - imagem lida pela câmera
        
    """
    
    global imgDraw
    imgDraw = img.copy()
    for quad in quadrados:
        for point in quad:
            cv.circle(imgDraw, (point[0], point[1]), 5, (255,0,255), -1)

def converteImagem(img):
    """!
        Converte uma imagem fornecida pelo ROS para o formato do OpenCV

        Parâmetros:
            @param img (np.array) - Imagem fornecida pelo ROS a ser convertida.
        Retorno:
            @returns Imagem no formato do OpenCV.
    """
    
    return CvBridge().imgmsg_to_cv2(img, "bgr8")

def inverteTransformacao(R, t):
    """!
        ##ELTON##
    """
    
    RInverso = np.transpose(R)

    tInverso  = - np.matmul(RInverso, t)

    return RInverso, tInverso


def recebeInfo(msg):
    """!
        ##ELTON##
    """
    

    global K

    index = 0

    for i in range(3):
        for j in range(3):
            K[i][j] = msg.K[index]
            index += 1

    K = np.array(K,dtype=np.float32)

def errorHandler(status, func_name, err_msg, dile_name, line, user_data):
    pass


def procuraQuadrado(mascara):
    """!
        Função que busca, identifica, localiza e define as coordenadas
        de um quadrado caso seja identificado em uma figura.
        Define os contornos presentes na imagem, localiza os vértices dos
        contornos, desenvolve curvas ligando tais vértices (cv.approxPoly)
        e verifica a similaridade com uma reta. Caso todos os vértices ligados
        correspondam ao formato desejado, extrai as coordenadas de tais vértices
        e aponta-os como o quadrado.
        Parâmetros:
            @param mascara(np.array) - imagem previamente filtrada.
        Retorno:
            @returns Coordenadas dos vértices do quadrado obtido.
    """
        
    kernel = np.ones((5,5),np.uint8)
    
    bordas = cv.Canny(mascara, 100, 500, kernel)

    contours = []
    hierarchy = []

    if(cv.__version__[0] == "4"):
        contours,hierarchy = cv.findContours(bordas, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    else:
        _, contours, hierarchy = cv.findContours(bordas, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    quadrados = []

    for i in range(len(contours)):
        epsilon = 0.1*cv.arcLength(contours[i],True)
        approx = cv.approxPolyDP(contours[i],epsilon,True)

        if(len(approx) < 4):
            continue

        if not cv.isContourConvex(approx):
            continue

        if cv.contourArea(approx) < 1000:
            continue

        quadrado = []

        for point in approx:
            quadrado.append([point[0][0], point[0][1]])
        
        quadrados.append(quadrado)

    quadrados = np.array(quadrados)

    i = 0

    while(len(quadrados)-2 >= i):
        if np.linalg.norm(quadrados[i+1][0]-quadrados[i][0]) < 5:
            quadrados = np.delete(quadrados, i, 0)
            
        i+=1

    i = 0

    while len(quadrados) > i:
        if np.linalg.norm(quadrados[i][0]-quadrados[i][1])/np.linalg.norm(quadrados[i][2]-quadrados[i][1]) <0.9:
           quadrados = np.delete(quadrados, i, 0) 
        i+=1

    i = 0

    while(len(quadrados) > i):
        

        mask = np.zeros(mascara.shape,np.uint8)
        cv.drawContours(mask,[quadrados[i]],0,255,-1)
        pixelpoints = np.transpose(np.nonzero(mask))


        zero = 0
        naoZero = 0

        for j in range(len(pixelpoints)):
            if mask[pixelpoints[j, 0], pixelpoints[j,1]] == 0:
                zero+=1
            else:
                naoZero+=1

        razao = float(zero)/float(naoZero)

        #print(razao)

        if(razao>0.05):
            quadrados = np.delete(quadrados, i, 0)

        i+=1

    return quadrados


def recebeImagem(msg):
    """!
        Realiza a filtragem de uma imagem para cores pré-definidas; localiza os
        vértices dos quadrados (se existirem); verifica as coordenadas do quadrado
        relativa ao centro da imagem; estima a pose do quadrado.
        Parâmetro:
            @param msg ##ELTON (VERIFICAR)##
        
    """
    
    img = converteImagem(msg)

    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    kernel = np.ones((5,5),"uint8")

    #Filtragem da cor azul
    min_azul = np.array([115,60,50], np.uint8)
    max_azul = np.array([125,255,255], np.uint8)
    masc_azul = cv.inRange(hsv,min_azul,max_azul)
    masc_azul = cv.dilate(masc_azul,kernel)
    masc_azul = cv.dilate(masc_azul,kernel)


    min_amarelo = np.array([25,40,200], np.uint8)
    max_amarelo = np.array([35,255,255], np.uint8)
    masc_amarelo = cv.inRange(hsv,min_amarelo,max_amarelo)
    masc_amarelo = cv.dilate(masc_amarelo,kernel)


    masc = masc_azul

    kernel = np.ones((5,5),np.uint8)


    quadrados = procuraQuadrado(masc)


    quadrados = np.array(quadrados, dtype=np.float32)

    if(len(quadrados) != 0):
        rospy.loginfo("Bases encontradas: "+str(len(quadrados)))
        #imprimeTodosQuadrados(quadrados, img)


    for pontoImagem in quadrados:
        global pontoReal

        a = 0
        RCamObj = []
        tCamObj = []

        if(cv.__version__[0] == "4"):
            a, RCamObj, tCamObj, _ = cv.solvePnPRansac(pontoReal,pontoImagem , K, np.zeros((5,1)))
        else:
            pontoI = np.expand_dims(pontoImagem, 1)
            pontoR = np.expand_dims(pontoReal, 1)
            rospy.loginfo(str(pontoI.shape)+" "+str(pontoR.shape))
            a, RCamObj, tCamObj, _ = cv.solvePnPRansac(pontoR,pontoI , K, np.zeros((5,1), dtype=np.float32))

        tCamObj = tCamObj.ravel()

        RCamObj, _ = cv.Rodrigues(RCamObj)

        publicaBase(RCamObj, tCamObj)


def publicaBase(R, t):
    """!
        ##ELTON##
    """
    msg = Object()

    msg.identifier.type.data = Identifier.TYPE_BASE

    msg.identifier.state.data = msg.identifier.STATE_DESCONHECIDO

    msg.pose.position.x = t[0]
    msg.pose.position.y = t[1]
    msg.pose.position.z = t[2]
    
    msg.identifier.index.data = -1

    r = Rotation.from_dcm(R)

    quat = r.as_quat()

    msg.pose.orientation.x = quat[0]
    msg.pose.orientation.y = quat[1]
    msg.pose.orientation.z = quat[2]
    msg.pose.orientation.w = quat[3]

    pubBase.publish(msg)

if __name__ == '__main__':
    try:

        rospy.init_node('detecta_base', anonymous="True")

        rospy.Subscriber('image_rect_color', Image, recebeImagem)
        rospy.Subscriber('camera_info', CameraInfo, recebeInfo)

        pubBase = rospy.Publisher('objeto_detectado',Object, queue_size=10)

        #Inscricao para publicar/receber aqui!

        rate = rospy.Rate(10) #Define a frequencia de execucao em Hz

    

        while not rospy.is_shutdown():

            #cv.imshow("Detector", imgDraw)
            #cv.waitKey(1)


            rate.sleep() #Espera o tempo para executar o programa na frequencia definida

    except rospy.ROSInternalException:
        pass
