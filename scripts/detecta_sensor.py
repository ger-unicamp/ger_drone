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

pontoReal = np.array([[2.5e-2, -2.5e-2, 0],
                    [2.5e-2, 2.5e-2, 0],
                    [-2.5e-2, 2.5e-2, 0],
                    [-2.5e-2, -2.5e-2, 0]], dtype=np.float32)



def converteImagem(img):
    """!
        Converte uma imagem fornecida pelo ROS para o formato do OpenCV

        @todo Criar um módulo com essa função de apoio

        Parâmetros:
            @param img (np.array) - Imagem fornecida pelo ROS a ser convertida.
        Retorno:
            @returns Imagem no formato do OpenCV.
    """

    return CvBridge().imgmsg_to_cv2(img, "bgr8")


def inverteTransformacao(R, t):
    """!
        Inverte uma transformação geométrica.

        Se a transformação leva do sistema de coordena A para o B, 
        o retorno será do sistema B para o A

        @todo Criar um módulo com essa função de apoio

        Parâmetros:
            @param R (np.darray 3x3) - Matriz de rotação
            @param t (np.darray 3x1) - Vetor de translação

        Retorno:
            @returns RInverso (np.darray 3x3) - Matriz de rotação inversa
            @returns tInverso (np.darray 3x1) - Vetor de translação inverso
            
    """

    RInverso = np.transpose(R)

    tInverso  = - np.matmul(RInverso, t)

    return RInverso, tInverso

#Procura quadrados na imagem usando uma mascara
def procuraQuadrado(mascara):
    '''! 
        Procura quadrados na imagem pré-processada


        Função que busca, identifica, localiza e define as coordenadas
        de um quadrado caso seja identificado em uma figura.
        Define os contornos presentes na imagem, localiza os vértices dos
        contornos, desenvolve curvas ligando tais vértices (cv.approxPoly)
        e verifica a similaridade com uma reta. Caso todos os vértices ligados
        correspondam ao formato desejado, extrai as coordenadas de tais vértices
        e aponta-os como o quadrado.

        Parâmetros:
            @param mascara (np.darray) - imagem pré-processada
    '''

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

        if cv.contourArea(approx) < 100:
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

    while len(quadrados) > i:
        if np.linalg.norm(quadrados[i][0]-quadrados[i][1])/np.linalg.norm(quadrados[i][2]-quadrados[i][1]) <0.9:
           quadrados = np.delete(quadrados, i, 0) 
        i+=1

    i = 0
    
    return quadrados

def estimaPoseSensor(quad):
    '''!
        Estima a pose do sensor

        Utiliza o solvePnP, junto da posição do sensor na imagem e no mundo métrico, 
        para estima a sua posição em relação ao frame da câmera

        Parâmetros:
            @param quad (np.darray) - Posição dos cantos do sensor na imagem

        Retorno:
            @returns RCamObj (np.darray 3x3) - matriz de rotação do sistema da câmera para o objeto
            @returns tCamObj (np.darray 3x1) - vetor de rotação do sistema da câmera para o objeto

    '''

    a = 0
    RCamObj = []
    tCamObj = []

    if(cv.__version__[0] == "4"):
        a, RCamObj, tCamObj, _ = cv.solvePnPRansac(pontoReal,quad , K, np.zeros((5,1)))
    else:
        pontoI = np.expand_dims(quad, 1)
        pontoR = np.expand_dims(pontoReal, 1)
        a, RCamObj, tCamObj, _ = cv.solvePnPRansac(pontoR,pontoI , K, np.zeros((5,1), dtype=np.float32))


    tCamObj = tCamObj.ravel()

    RCamObj, _ = cv.Rodrigues(RCamObj)

    return RCamObj, tCamObj


def recebeImagem(msg):
    """!
        Recebe uma mensagem de imagem, procura os sensores e publica

        Parâmetros:
            @param msg (Image) - mensagem da imagem
    """
    
    img = converteImagem(msg)

    #Calcula as mascaras
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    min_verde = np.array([50,200,150],np.uint8)
    max_verde = np.array([150,255,255],np.uint8)
    masc_verde = cv.inRange(hsv,min_verde,max_verde)

    min_verm = np.array([0,210,150],np.uint8)
    max_verm = np.array([8,255,255],np.uint8)
    masc_verm = cv.inRange(hsv,min_verm,max_verm)


    #Procura os quadrados nas imagens
    quadVerde = procuraQuadrado(masc_verde)
    quadVerm = procuraQuadrado(masc_verm)

    

    #Calcula as poses e publica
    quadVerde = quadVerde.astype(np.float32)

    #imprimeTodosQuadrados(quadVerde, img)

    for quad in quadVerde:
        R, t = estimaPoseSensor(quad)
        publicaSensor(R, t, True)

    quadVerm = quadVerm.astype(np.float32)

    for quad in quadVerm:
        R, t = estimaPoseSensor(quad)
        publicaSensor(R, t, False)

    #rospy.loginfo("Detectados "+str(len(quadVerm)+len(quadVerde))+" sensores")


def publicaSensor(R, t, cor):
    """!
        Publica o sensor detectado

        Recebe a posição e dados (cor) dele para criar e publicar o objeto detectado

        Parâmetros:
            @param R (list/np.darray 3x3) - matriz de rotação
            @param t (list/np.darray 3x1) - vetor de translação
            @param cor - True se verde, False se vermelho
    """

    msg = Object()

    if cor == True:
        msg.identifier.type.data = msg.identifier.TYPE_SENSOR_VERDE
    else:
        msg.identifier.type.data = msg.identifier.TYPE_SENSOR_VERMELHO
    

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

    pubSensor.publish(msg)

def recebeInfo(msg):
    """!
        Recebe a mensagem com as informações da câmera
        
        Extrai apenas a matriz de calibração

        @todo Criar um módulo com essa função de apoio

        Parâmetros:
            @param msg (CameraInfo) - informações da câmera
    """  

    global K

    index = 0

    for i in range(3):
        for j in range(3):
            K[i][j] = msg.K[index]
            index += 1

    K = np.array(K,dtype=np.float32)

def imprimeTodosQuadrados(quadrados, img):
    '''!
        Desenha os cantos quadrados na imagem e armazena em imgDraw

        @todo Possivelmente remover essa função

        Parâmetros:
            @param quadrados (list/np.darray nx4x2) - Lista com cantos dos quadrados
            @param img - imagem onde deverá ser desenhado os cantos
    '''

    global imgDraw
    imgDraw = img.copy()
    for quad in quadrados:
        for point in quad:
            cv.circle(imgDraw, (point[0], point[1]), 5, (255,0,255), -1)

if __name__ == '__main__':
    try:

        rospy.init_node('detecta_sensor', anonymous="True")

        rospy.Subscriber('image_rect_color', Image, recebeImagem)
        rospy.Subscriber('camera_info', CameraInfo, recebeInfo)

        pubSensor = rospy.Publisher('objeto_detectado',Object, queue_size=10)

        #Inscricao para publicar/receber aqui!

        rate = rospy.Rate(10) #Define a frequencia de execucao em Hz

        imgDraw = np.zeros((700, 700))
    

        while not rospy.is_shutdown():
            rate.sleep() #Espera o tempo para executar o programa na frequencia definida

            #cv.imshow("Mapa", imgDraw)
            #cv.waitKey(1)
        #cv.destroyAllWindows()
    except rospy.ROSInternalException:

        #cv.destroyAllWindows()
        pass
