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

pontoReal = np.array([[-3.588e-2,-3.588e-2,0],
                    [-3.588e-2,3.588e-2,0],
                    [3.588e-2,3.588e-2,0],
                    [3.588e-2,-3.588e-2,0]], dtype=np.float32)

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

def recebeImagem(msg):
    """!
        Recebe uma mensagem de imagem, extrai os QR codes e publica

        Utiliza o pacote pyzbar para extrair os QR codes, 
        e com as bordas utiliza o solvePnP para estimar a posição

        Parâmetros:
            @param msg (Image) - mensagem da imagem
        
    """
    
    global pontoReal

    img = converteImagem(msg)

    data = decode(img)

    if data == []:
        return

    #rospy.loginfo("Pacotes detectados: "+str(len(data)))

    for qr in data:
        if len(qr.polygon)!= 4:
            continue

        letra = qr.data


        pontoImagem = []
        for i in range(4):
            pontoImagem.append([qr.polygon[i][0],qr.polygon[i][1]])
        pontoImagem = np.array(pontoImagem, dtype=np.float32)

        a = 0
        RCamObj = []
        tCamObj = []

        if(cv.__version__[0] == "4"):
            a, RCamObj, tCamObj, _ = cv.solvePnPRansac(pontoReal,pontoImagem , K, np.zeros((5,1)))
        else:
            pontoI = np.expand_dims(pontoImagem, 1)
            pontoR = np.expand_dims(pontoReal, 1)
            a, RCamObj, tCamObj, _ = cv.solvePnPRansac(pontoR,pontoI , K, np.zeros((5,1), dtype=np.float32))

        tCamObj = tCamObj.ravel()

        RCamObj, _ = cv.Rodrigues(RCamObj)

        publicaCubo(RCamObj, tCamObj, letra)

def publicaCubo(R, t, letra):
    """!
        Publica o QR code

        Recebe a posição e dados (letra) dele para criar e publicar o objeto detectado

        @todo Criar um módulo com essa função de apoio e generalizar ela

        Parâmetros:
            @param R (list/np.darray 3x3) - matriz de rotação
            @param t (list/np.darray 3x1) - vetor de translação
    """
    
    msg = Object()

    msg.identifier.data = letra
    msg.identifier.type.data = Identifier.TYPE_PACOTE
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

    pubCubo.publish(msg)

if __name__ == '__main__':
    try:

        rospy.init_node('detecta_cubo', anonymous="True")

        rospy.Subscriber('image_rect_color', Image, recebeImagem)
        rospy.Subscriber('camera_info', CameraInfo, recebeInfo)

        pubCubo = rospy.Publisher('objeto_detectado',Object, queue_size=10)

        #Inscricao para publicar/receber aqui!

        rate = rospy.Rate(10) #Define a frequencia de execucao em Hz

    

        while not rospy.is_shutdown():

            rate.sleep() #Espera o tempo para executar o programa na frequencia definida

    except rospy.ROSInternalException:
        pass
