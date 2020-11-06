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
    return CvBridge().imgmsg_to_cv2(img)


def inverteTransformacao(R, t):
    RInverso = np.transpose(R)

    tInverso  = - np.matmul(RInverso, t)

    return RInverso, tInverso

#Procura quadrados na imagem usando uma mascara
def procuraQuadrado(mascara):
    kernel = np.ones((5,5),np.uint8)
    
    bordas = cv.Canny(mascara, 100, 500, kernel)
    contours,hierarchy = cv.findContours(bordas, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)

    quadrados = []

    for i in range(len(contours)):
        epsilon = 0.1*cv.arcLength(contours[i],True)
        approx = cv.approxPolyDP(contours[i],epsilon,True)

        if(len(approx) < 4):
            continue

        if not cv.isContourConvex(approx):
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
    
    return quadrados

#Estima a pose do sensor a partir de seus cantos na imagem
def estimaPoseSensor(quad):


    a, RCamObj, tCamObj, _ = cv.solvePnPRansac(pontoReal,quad , K, np.zeros((5,1)))

    tCamObj = tCamObj.ravel()

    RCamObj, _ = cv.Rodrigues(RCamObj)

    RObjCamera, tObjCamera = inverteTransformacao(RCamObj, tCamObj)

    return RObjCamera, tObjCamera


#Recebe a imagem, procura os quadrados e publica
def recebeImagem(msg):
    img = converteImagem(msg)

    #Calcula as mascaras
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    min_verde = np.array([50,200,0],np.uint8)
    max_verde = np.array([150,255,255],np.uint8)
    masc_verde = cv.inRange(hsv,min_verde,max_verde)

    min_verm = np.array([0,210,100],np.uint8)
    max_verm = np.array([8,255,255],np.uint8)
    masc_verm = cv.inRange(hsv,min_verm,max_verm)


    #Procura os quadrados nas imagens
    quadVerde = procuraQuadrado(masc_verde)
    quadVerm = procuraQuadrado(masc_verm)


    #Calcula as poses e publica
    quadVerde = quadVerde.astype(np.float32)

    for quad in quadVerde:
        R, t = estimaPoseSensor(quad)
        publicaSensor(R, t, True)

    quadVerm = quadVerm.astype(np.float32)

    for quad in quadVerm:
        R, t = estimaPoseSensor(quad)
        publicaSensor(R, t, False)

    #rospy.loginfo("Detectados "+str(len(quadVerm)+len(quadVerde))+" sensores")

#@param cor - True se verde, False se vermelho
def publicaSensor(R, t, cor):
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

    global K

    index = 0

    for i in range(3):
        for j in range(3):
            K[i][j] = msg.K[index]
            index += 1

    K = np.array(K,dtype=np.float32)

if __name__ == '__main__':
    try:

        rospy.init_node('detecta_sensor', anonymous="True")

        rospy.Subscriber('image_rect_color', Image, recebeImagem)
        rospy.Subscriber('camera_info', CameraInfo, recebeInfo)

        pubSensor = rospy.Publisher('objeto_detectado',Object, queue_size=10)

        #Inscricao para publicar/receber aqui!

        rate = rospy.Rate(10) #Define a frequencia de execucao em Hz

    

        while not rospy.is_shutdown():

            rate.sleep() #Espera o tempo para executar o programa na frequencia definida

    except rospy.ROSInternalException:
        pass