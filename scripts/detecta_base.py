#!/usr/bin/env python

import rospy
import rospkg

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo


from ger_drone.msg import Object
from ger_drone.msg import Identifier

from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np

#Constante
escala = 127.5
centro_base = (140,135)

K = [[0,0,0]*3]

base = 0

def converteImagem(img):
    return CvBridge().imgmsg_to_cv2(img)

def recebeImagem(msg):
    img = converteImagem(msg)

    orb = cv.ORB_create()

    kp1, des1 = orb.detectAndCompute(base, None)
    kp2, des2 = orb.detectAndCompute(img, None)

    FLANN_INDEX_LSH = 6
    index_params = dict(algorithm = FLANN_INDEX_LSH, table_number = 6, key_size = 12, multi_probe_level = 1)
    search_params = dict(checks = 100)

    flann = cv.FlannBasedMatcher(index_params, search_params)

    matches = flann.knnMatch(des1,des2,k=2)

    good = []
    for i,m_n in enumerate(matches):
        if len(m_n) != 2:
            continue
        (m,n) = m_n
        if m.distance < 0.7*n.distance:
            good.append(m)

    #Calcula os pontos da base na imagem
    pontoImagem = []

    pontoReal = []

    #train = kp2
    #query = kp1

    for match in good:
        point_base = kp1[match.queryIdx].pt
        point_photo = kp2[match.trainIdx].pt

        #Desloca o ponto para o centro de coordenadas do centro da base
        point_baseMetro = [point_base[0]-centro_base[0],point_base[1]-centro_base[1]]

        #Converte para metro
        point_baseMetro[0] = point_baseMetro[0] * escala 
        point_baseMetro[1] = point_baseMetro[1] * escala 

        #Adiciona as listas
        pontoImagem.append([point_photo[0],point_photo[1]])
        pontoReal.append(point_baseMetro)


    RObj, tObj = cv.solvePnP(pontoImagem, pontoReal, K, None)

    RCamera, tCamera = inverteTransformacao(RObj, tObj)

    publicaBase(RCamera, tCamera)



def inverteTransformacao(R, t):
    RInverso = np.transpose(R)

    tInverso  = - np.matmul(RInverso, t)

    return RInverso, tInverso


def recebeInfo(msg):  

    index = 0

    for i in range(3):
        for j in range(3):
            K[i][j] = msg.K[index]
            index += 1


def publicaBase(R, t):
    msg = Object()

    msg.identifier.type = msg.identifier.TYPE_BASE

    msg.pose.position.x = t[0]
    msg.pose.position.y = t[1]
    msg.pose.position.z = t[2]

    #Como colocar a orientacao?

    pubBase.publish(msg)

    pass

if __name__ == '__main__':

    try:
        rospy.init_node('detecta_base', anonymous="True")

        rospy.Subscriber('camera_baixo/image_rect_color', Image, recebeImagem)
        rospy.Subscriber('camera_baixo/camera_info', CameraInfo, recebeInfo)


        pubBase = rospy.Publisher('objeto_detectado',Object, queue_size=10)

        rate = rospy.Rate(10) #Define a frequencia de execucao em Hz

        
        #Consegue a foto de exemplo

        path = rospkg.RosPack().get_path('ger_drone')

        path = path + "/data/base.png"

        base = cv.imread(path,cv.IMREAD_GRAYSCALE)

        while not rospy.is_shutdown():

            #Codigo aqui!

            rate.sleep() #Espera o tempo para executar o programa na frequencia definida


    except rospy.ROSInternalException:
        pass