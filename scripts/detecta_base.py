#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo


from ger_drone.msg import Object
from ger_drone.msg import Identifier

from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np

K = [[0,0,0]*3]

def converteImagem(img):
    return CvBridge().imgmsg_to_cv2(img)

def recebeImagem(msg):
    img = converteImagem(msg)

    #Calcula os pontos da base na imagem
    pontoImagem = []

    pontoReal = []

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

    msg.identifier.id = 1

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


        pubBase = rospy.Publisher('base',Object, queue_size=10)

        rate = rospy.Rate(10) #Define a frequencia de execucao em Hz

        while not rospy.is_shutdown():

            #Codigo aqui!

            rate.sleep() #Espera o tempo para executar o programa na frequencia definida


    except rospy.ROSInternalException:
        pass