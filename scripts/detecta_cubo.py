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
    return CvBridge().imgmsg_to_cv2(img)

def inverteTransformacao(R, t):
    RInverso = np.transpose(R)

    tInverso  = - np.matmul(RInverso, t)

    return RInverso, tInverso


def recebeInfo(msg):  

    global K

    index = 0

    for i in range(3):
        for j in range(3):
            K[i][j] = msg.K[index]
            index += 1

    K = np.array(K,dtype=np.float32)

def recebeImagem(msg):
    img = converteImagem(msg)

    data = decode(img)

    for qr in data:
        letra = qr.data

        pontoImagem = []
        for i in range(4):
            pontoImagem.append([qr.polygon[i][0],qr.polygon[i][1]])
        pontoImagem = np.array(pontoImagem, dtype=np.float32)

        a, RCamObj, tCamObj, _ = cv.solvePnPRansac(pontoReal,pontoImagem , K, np.zeros((5,1)))

        tCamObj = tCamObj.ravel()

        RCamObj, _ = cv.Rodrigues(RCamObj)

        publicaCubo(RCamObj, tCamObj, letra)

def publicaCubo(R, t, letra):
    msg = Object()

    if(letra == 'A'):
        msg.identifier.type.data = msg.identifier.TYPE_PACOTE_A
    elif(letra =='B'):
        msg.identifier.type.data = msg.identifier.TYPE_PACOTE_B
    elif(letra =='C'):
        msg.identifier.type.data = msg.identifier.TYPE_PACOTE_C
    elif(letra =='D'):
        msg.identifier.type.data = msg.identifier.TYPE_PACOTE_D
    elif(letra =='E'):
        msg.identifier.type.data = msg.identifier.TYPE_PACOTE_E
    else:
        return

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