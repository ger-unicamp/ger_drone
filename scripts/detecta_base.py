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
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation

#Constante
escala = 0.007843137
centro_base = (140,135)

K = [[0,0,0],[0,0,0],[0,0,0]]

base = 0

def converteImagem(img):
    return CvBridge().imgmsg_to_cv2(img)

def recebeImagem(msg):
    img = converteImagem(msg)

    orb = cv.ORB_create()

    kp1, des1 = orb.detectAndCompute(base, None)

    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    kerno = np.ones((5,5),"uint8")

    #Filtragem da cor amarela
    min_amarelo = np.array([20,245,245], np.uint8)
    max_amarelo = np.array([40,255,255], np.uint8)
    masc_amarelo = cv.inRange(hsv,min_amarelo,max_amarelo)
    masc_amarelo = cv.dilate(masc_amarelo,kerno)
    res_amarelo = cv.bitwise_and(img,img,mask=masc_amarelo)

    #Filtragem da cor azul
    min_azul = np.array([99,157,186], np.uint8)
    max_azul = np.array([119,177,206], np.uint8)
    masc_azul = cv.inRange(hsv,min_azul,max_azul)
    masc_azul = cv.dilate(masc_azul,kerno)
    res_azul = cv.bitwise_and(img,img,mask=masc_azul)

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
        if m.distance < 0.8*n.distance:
            good.append(m)

    if(len(good)<10):
        return

    #Calcula os pontos da base na imagem
    pontoImagem = []

    pontoReal = []

    #train = kp2
    #query = kp1

    for match in good:
        point_base = kp1[match.queryIdx].pt
        point_photo = kp2[match.trainIdx].pt

        #Desloca o ponto para o centro de coordenadas do centro da base
        point_baseMetro = [point_base[0]-centro_base[0],point_base[1]-centro_base[1], 0]

        #Converte para metro
        point_baseMetro[0] = point_baseMetro[0] * escala 
        point_baseMetro[1] = point_baseMetro[1] * escala 

        #Adiciona as listas
        pontoImagem.append([point_photo[0],point_photo[1]])
        pontoReal.append(point_baseMetro)

    pontoImagem = np.array(pontoImagem, dtype=np.float32)
    pontoReal = np.array(pontoReal,dtype=np.float32)

    a, RObj, tObj, _ = cv.solvePnPRansac(pontoReal,pontoImagem , K, np.zeros((5,1)))

    if not a:
        return

    RObj, _ = cv.Rodrigues(RObj)

    RCamera, tCamera = inverteTransformacao(RObj, tObj)

    publicaBase(RCamera, tCamera)

    rospy.loginfo("Found "+str(len(good))+" matches")

    #img3 = cv.drawMatches(base, kp1, img, kp2, good, None, flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    #plt.imshow(img3)
    #plt.show()



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

def publicaBase(R, t):
    msg = Object()

    msg.identifier.type.data = msg.identifier.TYPE_BASE

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

    pass

if __name__ == '__main__':

    try:
        rospy.init_node('detecta_base', anonymous="True")

        rospy.Subscriber('image_rect_color', Image, recebeImagem)
        rospy.Subscriber('camera_info', CameraInfo, recebeInfo)


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