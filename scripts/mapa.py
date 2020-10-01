#!/usr/bin/env python

import rospy

from scipy.spatial.transform import Rotation as R
import numpy as np
from matplotlib import pyplot as plt

from ger_drone.msg import Object
from ger_drone.msg import Identifier

from mrs_msgs.msg import UavState



objetoX = []
objetoY = []

position = np.asarray([0,0,0])

rotation = np.asarray([[1,0,0],[0,1,0],[0,0,1]])

def recebeObjeto(msg):

    

    #Posicao do obj nas coordenadas da camera
    cameraT = np.asarray([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    cameraR = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]).as_dcm()

    #@todo concatenar pose da camera no frame do drone

    #Concatena com a pose do drone para obter a pose no mundo
    worldR = np.matmul(rotation, cameraR)
    worldT = position+np.matmul(rotation, cameraT)

    if(worldT[0] > 8 or worldT[0] < 0):
        return
    if(worldT[1] > 8 or worldT[1] < 0):
        return

    objetoX.append(worldT[0])
    objetoY.append(worldT[1])

    r = R.from_dcm(worldR)
    theta = r.as_euler('xyz')[2]


def recebePose(msg):
    global position, rotation

    position[0] = msg.pose.position.x
    position[1] = msg.pose.position.y
    position[2] = msg.pose.position.z

    r = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])
    rotation = r.as_dcm()


if __name__ == '__main__':
    try:
        rospy.init_node('mapa', anonymous="True")

        rospy.Subscriber('/uav1/bluefox_optflow/objeto_detectado', Object, recebeObjeto)
        rospy.Subscriber('/uav1/odometry/uav_state', UavState, recebePose)

        rate = rospy.Rate(10) #Define a frequencia de execucao em Hz

        plt.show()

        while not rospy.is_shutdown():
        
            plt.ylim(0,8)
            plt.xlim(0,8)
            plt.scatter(objetoX, objetoY)
            plt.draw()
            plt.pause(0.0000001)

            rate.sleep() #Espera o tempo para executar o programa na frequencia definida

        plt.close('all')

    except rospy.ROSInternalException:
        plt.close('all')
        pass