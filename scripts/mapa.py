#!/usr/bin/env python

import rospy

from ger_drone.msg import Object
from ger_drone.msg import Identifier

from matplotlib import pyplot as plt

objetoX = []
objetoY = []

def recebeObjeto(msg):

    objetoX.append(msg.pose.position.x)
    objetoY.append(msg.pose.position.y)

    pass

if __name__ == '__main__':
    try:
        rospy.init_node('mapa', anonymous="True")

        rospy.Subscriber('objeto_detectado', Object, recebeObjeto)

        rate = rospy.Rate(10) #Define a frequencia de execucao em Hz

        while not rospy.is_shutdown():
        
            plt.plot(objetoX, objetoY)
            plt.show()

            rate.sleep() #Espera o tempo para executar o programa na frequencia definida

        plt.close('all')

    except rospy.ROSInternalException:
        plt.close('all')
        pass