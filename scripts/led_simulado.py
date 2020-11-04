#!/usr/bin/env python

import rospy

import cv2 as cv
import numpy as np

from ger_drone.msg import LedColor

cor = [0,0,0]

img = np.ones((500,500,3),np.uint8)*255

def recebeCor(msg):

    cor[0] = msg.r
    cor[1] = msg.g
    cor[2] = msg.b

def alteraLed(cor):
    cv.circle(img, (255-1,255-1), 255, cor, -1)
    cv.imshow("LED", img)
    cv.waitKey(1)
    


if __name__ == '__main__':
    try:

        rospy.init_node('led', anonymous="True")

        rospy.Subscriber('led_cor', LedColor, recebeCor)

        #Inscricao para publicar/receber aqui!

        rate = rospy.Rate(10) #Define a frequencia de execucao em Hz

        alteraLed([0.5,0.5,0.5])

        while not rospy.is_shutdown():

            alteraLed(cor)

            rate.sleep() #Espera o tempo para executar o programa na frequencia definida
        cv.destroyAllWindows()

    except rospy.ROSInternalException:
        cv.destroyAllWindows()
        pass