#!/usr/bin/env python

'''!
    Nó para simular um LED
'''

import rospy

import cv2 as cv
import numpy as np

from ger_drone.msg import LedColor
from matplotlib import pyplot as plt

cor = [0,0,0]

img = np.ones((500,500,3),np.uint8)*255

def recebeCor(msg):
    '''!
        Recebe uma mensagem com a cor que o led precisa mostra

        Parâmetros:
            @param msg (LedColor): mensagem 
    '''

    global cor
    cor[2] = msg.r
    cor[1] = msg.g
    cor[0] = msg.b

def alteraLed(cor):
    '''!
        Altera a cor do LED sendo mostrado
        
        Parâmetros:
            @cor (list/np.darray 3x1): cor BGR que deve ser mostrada
    '''

    cv.circle(img, (255-1,255-1), 255, cor, -1)
    


if __name__ == '__main__':
    try:

        rospy.init_node('led', anonymous="True")

        rospy.Subscriber('led_color', LedColor, recebeCor)

        #Inscricao para publicar/receber aqui!

        rate = rospy.Rate(10) #Define a frequencia de execucao em Hz

        alteraLed([0.5,0.5,0.5])

        im = plt.imshow(img)
        while not rospy.is_shutdown():

            alteraLed(cor)

            if(cv.__version__[0] == "4"):
                cv.imshow("LED", img)
                cv.waitKey(1)
            else:
                im.set_data(cv.cvtColor(img, cv.COLOR_BGR2RGB))
                plt.pause(.1)
                plt.draw()

            rate.sleep() #Espera o tempo para executar o programa na frequencia definida
        if(cv.__version__[0] == "4"):
            cv.destroyAllWindows()

    except rospy.ROSInternalException:
        if(cv.__version__[0] == "4"):
            cv.destroyAllWindows()
        pass