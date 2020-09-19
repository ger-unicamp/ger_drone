#!/usr/bin/env python

import rospy

from ger_drone.msg import Object
from ger_drone.msg import Identifier

if __name__ == '__main__':
    try:

        #Inscrição para publicar/receber aqui!

        rate = rospy.Rate(10) #Define a frequência de execução em Hz

        while not rospy.is_shutdown():
            
            #Código aqui!

            rate.sleep() #Espera o tempo para executar o programa na frequência definida


    except rospy.ROSInternalException:
        pass