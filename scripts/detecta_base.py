#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Image

from ger_drone.msg import Object
from ger_drone.msg import Identifier

from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np


if __name__ == '__main__':
    try:
        rate = rospy.Rate(10) #Define a frequência de execução em Hz

        while not rospy.is_shutdown():

            #Código aqui!

            rate.sleep() #Espera o tempo para executar o programa na frequência definida


    except rospy.ROSInternalException:
        pass