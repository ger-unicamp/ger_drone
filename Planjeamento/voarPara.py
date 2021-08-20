#!/usr/bin/env python

from Drone import Drone
from mrs_msgs.srv import ReferenceStampedSrv

def main():
    nosso_drone = Drone()
    nosso_drone.voarPara([6,3,3])
main()