#!/usr/bin/env python

from Drone import Drone
from mrs_msgs.srv import ReferenceStampedSrv
from teste import descobrirPosicao

def main():
    nosso_drone = Drone()

    posicao_atual = descobrirPosicao()
    print("a")
    posicao_atual[0] = posicao_atual[0] + 4
    print(posicao_atual)
    nosso_drone.voarPara(posicao_atual)

    posicao_atual = nosso_drone.getPosicao()
    posicao_atual[1] = posicao_atual[1] + 4
    nosso_drone.voarPara(posicao_atual)

    posicao_atual = nosso_drone.getPosicao()
    posicao_atual[0] = posicao_atual[0] - 4 
    nosso_drone.voarPara(posicao_atual)

    posicao_atual = nosso_drone.getPosicao()
    posicao_atual[1] = posicao_atual[1] - 4
    nosso_drone.voarPara(posicao_atual)

main()