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

pontoReal = np.array([[5.5e-2, -5.5e-2, 0],
                    [5.5e-2, 5.5e-2, 0],
                    [-5.5e-2, 5.5e-2, 0],
                    [-5.5e-2, -5.5e-2, 0]], dtype=np.float32)

def converteImagem(img):
    """!
        Converte uma imagem fornecida pelo ROS para o formato do OpenCV

        @todo Criar um módulo com essa função de apoio

        Parâmetros:
            @param img (np.array) - Imagem fornecida pelo ROS a ser convertida.

        Retorno:
            @returns Imagem no formato do OpenCV.
    """

    return CvBridge().imgmsg_to_cv2(img, "bgr8")

def recebeInfo(msg): 
    """!
        Recebe a mensagem com as informações da câmera
        
        Extrai apenas a matriz de calibração

        @todo Criar um módulo com essa função de apoio

        Parâmetros:
            @param msg (CameraInfo) - informações da câmera
    """ 

    global K

    index = 0

    for i in range(3):
        for j in range(3):
            K[i][j] = msg.K[index]
            index += 1

    K = np.array(K,dtype=np.float32)


def procuraQuadrado(mascara):
    """!
        Procura o o quadrado do mostrador na imagem filtrada

        Função que busca, identifica, localiza e define as coordenadas
        de um quadrado caso seja identificado em uma figura.
        Define os contornos presentes na imagem, localiza os vértices dos
        contornos, desenvolve curvas ligando tais vértices (cv.approxPoly)
        e verifica a similaridade com uma reta. Caso todos os vértices ligados
        correspondam ao formato desejado, extrai as coordenadas de tais vértices
        e aponta-os como o quadrado.

        Parâmetros:
            @param mascara(np.array) - imagem previamente filtrada.

        Retorno:
            @returns Coordenadas dos vértices do quadrado obtido.
    """
    
    kernel = np.ones((5,5),np.uint8)
    
    bordas = cv.Canny(mascara, 100, 500, kernel)

    contours = []
    hierarchy = []

    if(cv.__version__[0] == "4"):
        contours,hierarchy = cv.findContours(bordas, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    else:
        _, contours, hierarchy = cv.findContours(bordas, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    quadrados = []

    for i in range(len(contours)):
        epsilon = 0.1*cv.arcLength(contours[i],True)
        approx = cv.approxPolyDP(contours[i],epsilon,True)

        if(len(approx) < 4):
            continue

        if not cv.isContourConvex(approx):
            continue

        if cv.contourArea(approx) < 2000:
            continue

        quadrado = []

        for point in approx:
            quadrado.append([point[0][0], point[0][1]])
        
        quadrados.append(quadrado)

    quadrados = np.array(quadrados)

    i = 0

    while(len(quadrados)-2 >= i):
        if np.linalg.norm(quadrados[i+1][0]-quadrados[i][0]) < 5:
            quadrados = np.delete(quadrados, i, 0)
            
        i+=1

    i = 0

    while len(quadrados) > i:
        if np.linalg.norm(quadrados[i][0]-quadrados[i][1])/np.linalg.norm(quadrados[i][2]-quadrados[i][1]) <0.9:
           quadrados = np.delete(quadrados, i, 0) 
        i+=1
        

    return quadrados

def processaImagem(img):
    """!
        Recebe a mensagem de imagem, procura os mostradores e publica
        
        Realiza um threshold de uma imagem a fim de aplicar o filtro closing. Em
        seguida, aplica a função de identificar a presença de um quadrado na imagem
        filtrada.
        
        Parâmetros:
            @param img (np.array) - imagem a ser processada.

        Retorno:
            @returns Verificação da presença de quadrados na imagem filtrada.

    """
    
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    hls = cv.cvtColor(img, cv.COLOR_BGR2HLS)

    kernel = np.ones((5,5),np.uint8)

    min_preto = np.array([0,0,0],np.uint8)
    max_preto = np.array([180,255,40],np.uint8)

    min_branco = np.array([0,240,0], dtype=np.uint8)
    max_branco = np.array([180,255,255], dtype=np.uint8)

    masc_preto = cv.inRange(hsv,min_preto,max_preto)
    masc_branco = cv.inRange(hls,min_branco,max_branco)



    masc = cv.bitwise_or(masc_preto, masc_branco, np.ones_like(masc_branco))

    
    closing = cv.morphologyEx(masc, cv.MORPH_CLOSE, kernel, iterations=10)

    res = cv.bitwise_and(img, img, mask=closing)


    quadrados = procuraQuadrado(closing)

    return quadrados


def transformaImagem(img, quad):
    """!
        Aplica uma transformação projetiva para colocar o mostrador em evidência
        
        Transforma os quatro cantos dele para os quatro cantos da imagem, tornando seus lados paralelos

        Parâmetros:
            img (np.darray) - Imagem em que o mostrador está
            quad (np.darray 4x2) - Cantos do mostrador
    """

    pts1 = np.float32(quad[:4])
    pts1 = np.asarray(pts1, dtype = np.float32)

    CE = 0
    BE = 0
    CD = 0
    BD = 0

    for i in range(4):
        
        nAbaixo = 0
        nEsquerda = 0
        for j in range(4):
            if(pts1[i][0]>pts1[j][0]):
                nEsquerda += 1
            if(pts1[i][1]>pts1[j][1]):
                nAbaixo += 1
        
        if(nAbaixo >=2 and nEsquerda >=2):
            CD = i
        elif(nAbaixo >=2):
            CE = i
        elif(nEsquerda >= 2):
            BD = i
        else:
            BE = i

    pts1 = np.array([pts1[BE],pts1[BD], pts1[CD], pts1[CE] ], dtype=np.float32)

    pts2 = np.float32([[0,0],[100,0], [100,100], [0,100]], dtype = np.float32)

    M = cv.getPerspectiveTransform(pts1,pts2)
    img2 = cv.warpPerspective(img,M,(100,100))

    return img2

def rotacionaImagem(img):
    """!
        Rotaciona o mostrador para ficar com a orientação correta de leitura

        Identifica a inclinação do mostrador em relação à câmera de forma a deixar
        seus dígitos no formato de leitura da esquerda para a direita e de cima
        para baixo. Realiza rotações da imagem até que sua inclinação esteja de
        acordo com tais condições para posterior extração de dígitos.
        
        Parâmetros:
            @param img (np.array) - Imagem lida para análise

        Retorno: 
            @param img (np.array) - Imagem rotacionada
    
    """
    
    template = percentual[:,0:45]

    w = template.shape[1]
    h = template.shape[0]

    res = cv.matchTemplate(img,template,cv.TM_CCOEFF_NORMED)

    threshold = 0.5
    loc = np.where( res >= threshold)

    E = 0
    C = 0
    B = 0
    D = 0

    if(len(loc[0]) == 0):
        template = percentual2[:45,:]

        w = template.shape[1]
        h = template.shape[0]

        res = cv.matchTemplate(img,template,cv.TM_CCOEFF_NORMED)

        threshold = 0.5
        loc = np.where( res >= threshold)


        for pt in zip(*loc[::-1]):
            if(pt[0]<25 or pt[0] > 75) and (pt[1]<25 or pt[1] > 75):
                continue
                
            if(pt[0]>50):
                D += 1
            else:
                E += 1

    else:
        for pt in zip(*loc[::-1]):
                
            if(pt[1]< 50):
                C += 1
            else:
                B += 1

    if E>D and E>B and E>C:
        #Esquerda
        img = cv.rotate(img,cv.ROTATE_180)
    elif B> D and B>C:
        #Baixo
        img = cv.rotate(img,cv.ROTATE_90_COUNTERCLOCKWISE)
    elif C>D:
        #Cima
        img = cv.rotate(img,cv.ROTATE_90_CLOCKWISE)
    elif D == 0:
        #Nao Direita
        return None

    return img

def extraiDigitos(dig):
    """!
        Extrai o dígito na imagem

        Fragmenta uma imagem de um dígito em uma forma de um display de 7 segmentos.
        Analisa quais os segmentos que estão "apagados" ou "acesos" e, baseado nisto,
        determina qual o dígito em questão. É um método pouco eficiente por depender
        de coordenadas em uma imagem, devendo ser otimizado para que reconheça
        formatos em vez disso.
        
        Parâmetro:
            @param dig (np.array) - lista de imagens de dígitos a serem analisados.
        Retorno:
            @returns  dígito obtido (int)

    
    """
    
    ret,dig = cv.threshold(dig,127,255, cv.THRESH_BINARY)
    parte0 = dig[6:18,22:80]
    parte1 = dig[15:50,5:25]
    parte2 = dig[15:50,78:94]
    parte3 = dig[45:57,22:80]
    parte4 = dig[54:87,5:25]
    parte5 = dig[54:87,78:94]
    parte6 = dig[85:95,22:80]

    verifparte0 = cv.countNonZero(parte0)
    verifparte1 = cv.countNonZero(parte1)
    verifparte2 = cv.countNonZero(parte2)
    verifparte3 = cv.countNonZero(parte3)
    verifparte4 = cv.countNonZero(parte4)
    verifparte5 = cv.countNonZero(parte5)
    verifparte6 = cv.countNonZero(parte6)

    verif = [verifparte0, verifparte1, verifparte2, verifparte3, verifparte4, verifparte5, verifparte6]
    for i in range(len(verif)):
        if verif[i] > 200:
            verif[i] = 1
        else:
            verif[i] = 0
    
    if verif[0]==1 and verif[1]==1 and verif[2]==1 and verif[3]==0 and verif[4]==1 and verif[5]==1 and verif[6]==1:
        return 0
    elif verif[0]==0 and verif[1]==0 and verif[2]==1 and verif[3]==0 and verif[4]==0 and verif[5]==1 and verif[6]==0:
        return 1
    elif verif[0]==1 and verif[1]==0 and verif[2]==1 and verif[3]==1 and verif[4]==1 and verif[5]==0 and verif[6]==1:
        return 2
    elif verif[0]==1 and verif[1]==0 and verif[2]==1 and verif[3]==1 and verif[4]==0 and verif[5]==1 and verif[6]==1:
        return 3
    elif verif[0]==0 and verif[1]==1 and verif[2]==1 and verif[3]==1 and verif[4]==0 and verif[5]==1 and verif[6]==0:
        return 4
    elif verif[0]==1 and verif[1]==1 and verif[2]==0 and verif[3]==1 and verif[4]==0 and verif[5]==1 and verif[6]==1:
        return 5
    elif verif[0]==1 and verif[1]==1 and verif[2]==0 and verif[3]==1 and verif[4]==1 and verif[5]==1 and verif[6]==1:
        return 6
    elif verif[0]==1 and verif[1]==0 and verif[2]==1 and verif[3]==0 and verif[4]==0 and verif[5]==1 and verif[6]==0:
        return 7
    elif verif[0]==1 and verif[1]==1 and verif[2]==1 and verif[3]==1 and verif[4]==1 and verif[5]==1 and verif[6]==1:
        return 8
    elif verif[0]==1 and verif[1]==1 and verif[2]==1 and verif[3]==1 and verif[4]==0 and verif[5]==1 and verif[6]==1:
        return 9

def converte_coord(valor):
    """!
        Redimensiona a imagem

        Retorno:
            @returns Imagem redimensionada
        
    """

    pts1 = ([0,0],[24,0],[24,44],[0,44])
    pts1 = np.asarray(pts1, dtype = np.float32)
    pts2 = np.float32([[0,0],[100,0], [100,100], [0,100]])

    M = cv.getPerspectiveTransform(pts1,pts2)
    img2 = cv.warpPerspective(valor,M,(100,100))
    return img2

def recebeImagem(msg):
    '''!
        Recebe a mensagem com a imagem, procura os mostradores e publica

        Parâmetros:
            msg (Image): mensagem de imagem onde os mostradores serão procurados

    '''

    global pontoReal

    img = converteImagem(msg)

    if(img is None):
        return

    quadrados = processaImagem(img)

    if(quadrados is None):
        return

    if len(quadrados) == 0:
        return

    for quad in quadrados:
        if(len(quad) != 4):
            continue

        img = transformaImagem(img, quad)

        img = rotacionaImagem(img)

        if(img is None):
            continue

        _, th = cv.threshold(cv.cvtColor(img,cv.COLOR_BGR2GRAY),100,255,cv.THRESH_BINARY)

        dig11 = th[3:48,12:37]
        dig12 = th[3:48,40:64]
        dig21 = th[48:94, 12:37]
        dig22 = th[48:94,40:63]

        dig = [dig11, dig12,dig21, dig22]
        
        digitoExtraido = []
        for j in range(len(dig)):
            dig[j] = converte_coord(dig[j])
            digitoExtraido.append(extraiDigitos(dig[j]))

        if(digitoExtraido[2] == None):
            digitoExtraido[2] = '-'
        if(digitoExtraido[0] == None or digitoExtraido[1] == None or digitoExtraido[3] == None):
            continue
        
        data = str(digitoExtraido[0])  +str(digitoExtraido[1])+" "+ str(digitoExtraido[2]) +str(digitoExtraido[3]) 
   
        a = 0
        RCamObj = []
        tCamObj = []


        pontoImagem = np.asarray(quad, dtype=np.float32)

        if(cv.__version__[0] == "4"):
            a, RCamObj, tCamObj, _ = cv.solvePnPRansac(pontoReal,pontoImagem , K, np.zeros((5,1)))
        else:
            pontoI = np.expand_dims(pontoImagem, 1)
            pontoR = np.expand_dims(pontoReal, 1)
            a, RCamObj, tCamObj, _ = cv.solvePnPRansac(pontoR,pontoI , K, np.zeros((5,1), dtype=np.float32))

        tCamObj = tCamObj.ravel()

        RCamObj, _ = cv.Rodrigues(RCamObj)

        publicaObj(RCamObj, tCamObj, data)
        
        

def publicaObj(R, t, data):
    """!
        Publica um mostrador

        Recebe a informação da posição do mostrador e publica um objeto detectado nesse lugar

        Parâmetros:
            @param R (list/np.darray 3x3) - matriz de rotação
            @param t (list/np.darray 3x1) - vetor de translação
            @param data (int) - dígito que está no mostrador
    """

    msg = Object()

    msg.identifier.data = data
    msg.identifier.type.data = Identifier.TYPE_MOSTRADOR
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

    pubObj.publish(msg)

if __name__ == '__main__':
    try:

        rospy.init_node('detecta_mostrador', anonymous="True")

        path = rospkg.RosPack().get_path('ger_drone')

        path = path + "/data/"

        percentual = cv.imread(path+'percentual.png')
        percentual = cv.rotate(percentual,cv.ROTATE_90_COUNTERCLOCKWISE)
        percentual = cv.resize(percentual,(100,35))
        percentual2 = cv.rotate(percentual,cv.ROTATE_90_COUNTERCLOCKWISE)

        rospy.Subscriber('image_rect_color', Image, recebeImagem)
        rospy.Subscriber('camera_info', CameraInfo, recebeInfo)

        pubObj = rospy.Publisher('objeto_detectado',Object, queue_size=10)

        #Inscricao para publicar/receber aqui!

        rate = rospy.Rate(10) #Define a frequencia de execucao em Hz

    

        while not rospy.is_shutdown():

            rate.sleep() #Espera o tempo para executar o programa na frequencia definida

    except rospy.ROSInternalException:
        pass
