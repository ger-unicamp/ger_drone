import numpy as np
import cv2 as cv

K = np.array([[215.6810060961547, 0.0, 376.5], [0.0, 215.6810060961547, 240.5], [0.0, 0.0, 1.0]],dtype=np.float32)

pontoReal = np.array([[5.5e-2, -5.5e-2, 0],
                    [5.5e-2, 5.5e-2, 0],
                    [-5.5e-2, 5.5e-2, 0],
                    [-5.5e-2, -5.5e-2, 0]], dtype=np.float32)

def procuraQuadrado(mascara):
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

#Estima a pose do sensor a partir de seus cantos na imagem
def estimaPose(quad):
    global pontoReal
    a = 0
    RCamObj = []
    tCamObj = []
    try:
            a, RCamObj, tCamObj, _ = cv.solvePnPRansac(pontoReal,quad , K, np.zeros((5,1)))
    except:
        pontoI = np.expand_dims(quad, 1)
        pontoR = np.expand_dims(pontoReal, 1)
        a, RCamObj, tCamObj, _ = cv.solvePnPRansac(pontoR,pontoI , K, np.zeros((5,1), dtype=np.float32))

    #return RObjCamera, tObjCamera 

    return RCamObj, tCamObj

def salvaTodosQuadrados(quadrados, img, i):
    imgDraw = img.copy()
    
    if(len(quadrados) != 0):
        imgDraw = cv.circle(imgDraw, (0,0), 50, (0,0,255),-1)

    for quad in quadrados:
        for point in quad:
            cv.circle(imgDraw, (point[0], point[1]), 2, (255,0,0), -1)

    cv.imwrite("result/"+str(i)+".png", imgDraw)


def imprimeQuadrado(quadrado, img):
    imgDraw = img.copy()
    
    for point in quadrado:
        cv.circle(imgDraw, (point[0], point[1]), 2, (255,0,0), -1)

    plt.imshow(cv.cvtColor(imgDraw,cv.COLOR_BGR2RGB))
    plt.show()

def imprimeTodosQuadrados(quadrados, img):
    imgDraw = img.copy()
    for quad in quadrados:
        for point in quad:
            cv.circle(imgDraw, (point[0], point[1]), 2, (255,0,0), -1)
    
    plt.imshow(cv.cvtColor(imgDraw,cv.COLOR_BGR2RGB))
    plt.show()

def verifica_semelhanca(imagem1,imagem2):
    imagem1 = cv.cvtColor(imagem1, cv.COLOR_BGR2GRAY)
    imagem2 = cv.cvtColor(imagem2, cv.COLOR_BGR2GRAY)
    ret,thresh1 = cv.threshold(imagem1, 127, 255,0)
    ret,thresh2 = cv.threshold(imagem2, 127, 255,0)
    _, contours, hierarchy = cv.findContours(thresh1,cv.RETR_CCOMP,cv.CHAIN_APPROX_NONE)
    cnt1 = contours[0]
    _, contours, hierarchy = cv.findContours(thresh2,cv.RETR_CCOMP,cv.CHAIN_APPROX_NONE)
    cnt2 = contours[0]
    ret = cv.matchShapes(cnt1,cnt2,1,0.0)
    return ret

def processaImagem(img, index):
    global percentual

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

    if len(quadrados) == 0:
        return []

    for quad in quadrados:

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

        pts2 = np.float32([[0,0],[100,0], [100,100], [0,100]])

        M = cv.getPerspectiveTransform(pts1,pts2)
        img2 = cv.warpPerspective(img,M,(100,100))

        #Procura a orientacao da imagem
        img3d = img2[4:95,65:96]
        img3i = img2[63:96,7:96]
        img3e = img2[2:95,4:35]
        img3s = img2[6:38,4:94]

        min = 10
        minIndex = 0
        l3 = [img3d, img3i, img3e, img3s]
        for i in range(4):
            ret = verifica_semelhanca(l3[i],percentual)
            if (ret < min):
                min = ret
                minIndex = i

        img3 = l3[minIndex]
        # Uma vez encontrado onde os % estao, rotaciona e endireita a imagem p/ padroniza-la
        if minIndex == 1:
            img2 = cv.rotate(img2,cv.ROTATE_90_COUNTERCLOCKWISE)
        elif minIndex == 2:
            img2 = cv.rotate(img2,cv.ROTATE_180)
        elif minIndex == 3:
            img2 = cv.rotate(img2,cv.ROTATE_90_CLOCKWISE)

        #salvaTodosQuadrados([],img2,index)

    return quadrados

percentual = cv.imread('percentual.png')

#0, 170
for i in range(0, 170):
    name = "/home/elton/Documentos/GitHub/drone_data/Mostrador1/img/frame" + ((4-len(str(i)))*"0"+str(i)) + ".jpg"

    img = cv.imread(name)

    quad = processaImagem(img, i)

    print(len(quad))

    #salvaTodosQuadrados(quad, img, i)

    

'''
import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt

foto = cv.imread('fase3-7.png')
kernel = np.ones((5,5),np.uint8)

hsv = cv.cvtColor(foto, cv.COLOR_BGR2HSV)
## Aplicacao das mascaras
min_masc = np.array([0,0,0],np.uint8)
max_masc = np.array([0,255,255],np.uint8)
masc = cv.inRange(hsv,min_masc,max_masc)
res = cv.bitwise_and(foto,foto,mask= masc)
closing = cv.morphologyEx(masc, cv.MORPH_CLOSE, kernel, iterations=3)

#cv.imshow('foto',foto); cv.imshow('masc',masc); cv.imshow('close',closing); cv.waitKey(); cv.destroyAllWindows()

def procuraQuadrado(mascara):
    kernel = np.ones((5,5),np.uint8)
    
    bordas = cv.Canny(mascara, 100, 500, kernel)
    _,contours,hierarchy = cv.findContours(bordas, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)

    quadrados = []

    for i in range(len(contours)):
        epsilon = 0.1*cv.arcLength(contours[i],True)
        approx = cv.approxPolyDP(contours[i],epsilon,True)

        if(len(approx) < 4):
            continue

        if not cv.isContourConvex(approx):
            continue

        if cv.contourArea(approx) < 100:
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
    
    return quadrados

def imprimeTodosQuadrados(quadrados, img):
    imgDraw = img.copy()
    for quad in quadrados:
        for point in quad:
            cv.circle(imgDraw, (point[0], point[1]), 5, (255,0,255), -1)
    
    plt.imshow(cv.cvtColor(imgDraw,cv.COLOR_BGR2RGB))
    plt.show()

rows,cols,ch = foto.shape
L = procuraQuadrado(masc)

## Converte o quadrado encontrado para um novo sistema de coordenadas
if len(L) == 0:
    exit()
pts1 = np.float32(L[0,:4]        # Uma vez encontrado onde os % estao, rotaciona e endireita a imagem p/ padroniza-la
        if minj == 1:
            img2 = cv.rotate(img2,cv.ROTATE_90_COUNTERCLOCKWISE)
        elif minj == 2:
            img2 = cv.rotate(img2,cv.ROTATE_180)
        elif minj == 3:
            img2 = cv.rotate(img2,cv.ROTATE_90_CLOCKWISE))
pts1 = np.asarray(pts1, dtype = np.float32)
pts2 = np.float32([[0,0],[100,0], [100,100], [0,100]])

M = cv.getPerspectiveTransform(pts1,pts2)
img2 = cv.warpPerspective(foto,M,(100,100))

#Leitura de uma foto contendo os dois %
percentual = cv.imread('percentual.png')

## Funcao - Verifica a semelhanca entre duas imagens quaisquer
def verifica_semelhanca(imagem1,imagem2):
    imagem1 = cv.cvtColor(imagem1, cv.COLOR_BGR2GRAY)
    imagem2 = cv.cvtColor(imagem2, cv.COLOR_BGR2GRAY)
    ret,thresh1 = cv.threshold(imagem1, 127, 255,0)
    ret,thresh2 = cv.threshold(imagem2, 127, 255,0)
    _, contours, hierarchy = cv.findContours(thresh1,cv.RETR_CCOMP,cv.CHAIN_APPROX_NONE)
    cnt1 = contours[0]
    _, contours, hierarchy = cv.findContours(thresh2,cv.RETR_CCOMP,cv.CHAIN_APPROX_NONE)
    cnt2 = contours[0]
    ret = cv.matchShapes(cnt1,cnt2,1,0.0)
    return ret


## Verifica onde os dois % estao na imagem original
img3d = img2[4:95,65:96]
img3i = img2[63:96,7:96]
img3e = img2[2:95,4:35]
img3s = img2[6:38,4:94]



plt.imshow(img2)
plt.show()

min = 10
l3 = [img3d, img3i, img3e, img3s]
for i in range(4):
    ret = verifica_semelhanca(l3[i],percentual)
    if (ret < min):
        min = ret
        minj = i

img3 = l3[minj]
# Uma vez encontrado onde os % estao, rotaciona e endireita a imagem p/ padroniza-la
if minj == 1:
    img2 = cv.rotate(img2,cv.ROTATE_90_COUNTERCLOCKWISE)
elif minj == 2:
    img2 = cv.rotate(img2,cv.ROTATE_180)
elif minj == 3:
    img2 = cv.rotate(img2,cv.ROTATE_90_CLOCKWISE)

plt.imshow(img2)
plt.show()

## Leitura dos digitos da imagem rotacionada
dig11 = img2[3:48,12:37]
dig12 = img2[3:48,40:64]
dig21 = img2[48:94, 12:37]
dig22 = img2[48:94,40:63]

dig = [dig11, dig12, dig22]

plt.imshow(dig11)
plt.show()
plt.imshow(dig12)
plt.show()
plt.imshow(dig22)
plt.show()
#imprimeTodosQuadrados(procuraQuadrado(masc), foto)


## Funcao - Converte coordenadas para um sistema cartesiano
def converte_coord(valor):
    pts1 = ([0,0],[24,0],[24,44],[0,44])
    pts1 = np.asarray(pts1, dtype = np.float32)
    pts2 = np.float32([[0,0],[100,0], [100,100], [0,100]])

    M = cv.getPerspectiveTransform(pts1,pts2)
    img2 = cv.warpPerspective(valor,M,(100,100))
    return img2



def extraiDigitos(dig):
    ret,dig = cv.threshold(dig,127,255, cv.THRESH_BINARY)
    dig = cv.cvtColor(dig,cv.COLOR_BGR2GRAY)
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
    print(verif)
    for i in range(len(verif)):
        if verif[i] > 150:
            verif[i] = 1
        else:
            verif[i] = 0
    print(verif)
    
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

retorno = []
for i in range(len(dig)):
    dig[i] = converte_coord(dig[i])
    retorno.append(extraiDigitos(dig[i]))

print(retorno)

'''
