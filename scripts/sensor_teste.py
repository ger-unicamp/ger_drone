import cv2 as cv
import numpy as np

foto = cv.imread('fase2-6.png')

kernel = np.ones((5,5),np.uint8)
#testar mascara
hsv = cv.cvtColor(foto, cv.COLOR_BGR2HSV)
#filtro vermelho
min_verm = np.array([0,210,100],np.uint8)
max_verm = np.array([8,255,255],np.uint8)
masc_verm = cv.inRange(hsv,min_verm,max_verm)
res_verm = cv.bitwise_and(foto,foto,mask= masc_verm)

#filtro verde
#min_verde
min_verde = np.array([50,200,0],np.uint8)
max_verde = np.array([150,255,255],np.uint8)
masc_verde = cv.inRange(hsv,min_verde,max_verde)
res_verde = cv.bitwise_and(foto,foto,mask= masc_verde)

#Pelo visto, so o canny ja basta para localizar as quinas
bordas = cv.Canny(masc_verm, 100, 500, kernel)
_, contorno,hierarchy = cv.findContours(bordas, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
for cnt in contorno:
    (x,y,l,a) = cv.boundingRect(cnt)
#    print(cnt)

cantos = cv.goodFeaturesToTrack(masc_verm, 40, 0.0001, 7)
if (cantos is None):
    exit()          #Alterar no programa final, fazendo return

cantos = np.int0(cantos)
pos = []
quad = []
for canto in cantos:
	x,y = canto.ravel()
	cv.circle(foto,(x,y),2,(255,0,0),-1)
	pos.append((x,y))

print(pos)
while len(pos) != 0:
	i = 0; j = 1
	while j <= len(pos)-1:
		if abs(pos[i][0] - pos[i+j][0]) < 20:
			if abs(pos[i][1] - pos[i+j][1]) < 20:
				quad.append(pos[j])
				del(pos[j])
		j+=1
	if abs(pos[i][0] - pos[len(pos)-1][0]) < 20:
		if abs(pos[i][1] - pos[len(pos)-1][1]) < 20:
			quad.append(pos[len(pos)-1])
			del(pos[len(pos)-1])
	quad.append(pos[i]); del(pos[i])
print(quad)
coord = []
j = 0
for i in range(len(quad)/4):
	coord.append([quad[0+j], quad[1+j], quad[2+j], quad[3+j]])
	j += 4


print(coord)

cv.imshow('contornos',bordas)
cv.imshow('foto',foto)
cv.imshow('resultado vermelho',res_verm)
cv.imshow('resultado verde',res_verde)
cv.waitKey()
cv.destroyAllWindows()
