import subprocess

# Llamar al programa de Python con argumentos
subprocess.run(["python", "captura.py"])

import cv2
import matplotlib.pyplot as plt
import numpy as np
# Cargar la imagen
imagen1 = cv2.imread("Figure_1.png")
imagen2 = cv2.imread("captura.png")


# Cargamos el vídeo
#frame = cv2.imread('pruebat.png')
camera = cv2.VideoCapture(4)
#camera = cv2.VideoCapture("http://192.168.1.146:8080/video")


# Crear el objeto ORB
orb = cv2.ORB_create(nfeatures=2000, scaleFactor=1.8, nlevels=10, edgeThreshold=11)
# Detectar los puntos clave y calcular los descriptores para ambas imágenes
keypoints1, descriptores1 = orb.detectAndCompute(imagen1, None)
keypoints2, descriptores2 = orb.detectAndCompute(imagen2, None)

# Crear el objeto BFMatcher
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# Realizar la comparación de los descriptores
matches = bf.match(descriptores1, descriptores2)

print("Número de matches encontrados:", len(matches))

# Ordenar los matches por distancia
matches = sorted(matches, key=lambda x: x.distance)

# Dibujar los mejores matches en una imagen
resultado = cv2.drawMatches(imagen1, keypoints1, imagen2, keypoints2, matches[:10], None, flags=2)

# Mostrar la imagen con los matches
cv2.imshow("Matches", resultado)

cv2.waitKey(0)
cv2.destroyAllWindows()