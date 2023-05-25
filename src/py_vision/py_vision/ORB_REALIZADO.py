import os
os.system('clear')

import subprocess

# Llamar al programa de Python con argumentos
subprocess.run(["python", "captura.py"])

import cv2
import numpy as np

# Cargar las imágenes
imagen1 = cv2.imread("Figure_1.png")
imagen2 = cv2.imread("9.jpeg")

# Crear el objeto ORB
orb = cv2.ORB_create(nfeatures=6900, scaleFactor=1.5, nlevels=9, edgeThreshold=10)

while True:
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

    # Obtener las dimensiones de la imagen 1
    alto, ancho, _ = imagen1.shape

    # Dividir la imagen 1 en secciones de 3x3
    tamano_seccion_alto = alto // 3
    tamano_seccion_ancho = ancho // 3

    # Matriz para almacenar los conteos de matches en cada sección
    conteo_matches_secciones = np.zeros((3, 3), dtype=int)

    # Recorrer los mejores matches
    num_mejores_matches = 10  # Número de mejores matches a considerar
    for match in matches[:num_mejores_matches]:
        # Obtener las coordenadas del punto clave en imagen 1
        x, y = keypoints1[match.queryIdx].pt
        x = int(x)
        y = int(y)

        # Determinar en qué sección se encuentra el punto clave
        seccion_y = y // tamano_seccion_alto
        seccion_x = x // tamano_seccion_ancho

        # Incrementar el conteo de matches en la sección correspondiente
        conteo_matches_secciones[seccion_y, seccion_x] += 1

        # Dibujar un círculo en la posición del punto clave en imagen 1
        cv2.circle(imagen1, (x, y), 5, (0, 255, 0), -1)

    # Imprimir los conteos de matches por sección
    print("Conteos de matches por sección:")
    for fila in conteo_matches_secciones:
        print(fila)

    # Dibujar las secciones en la imagen 1
    for i in range(1, 3):
        # Líneas horizontales
        cv2.line(imagen1, (0, i * tamano_seccion_alto), (ancho, i * tamano_seccion_alto), (0, 255, 0), 2)

        # Líneas verticales
        cv2.line(imagen1, (i * tamano_seccion_ancho, 0), (i * tamano_seccion_ancho, alto), (0, 255, 0), 2)

    # Mostrar la imagen 1 con las secciones y los puntos clave resaltados
    cv2.imshow("Imagen 1 con secciones y puntos clave", imagen1)
    # Obtener la sección con más matches
    indice_max_matches = np.unravel_index(np.argmax(conteo_matches_secciones), conteo_matches_secciones.shape)
    fila_max_matches, columna_max_matches = indice_max_matches

    if conteo_matches_secciones[fila_max_matches, columna_max_matches] > 4:
        print("Sección con más matches:", fila_max_matches, columna_max_matches)
        break
    else:
        # Segmentar la imagen en secciones más pequeñas
        tamano_seccion_alto //= 2
        tamano_seccion_ancho //= 2
        print("Segmentando la imagen nuevamente...")

    # Esperar por una tecla
    cv2.waitKey(0)
    cv2.destroyAllWindows()