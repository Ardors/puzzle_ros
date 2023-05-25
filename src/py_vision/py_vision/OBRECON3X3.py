import cv2
import numpy as np
import matplotlib.pyplot as plt

# Cargar la imagen 1 (la que se divide en 3x3)
imagen1 = cv2.imread("PuzzleJ.jpeg")

# Cargar la imagen de referencia
imagen_referencia = cv2.imread("1.jpeg")

# Dividir la imagen 1 en 3x3 partes
alto, ancho, _ = imagen1.shape
tamano_parte_alto = alto // 3
tamano_parte_ancho = ancho // 3
partes_imagen1 = []
for i in range(3):
    for j in range(3):
        parte = imagen1[i*tamano_parte_alto:(i+1)*tamano_parte_alto, j*tamano_parte_ancho:(j+1)*tamano_parte_ancho]
        partes_imagen1.append(parte)

# Crear el objeto ORB
orb = cv2.ORB_create(nfeatures=2000, scaleFactor=1.8, nlevels=10, edgeThreshold=11)

# Detectar los puntos clave y calcular los descriptores para la imagen de referencia
keypoints_referencia, descriptores_referencia = orb.detectAndCompute(imagen_referencia, None)

# Crear una figura y establecer el tamaño de las subfiguras
fig, axs = plt.subplots(3, 3, figsize=(8, 8), facecolor='black')

# Realizar el proceso de OBR para cada parte de la imagen 1 y la imagen de referencia
for i, parte in enumerate(partes_imagen1):
    # Detectar los puntos clave y calcular los descriptores para la parte de la imagen 1
    keypoints_parte, descriptores_parte = orb.detectAndCompute(parte, None)

    # Crear el objeto BFMatcher
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    # Realizar la comparación de los descriptores de la parte de la imagen 1 y la imagen de referencia
    matches = bf.match(descriptores_parte, descriptores_referencia)

    # Ordenar los matches por distancia
    matches = sorted(matches, key=lambda x: x.distance)

    # Dibujar los mejores matches en una imagen
    resultado = cv2.drawMatches(parte, keypoints_parte, imagen_referencia, keypoints_referencia, matches[:10], None, flags=2)

    # Mostrar la imagen con los matches en la subfigura correspondiente
    fila = i // 3
    columna = i % 3
    axs[fila, columna].set_facecolor('black')  # Establecer el fondo en negro
    axs[fila, columna].imshow(cv2.cvtColor(resultado, cv2.COLOR_BGR2RGB))
    axs[fila, columna].axis('off')
    axs[fila, columna].set_title(f"Parte {i+1} - Matches: {len(matches)}", color='white')  # Establecer el color del título en blanco

    # Imprimir el número de matches
    print("Número de matches para la parte", i+1, ":", len(matches))

# Mostrar la figura
plt.tight_layout()
plt.show()