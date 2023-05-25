import cv2
import matplotlib.pyplot as plt
import numpy as np

# Cargar la imagen
imagen1 = cv2.imread("Figure_1.png")
imagen2 = cv2.imread("5.jpeg")

# Crear el objeto ORB
orb = cv2.ORB_create(nfeatures=2000, scaleFactor=1.8, nlevels=11, edgeThreshold=10)
# Detectar los puntos clave y calcular los descriptores para ambas imágenes
keypoints1, descriptores1 = orb.detectAndCompute(imagen1, None)
keypoints2, descriptores2 = orb.detectAndCompute(imagen2, None)

# Crear el objeto BFMatcher
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# Realizar la comparación de los descriptores
matches = bf.match(descriptores1, descriptores2)

# Ordenar los matches por distancia
matches = sorted(matches, key=lambda x: x.distance)

# Dibujar los mejores matches en una imagen
resultado = cv2.drawMatches(imagen1, keypoints1, imagen2, keypoints2, matches[:10], None, flags=2)

# Convertir la imagen resultado a escala de grises
resultado_gris = cv2.cvtColor(resultado, cv2.COLOR_BGR2GRAY)

# Obtener las dimensiones de la imagen resultado
alto_resultado, ancho_resultado = resultado_gris.shape[:2]

# Calcular el tamaño de cada parte
tamano_parte_alto = alto_resultado // 3
tamano_parte_ancho = ancho_resultado // 3

# Dividir la imagen resultado en partes
partes_resultado = []
for i in range(3):
    for j in range(3):
        parte_resultado = resultado_gris[i * tamano_parte_alto:(i + 1) * tamano_parte_alto,
                          j * tamano_parte_ancho:(j + 1) * tamano_parte_ancho]
        partes_resultado.append(parte_resultado)

# Calcular la cantidad de matches en cada parte
matches_partes = []
for i in range(3):
    for j in range(3):
        parte_resultado = partes_resultado[i * 3 + j]

        # Obtener solo los matches que están dentro de la parte
        matches_parte = [match for match in matches if
                         tamano_parte_alto * i <= keypoints1[match.queryIdx].pt[1] <= tamano_parte_alto * (i + 1) and
                         tamano_parte_ancho * j <= keypoints1[match.queryIdx].pt[0] <= tamano_parte_ancho * (j + 1)]

        # Almacenar la cantidad de matches de la parte
        matches_partes.append(len(matches_parte))

# Identificar la parte con la mayor cantidad de matches
indice_max_matches = np.argmax(matches_partes)
parte_max_matches = partes_resultado[indice_max_matches]

# Convertir el índice en coordenadas fila-columna
fila = indice_max_matches // 3
columna = indice_max_matches % 3

# Mostrar la imagen con los matches
plt.imshow(cv2.cvtColor(resultado, cv2.COLOR_BGR2RGB))
plt.axis('off')

# Etiquetar la imagen1 dividida
for i in range(3):
    for j in range(3):
        x = j * tamano_parte_ancho
        y = i * tamano_parte_alto
        etiqueta = f"Parte {i+1}-{j+1}"

        # Dibujar un rectángulo en la parte
        rect = plt.Rectangle((x, y), tamano_parte_ancho, tamano_parte_alto, linewidth=2, edgecolor='red', facecolor='none')
        plt.gca().add_patch(rect)

        # Agregar etiqueta
        plt.text(x + 5, y + 20, etiqueta, color='white', fontsize=10, bbox=dict(facecolor='black', edgecolor='none', boxstyle='round,pad=0.2'))

plt.show()
