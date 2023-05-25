import cv2
import numpy as np

# Cargar las imágenes
imagen1 = cv2.imread("PuzzleJ.jpeg")
imagen2 = cv2.imread("prueba.png")

# Dividir la imagen 1 en 3x3 partes
alto, ancho, _ = imagen1.shape
tamano_parte_alto = alto // 3
tamano_parte_ancho = ancho // 3
partes_imagen1 = []

for i in range(3):
    for j in range(3):
        parte = imagen1[i * tamano_parte_alto:(i + 1) * tamano_parte_alto, j * tamano_parte_ancho:(j + 1) * tamano_parte_ancho]
        partes_imagen1.append(parte)

# Recorrer cada subfigura y comparar con la imagen 2
for subfigura in partes_imagen1:
    # Convertir la subfigura a escala de grises
    subfigura_gris = cv2.cvtColor(subfigura, cv2.COLOR_BGR2GRAY)

    # Redimensionar la imagen 2 al tamaño de la subfigura
    imagen2_redimensionada = cv2.resize(imagen2, (subfigura.shape[1], subfigura.shape[0]))

    # Convertir la imagen 2 redimensionada a escala de grises
    imagen2_gris = cv2.cvtColor(imagen2_redimensionada, cv2.COLOR_BGR2GRAY)

    # Calcular la diferencia absoluta entre los valores de los píxeles
    diferencia = np.abs(subfigura_gris.astype(np.float32) - imagen2_gris.astype(np.float32))

    # Calcular el porcentaje de error
    porcentaje_error = np.mean(diferencia) / 255.0 * 100

    # Imprimir el porcentaje de error
    print("Porcentaje de error de similitud:", porcentaje_error)
