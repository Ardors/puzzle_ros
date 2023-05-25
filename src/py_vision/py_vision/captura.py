import cv2

# Capturar una imagen de la cámara
#cap = cv2.VideoCapture("http://192.168.43.1:8080/video")
cap = cv2.VideoCapture(4)
ret, frame = cap.read()

# Verificar si la captura fue exitosa
if not ret:
    print("No se pudo obtener una imagen de la cámara.")
    cap.release()
    cv2.destroyAllWindows()
    exit()

# Guardar la imagen en un archivo
cv2.imwrite("captura.png", frame)
print("Imagen guardada como captura.png")

# Liberar los recursos
cap.release()
cv2.destroyAllWindows()