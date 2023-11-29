import cv2
import numpy as np
import pyttsx3
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

# Inicializar el motor de voz
motor = pyttsx3.init()

# Obtener las voces disponibles
voces = motor.getProperty('voices')

# Buscar una voz en español
for voz in voces:
    if 'spanish' in voz.languages:
        motor.setProperty('voice', voz.id)
        break

# Definir una función para hablar
def hablar(texto):
    motor.say(texto)
    motor.runAndWait()

# Tamaño conocido del semáforo en la vida real (en metros)
TAMANO_REAL = 1.0  # Ajusta este valor al tamaño real del semáforo

# Distancia conocida desde la cámara al semáforo (en metros)
DISTANCIA_CONOCIDA = 10.0  # Ajusta este valor a la distancia real desde la cámara al semáforo

# Tamaño del semáforo en la imagen (en píxeles) a una distancia conocida
TAMANO_IMAGEN = 100.0  # Ajusta este valor al tamaño del semáforo en la imagen a una distancia conocida

# Calcular la "constante de distancia"
CONSTANTE_DISTANCIA = TAMANO_IMAGEN * DISTANCIA_CONOCIDA / TAMANO_REAL

def calcular_distancia(tamano_imagen):
    # Calcular y devolver la distancia desde la cámara al semáforo
    return CONSTANTE_DISTANCIA * TAMANO_REAL / tamano_imagen

def detectar_color(imagen):
    # Convertir la imagen a espacio de color HSV
    hsv = cv2.cvtColor(imagen, cv2.COLOR_BGR2HSV)

    # Definir rangos de colores para rojo, verde y amarillo
    rojo_bajo = np.array([0, 70, 50])
    rojo_alto = np.array([10, 255, 255])
    verde_bajo = np.array([36, 50, 70])
    verde_alto = np.array([86, 255, 255])
    amarillo_bajo = np.array([15, 150, 150])
    amarillo_alto = np.array([35, 255, 255])

    # Crear máscaras para cada color
    mascara_rojo = cv2.inRange(hsv, rojo_bajo, rojo_alto)
    mascara_verde = cv2.inRange(hsv, verde_bajo, verde_alto)
    mascara_amarillo = cv2.inRange(hsv, amarillo_bajo, amarillo_alto)

    # Verificar si el semáforo es de color rojo, verde o amarillo
    if mascara_rojo.any():
        return 'red'
    elif mascara_verde.any():
        return 'green'
    elif mascara_amarillo.any():
        return 'yellow'
    else:
        return 'unknown color'

def detectar_semaforos(imagen, archivo_cascade):
    # Cargar el clasificador en cascada
    semaforo_cascade = cv2.CascadeClassifier(archivo_cascade)

    # Convertir la imagen a escala de grises
    gris = cv2.cvtColor(imagen, cv2.COLOR_BGR2GRAY)

    # Detectar semáforos
    semaforos = semaforo_cascade.detectMultiScale(gris, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    # Dibujar un rectángulo alrededor de los semáforos y detectar su color
    for (x, y, w, h) in semaforos:
        cv2.rectangle(imagen, (x, y), (x+w, y+h), (0, 255, 0), 2)
        color_semaforo = detectar_color(imagen[y:y+h,x:x+w])

        # Calcular la distancia al semáforo y mostrarla si es menor que la distancia máxima
        distancia_semaforo = calcular_distancia(w)
        if distancia_semaforo <= DISTANCIA_CONOCIDA:
            texto_resultado = 'Traffic light in ' + color_semaforo + ' at ' + str(round(distancia_semaforo)) + ' meters'
            print(texto_resultado)
            hablar(texto_resultado)
            cv2.putText(imagen,texto_resultado,(x,y-10),cv2.FONT_HERSHEY_SIMPLEX,
                        0.9,(255,255,255),2,cv2.LINE_AA)

    return imagen

# Inicializar la cámara
camera = PiCamera()
camera.resolution = (320, 240)  # Reducir la resolución para mejorar el rendimiento
camera.framerate = 10  # Reducir la velocidad de fotogramas para mejorar el rendimiento
rawCapture = PiRGBArray(camera, size=(320, 240))

# Permitir que la cámara se caliente
time.sleep(0.1)

# Capturar cuadros de la cámara
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # Tomar la matriz NumPy que representa la imagen
    image = frame.array

    # Detectar los semáforos en la imagen
    resultado = detectar_semaforos(image,'Cascade_Semaforo.xml')

    # Mostrar el resultado
    cv2.imshow('Resultado', resultado)

    # Salir con 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # Limpiar el flujo en preparación para el siguiente cuadro
    rawCapture.truncate(0)

cv2.destroyAllWindows()
