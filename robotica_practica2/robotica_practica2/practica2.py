"""Práctica 2: ingesta de imágenes (tarea 2) y corrección con la calibración (tarea 4)."""

import os
from typing import Optional

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

TOPIC_IMAGEN: str = '/image_raw'
TAMANO_COLA: int = 10
# Formato de color que usa OpenCV: azul, verde y rojo, 8 bits por canal
CODIFICACION_OPENCV: str = 'bgr8'

NUMERO_IMAGENES: int = 10
CARPETA_IMAGENES: str = os.path.expanduser('~/ros_workspace/imagenes_practica2')
FORMATO_NOMBRE_IMAGEN: str = 'imagen_{:02d}.png'  # imagen_01.png, imagen_02.png...
# Diferencia media mínima entre píxeles (escala 0-255) para considerar dos imágenes
# distintas. No se compara con 0 porque dos imágenes seguidas de una cámara quieta
# nunca son idénticas: el ruido del sensor cambia ligeramente cada píxel
UMBRAL_DIFERENCIA: float = 8.0


# TAREA 2: suscriptor e ingesta de imágenes
class IngestaImagenes(Node):
    """Guarda las 10 primeras imágenes de la cámara que sean diferentes entre sí."""

    def __init__(self) -> None:
        """Crea la carpeta de destino y se suscribe al topic de la cámara."""
        super().__init__('ingesta_imagenes')
        os.makedirs(CARPETA_IMAGENES, exist_ok=True)
        # Puente entre los mensajes Image de ROS y las imágenes (matrices) de OpenCV
        self._puente: CvBridge = CvBridge()
        self._ultima_guardada: Optional[np.ndarray] = None
        self._imagenes_guardadas: int = 0
        self.create_subscription(Image, TOPIC_IMAGEN, self._recibir_imagen, TAMANO_COLA)
        self.get_logger().info(
            f'Esperando imágenes en {TOPIC_IMAGEN}; se guardarán en {CARPETA_IMAGENES}'
        )

    @property
    def terminado(self) -> bool:
        """Indica si ya se han guardado todas las imágenes."""
        return self._imagenes_guardadas >= NUMERO_IMAGENES

    def _recibir_imagen(self, mensaje: Image) -> None:
        """Convierte la imagen a OpenCV y la guarda si es diferente de la anterior."""
        if self.terminado:
            return
        imagen: np.ndarray = self._puente.imgmsg_to_cv2(
            mensaje, desired_encoding=CODIFICACION_OPENCV
        )
        if self._es_diferente(imagen):
            self._guardar(imagen)

    def _es_diferente(self, imagen: np.ndarray) -> bool:
        """Compara la imagen con la última guardada usando la diferencia media de píxeles."""
        if self._ultima_guardada is None:
            return True  # la primera imagen siempre se guarda
        # Se compara en escala de grises: basta con la luminosidad y es más rápido
        gris_actual: np.ndarray = cv2.cvtColor(imagen, cv2.COLOR_BGR2GRAY)
        gris_anterior: np.ndarray = cv2.cvtColor(self._ultima_guardada, cv2.COLOR_BGR2GRAY)
        diferencia: float = float(np.mean(cv2.absdiff(gris_actual, gris_anterior)))
        return diferencia > UMBRAL_DIFERENCIA

    def _guardar(self, imagen: np.ndarray) -> None:
        """Guarda la imagen en disco con un nombre numerado."""
        self._imagenes_guardadas += 1
        nombre: str = FORMATO_NOMBRE_IMAGEN.format(self._imagenes_guardadas)
        ruta: str = os.path.join(CARPETA_IMAGENES, nombre)
        cv2.imwrite(ruta, imagen)
        self._ultima_guardada = imagen
        self.get_logger().info(
            f'Imagen {self._imagenes_guardadas}/{NUMERO_IMAGENES} guardada en {ruta}'
        )


def main_ingesta() -> None:
    """Tarea 2: guarda 10 imágenes diferentes de la cámara y termina."""
    rclpy.init()
    nodo = IngestaImagenes()
    try:
        # spin_once en lugar de spin para poder parar al llegar a las 10 imágenes
        while rclpy.ok() and not nodo.terminado:
            rclpy.spin_once(nodo)
    except KeyboardInterrupt:
        pass
    nodo.get_logger().info('Ingesta terminada')
    nodo.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main_ingesta()
    