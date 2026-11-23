"""Práctica 2: ingesta de imágenes (tarea 2) y corrección con la calibración (tarea 4)."""

import os
from typing import List, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import yaml

TOPIC_IMAGEN: str = '/image_raw'
TAMANO_COLA: int = 10
# Formato de color que usa OpenCV: azul, verde y rojo, 8 bits por canal
CODIFICACION_OPENCV: str = 'bgr8'
PERIODO_AVISO_S: float = 2.0  # como mucho un aviso repetido cada 2 segundos

# --- Tarea 2 ---
NUMERO_IMAGENES: int = 10
CARPETA_IMAGENES: str = os.path.expanduser('~/ros_workspace/imagenes_practica2')
FORMATO_NOMBRE_IMAGEN: str = 'imagen_{:02d}.png'  # imagen_01.png, imagen_02.png...
# Diferencia media mínima entre píxeles (escala 0-255) para considerar dos imágenes
# distintas. No se compara con 0 porque dos imágenes seguidas de una cámara quieta
# nunca son idénticas: el ruido del sensor cambia ligeramente cada píxel
UMBRAL_DIFERENCIA: float = 25.0

# --- Tarea 4 ---
TOPIC_IMAGEN_CORREGIDA: str = '/image_corregida'
PAQUETE: str = 'robotica_practica2'
ARCHIVO_CALIBRACION: str = 'calibracion_camara.yaml'  # generado en la tarea 3
# Al corregir la distorsión de barril, los bordes de la imagen quedan curvados hacia
# dentro y aparecen zonas negras. ALPHA decide qué hacer con ellas: 0 recorta la
# imagen para que no haya zonas negras; 1 conserva todos los píxeles originales
ALPHA_CORRECCION: float = 0.0


# TAREA 2: suscriptor e ingesta de imágenes
class IngestaImagenes(Node):
    """Guarda las 10 primeras imágenes de la cámara que sean diferentes entre sí."""

    def __init__(self) -> None:
        """Crea la carpeta de destino y se suscribe al topic de la cámara."""
        super().__init__('ingesta_imagenes')
        os.makedirs(CARPETA_IMAGENES, exist_ok=True)
        # Puente entre los mensajes Image de ROS y las imágenes (matrices) de OpenCV
        self._puente: CvBridge = CvBridge()
        self._imagenes_grises: List[np.ndarray] = []  # las ya guardadas, en gris
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
        """Convierte la imagen a OpenCV y la guarda si es diferente de las anteriores."""
        if self.terminado:
            return
        imagen: np.ndarray = self._puente.imgmsg_to_cv2(
            mensaje, desired_encoding=CODIFICACION_OPENCV
        )
        # Una imagen vacía (la cámara no ha enviado datos) no se puede procesar
        if imagen.size == 0:
            self.get_logger().warn(
                'Imagen vacía recibida: se ignora', throttle_duration_sec=PERIODO_AVISO_S
            )
            return
        if self._es_diferente(imagen):
            self._guardar(imagen)

    def _es_diferente(self, imagen: np.ndarray) -> bool:
        """Comprueba que la imagen se diferencia de todas las ya guardadas."""
        # Se compara en escala de grises: basta con la luminosidad y es más rápido
        gris: np.ndarray = cv2.cvtColor(imagen, cv2.COLOR_BGR2GRAY)
        for guardada in self._imagenes_grises:
            diferencia: float = float(np.mean(cv2.absdiff(gris, guardada)))
            if diferencia <= UMBRAL_DIFERENCIA:
                return False  # se parece demasiado a una que ya está guardada
        return True  # la primera imagen siempre pasa, porque la lista está vacía

    def _guardar(self, imagen: np.ndarray) -> None:
        """Guarda la imagen en disco con un nombre numerado."""
        self._imagenes_guardadas += 1
        nombre: str = FORMATO_NOMBRE_IMAGEN.format(self._imagenes_guardadas)
        ruta: str = os.path.join(CARPETA_IMAGENES, nombre)
        cv2.imwrite(ruta, imagen)
        self._imagenes_grises.append(cv2.cvtColor(imagen, cv2.COLOR_BGR2GRAY))
        self.get_logger().info(
            f'Imagen {self._imagenes_guardadas}/{NUMERO_IMAGENES} guardada en {ruta}'
        )


# TAREA 4: aplicación de la calibración con OpenCV
class CorrectorImagen(Node):
    """Corrige la distorsión de la imagen de la cámara con los parámetros de calibración."""

    def __init__(self) -> None:
        """Carga la calibración, se suscribe a la cámara y crea el publicador corregido."""
        super().__init__('corrector_imagen')
        # Ruta de la calibración como parámetro de ROS: por defecto, la del paquete
        ruta_por_defecto: str = os.path.join(
            get_package_share_directory(PAQUETE), 'config', ARCHIVO_CALIBRACION
        )
        ruta: str = self.declare_parameter('calibracion', ruta_por_defecto).value
        self._matriz_camara: np.ndarray
        self._distorsion: np.ndarray
        self._tamano_calibracion: Tuple[int, int]
        self._cargar_calibracion(ruta)
        self._puente: CvBridge = CvBridge()
        # Matriz de la cámara tras corregir; depende del tamaño de imagen, así que
        # se calcula una sola vez, al llegar la primera imagen
        self._matriz_corregida: Optional[np.ndarray] = None
        self._publicador = self.create_publisher(Image, TOPIC_IMAGEN_CORREGIDA, TAMANO_COLA)
        self.create_subscription(Image, TOPIC_IMAGEN, self._recibir_imagen, TAMANO_COLA)
        self.get_logger().info(
            f'Calibración cargada de {ruta}; publicando la imagen corregida en '
            f'{TOPIC_IMAGEN_CORREGIDA}'
        )

    def _cargar_calibracion(self, ruta: str) -> None:
        """Lee la matriz de la cámara y la distorsión del YAML de camera_calibration."""
        with open(ruta, 'r') as archivo:
            datos: dict = yaml.safe_load(archivo)
        # En el YAML las matrices vienen como listas planas: se les da su forma
        self._matriz_camara = np.array(
            datos['camera_matrix']['data'], dtype=np.float64
        ).reshape(3, 3)
        self._distorsion = np.array(
            datos['distortion_coefficients']['data'], dtype=np.float64
        )
        self._tamano_calibracion = (datos['image_width'], datos['image_height'])

    def _recibir_imagen(self, mensaje: Image) -> None:
        """Convierte la imagen a OpenCV, la corrige y la publica de nuevo en ROS."""
        imagen: np.ndarray = self._puente.imgmsg_to_cv2(
            mensaje, desired_encoding=CODIFICACION_OPENCV
        )
        if imagen.size == 0:
            self.get_logger().warn(
                'Imagen vacía recibida: se ignora', throttle_duration_sec=PERIODO_AVISO_S
            )
            return
        corregida: np.ndarray = self._corregir(imagen)
        salida: Image = self._puente.cv2_to_imgmsg(corregida, encoding=CODIFICACION_OPENCV)
        salida.header = mensaje.header  # misma marca de tiempo que la imagen original
        self._publicador.publish(salida)

    def _corregir(self, imagen: np.ndarray) -> np.ndarray:
        """Elimina la distorsión de la lente con cv2.undistort."""
        alto, ancho = imagen.shape[:2]
        if self._matriz_corregida is None:
            # La calibración solo es válida para la resolución con la que se hizo
            if (ancho, alto) != self._tamano_calibracion:
                self.get_logger().warn(
                    f'La imagen es de {ancho}x{alto} pero la calibración es de '
                    f'{self._tamano_calibracion[0]}x{self._tamano_calibracion[1]}'
                )
            self._matriz_corregida, _ = cv2.getOptimalNewCameraMatrix(
                self._matriz_camara, self._distorsion, (ancho, alto),
                ALPHA_CORRECCION, (ancho, alto)
            )
        return cv2.undistort(
            imagen, self._matriz_camara, self._distorsion, None, self._matriz_corregida
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


def main_corregir() -> None:
    """Tarea 4: corrige continuamente la imagen de la cámara hasta pulsar Ctrl+C."""
    rclpy.init()
    nodo = CorrectorImagen()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    nodo.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main_ingesta()
