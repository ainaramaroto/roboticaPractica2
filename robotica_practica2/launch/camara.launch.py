"""Tarea 1: lanza la cámara con usb_cam y muestra sus imágenes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PAQUETE: str = 'robotica_practica2'
ARCHIVO_PARAMETROS: str = 'usb_cam.yaml'
TOPIC_IMAGEN: str = '/image_raw'
DISPOSITIVO_POR_DEFECTO: str = '/dev/video0'


def generate_launch_description() -> LaunchDescription:
    """Crea el nodo de usb_cam y, opcionalmente, el visor de imágenes."""
    parametros: str = os.path.join(
        get_package_share_directory(PAQUETE), 'config', ARCHIVO_PARAMETROS
    )

    # Argumentos que se pueden cambiar al lanzar, sin tocar el código:
    # p. ej. dispositivo:=/dev/video2 si la webcam no es la video0
    argumento_dispositivo = DeclareLaunchArgument(
        'dispositivo', default_value=DISPOSITIVO_POR_DEFECTO,
        description='Dispositivo de la webcam'
    )
    argumento_ver = DeclareLaunchArgument(
        'ver_imagen', default_value='true',
        description='Abrir rqt_image_view para ver la imagen'
    )

    # Publicador: el nodo de usb_cam lee la webcam y publica en /image_raw.
    # El dispositivo indicado al lanzar sustituye al del archivo de parámetros.
    camara = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[parametros, {'video_device': LaunchConfiguration('dispositivo')}],
        output='screen',
    )

    # Visor para mostrar las imágenes que publica la cámara
    visor = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=[TOPIC_IMAGEN],
        condition=IfCondition(LaunchConfiguration('ver_imagen')),
    )

    return LaunchDescription([argumento_dispositivo, argumento_ver, camara, visor])
