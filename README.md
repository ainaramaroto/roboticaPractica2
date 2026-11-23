# Práctica 2 - Ingesta, calibración y procesamiento de imágenes con ROS 2

**Autores:** Iñigo Fiat Sarasola y Ainara Maroto Fernández

## Archivos del proyecto

Tenemos un paquete, **`robotica_practica2`**:
- `robotica_practica2/practica2.py`: ingesta de imágenes (Tarea 2) y corrección de la imagen (Tarea 4).
- `launch/camara.launch.py`: lanza la cámara con `usb_cam` y el visor de imágenes (Tarea 1).
- `config/usb_cam.yaml`: parámetros de la cámara.
- `config/calibracion_camara.yaml`: resultado de la calibración (Tarea 3).

En la carpeta `imagenes/` están las 10 imágenes guardadas y la comparación de la imagen antes y después de corregirla.

## Compilación

Desde el contenedor:

```bash
sudo apt update && sudo apt install -y ros-jazzy-camera-calibration ros-jazzy-image-publisher
cd ~/ros_workspace
colcon build --symlink-install --packages-select robotica_practica2
source install/setup.bash
```

El comando `source install/setup.bash` hay que ejecutarlo en cada terminal nueva.

## Ejecución

Primero hay que tener la imagen de la cámara en el topic `/image_raw`.

**En el laboratorio**, con `./lab.sh iniciar --lab`, se lanza la webcam con `usb_cam`:

```bash
ros2 launch robotica_practica2 camara.launch.py
```

**Desde nuestro portátil**, el contenedor no puede acceder a la webcam, así que la emitimos desde Windows con `ffmpeg` y la publicamos en ROS con `image_publisher`:

```powershell
ffmpeg -f dshow -rtbufsize 20M -vcodec mjpeg -video_size 1280x720 -framerate 30 -i video="HD USB Camera" -c:v copy -f mpjpeg -listen 1 http://0.0.0.0:5055/camara.mjpg
```

```bash
ros2 run image_publisher image_publisher_node http://host.docker.internal:5055/camara.mjpg --ros-args -p publish_rate:=60.0 -p flip_vertical:=true -p flip_horizontal:=true
```

Después, en otra terminal, se ejecuta la parte que se quiera probar:

| Parte | Comando |
|---|---|
| Tarea 1: publicador de imágenes | `ros2 launch robotica_practica2 camara.launch.py` |
| Tarea 2: ingesta de 10 imágenes | `ros2 run robotica_practica2 ingesta` |
| Tarea 3: calibración | `ros2 run camera_calibration cameracalibrator --size 9x7 --square 0.020 --no-service-check --ros-args -r image:=/image_raw` |
| Tarea 4: corrección de la imagen | `ros2 run robotica_practica2 corregir` |

Las imágenes se guardan en `~/ros_workspace/imagenes_practica2/` y la comparación en `~/ros_workspace/comparacion_practica2/`.

## Parte 1

**Tarea 1.** Publicar las imágenes de la cámara con `usb_cam` y mostrarlas. Hecho con un launch que arranca el nodo de `usb_cam`, que publica la imagen en `/image_raw`, y abre `rqt_image_view` para verla.

**Tarea 2.** Guardar 10 imágenes seguidas y diferentes entre sí. Hecho con un nodo suscrito a `/image_raw` que convierte cada imagen a OpenCV con `cv_bridge`. Para saber si una imagen es diferente, la pasamos a escala de grises y calculamos la diferencia media de píxeles con cada una de las ya guardadas. Solo se guarda si supera un umbral con todas. No usamos 0 como límite porque, aunque la cámara esté quieta, dos imágenes nunca son exactamente iguales.

**Tarea 3.** Calibrar la cámara con `camera_calibration`. Hecho con un tablero de ajedrez de 9x7 esquinas interiores y cuadrados de 2 cm, moviendo la cámara hasta llenar las barras X, Y, Size y Skew. El resultado se guarda en `calibracion_camara.yaml` y `usb_cam` lo carga para publicarlo en `/camera_info`.

| Parámetro | Valor |
|---|---|
| Distancia focal (fx, fy) | 1640.5, 1644.6 px |
| Centro óptico (cx, cy) | 648.9, 403.4 px |
| Distorsión (k1, k2, p1, p2, k3) | -0.410, 0.302, -0.001, -0.001, 0.0 |

**Tarea 4.** Aplicar la calibración a la imagen con OpenCV. Hecho con un nodo que lee la calibración, convierte cada imagen con `cv_bridge` y corrige la distorsión. Publica la imagen corregida en `/image_corregida` y la original y la corregida lado a lado en `/image_comparacion`.

## Observaciones

- En Windows, Docker no puede acceder a la webcam porque el núcleo de WSL 2 no tiene drivers de vídeo. Por eso, en nuestros portátiles usamos `ffmpeg` e `image_publisher` en lugar de `usb_cam`.
- Nuestra webcam da la imagen boca abajo, así que la giramos 180° con los `flip` de `image_publisher`.
- La calibración solo vale para la resolución con la que se hizo (1280x720).
- La primera calibración nos salió mal, porque la imagen corregida se curvaba hacia fuera. Faltaban muestras inclinadas y en las esquinas, y la repetimos inclinando más la cámara.

## Reparto del trabajo

Hemos realizado la práctica entre los dos, trabajando juntos todas las tareas.
Para mantener limpio el historial de GitHub, hemos intentado que los commits queden lo más ordenados posible.
Como solo Iñigo tenía webcam, las pruebas con la cámara las ha hecho él.
Que un commit figure a nombre de uno de nosotros, no significa que esa parte la haya hecho solo.

## Uso de la IA

Hemos utilizado un asistente de IA (Claude) como apoyo durante la práctica. Nos ha servido para entender los conceptos, para resolver problemas del entorno, como acceder a la webcam desde Docker en Windows y, en algunas ocasiones, para definir cierta funcionalidad.

En ningún momento nos ha facilitado el código entero, el código lo hemos escrito nosotros. La IA ha sido solo una herramienta de apoyo y no la hemos usado para sustituir nuestro criterio.