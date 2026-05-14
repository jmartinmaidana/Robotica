# Guía de Docker para Dummies (ROS 2 & CoppeliaSim)

## 1. Configuración Inicial de Docker (PARA PC DE LABO)
En una terminal abierta en `Escritorio/Docker`, verifica el socket de Docker:
```bash
ls -l /run/user/$(id -u)/docker.sock
```
Si el comando anterior da error, inicia el demonio rootless en segundo plano:
```bash
dockerd-rootless.sh &
```
Luego, exporta la variable de entorno:
```bash
export DOCKER_HOST=unix:///run/user/$(id -u)/docker.sock
```
> **NOTA para Notebook:** Ejecuta `newgrp docker` para recargar tus credenciales de grupo y evitar usar `sudo` en los comandos de Docker.

## 2. Iniciar y Entrar al Contenedor 
En la carpeta que contenga volume, levanta el contenedor en segundo plano (build antes si hace falta):
```bash
bash start-docker.sh start
```
Abre una terminal interactiva dentro del contenedor:
```bash
bash start-docker.sh open
```
*¡Listo! Ya estás dentro del contenedor compartiendo el volumen.* Cambia al directorio del workspace de ROS 2:
```bash
cd ros2_ws/
```

## 3. Configurar y Compilar Workspace
Para editar los archivos, inyectar los mensajes/dependencias y compilar el workspace, ejecuta el script de actualización:
```bash
bash src/robotica/actualizar_coppelia.sh
```
Y luego, recuerda siempre cargar las variables de entorno de tu workspace:
```bash
source install/setup.bash
```

## 4. Lanzar CoppeliaSim
Abre el simulador directamente desde la terminal del contenedor:
```bash
coppeliaSim.sh
```
Dentro de Coppelia, ve a **File > Open Scene**, navega por la ruta `/root/ros2_ws/src/robotica/coppeliaSim`, y abre el archivo `omni_ekf.ttt` (o la escena que vayas a usar).

## 5. Ejecutar Nodos de ROS 2
Para mover el robot con el teclado, abre **otra terminal nueva** (`bash start-docker.sh open`), ejecuta el `source install/setup.bash` y corre:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot/cmd_vel
```
En **otra terminal adicional** (con su respectivo `source`), inicia el nodo de la odometría:
```bash
ros2 run ej1 pioneer_odometry_node
```
