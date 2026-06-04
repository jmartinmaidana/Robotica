#!/bin/bash

# 1. Iniciar Docker
echo "Iniciando contenedor ros2_robotica..."
bash start-docker.sh start
sleep 3 # Espera para que el contenedor levante correctamente

CONTENEDOR="ros2_robotica"

# 2. Abrir Konsole e inyectar comandos
# Pestaña 1: CoppeliaSim y Compilación (Forzamos source de ROS2 Humble)
konsole --new-tab -p tabtitle="CoppeliaSim" -e bash -c "docker exec -it $CONTENEDOR bash -c 'source /opt/ros/humble/setup.bash && bash ros2_ws/src/robotica/actualizar_archivos.sh && cd ros2_ws && source install/setup.bash && ./coppeliaSim.sh'; exec bash" &

# Le damos 2 segundos enteros para garantizar que esta ventana sea la principal
sleep 2

# Pestaña 2: Rviz2
konsole --new-tab -p tabtitle="Rviz2" -e bash -c "docker exec -it $CONTENEDOR bash -c 'source /opt/ros/humble/setup.bash && cd ros2_ws && source install/setup.bash && rviz2'; exec bash" &

sleep 1

# Pestaña 3: Terminal libre (Usamos -ic para forzar una sesión interactiva que lea el .bashrc)
konsole --new-tab -p tabtitle="Terminal ROS2" -e bash -c "docker exec -it $CONTENEDOR bash -ic 'exec bash'" &