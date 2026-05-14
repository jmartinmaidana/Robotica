#!/bin/bash

echo "🚀 Configurando mensajes y dependencias para sim_ros2_interface..."

echo "🔄 Limpiando modificaciones anteriores..."
cd /root/ros2_ws/src/sim_ros2_interface || exit
git restore CMakeLists.txt package.xml meta/interfaces.txt

# 1. Agregar los mensajes a interfaces.txt
echo "robmovil_msgs/msg/EncoderTicks" >> /root/ros2_ws/src/sim_ros2_interface/meta/interfaces.txt
echo "robmovil_msgs/msg/MultiEncoderTicks" >> /root/ros2_ws/src/sim_ros2_interface/meta/interfaces.txt
echo "rosgraph_msgs/msg/Clock" >> /root/ros2_ws/src/sim_ros2_interface/meta/interfaces.txt

# 2. Agregar la dependencia en package.xml
sed -i '/<depend>image_transport<\/depend>/a \ \ <depend>robmovil_msgs<\/depend>' /root/ros2_ws/src/sim_ros2_interface/package.xml

# 3. Agregar las dependencias en CMakeLists.txt
sed -i '/find_package(image_transport)/a find_package(robmovil_msgs REQUIRED)' /root/ros2_ws/src/sim_ros2_interface/CMakeLists.txt
sed -i '/^    pendulum_msgs/a \ \ \ \ robmovil_msgs' /root/ros2_ws/src/sim_ros2_interface/CMakeLists.txt

echo "🔨 Compilando el workspace..."
cd /root/ros2_ws || exit
colcon build

echo "================================================================"
echo "✅ ¡Todo listo y compilado con éxito!"
echo "⚠️  IMPORTANTE: Ahora ejecuta: source install/setup.bash"
echo "================================================================"
