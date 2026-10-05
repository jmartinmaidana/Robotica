# Resumen del Código pioneer_odometry.cpp

Este archivo implementa el puente entre los comandos de alto nivel de ROS 2 y las ruedas del robot móvil holonómico (omnidireccional). Se encarga de dos tareas fundamentales:

1. **Cinemática Inversa:** Traducir las velocidades deseadas del chasis (adelante, a los lados, rotación) en velocidades individuales para cada una de las 4 ruedas.
2. **Cinemática Directa (Odometría):** Leer los sensores de las ruedas (encoders) para estimar y publicar la posición y orientación actual del robot en el mundo.

---

## ¿Cómo funciona el código paso a paso?

### 1. Cinemática Inversa (Control de Movimiento)
* **Suscripción:** El nodo se suscribe al tópico `/robot/cmd_vel` para recibir mensajes de tipo `Twist` (velocidad lineal y angular).
* **Procesamiento (`on_velocity_cmd`):** Extrae la velocidad lineal en $X$ (avance), la velocidad lineal en $Y$ (desplazamiento lateral) y la velocidad angular en $Z$ (rotación).
* **Cálculo:** Utiliza las ecuaciones matemáticas del modelo cinemático del robot omnidireccional tomando en cuenta la distancia entre ruedas (`L_X` y `L_Y`) y el radio de las mismas (`WHEEL_RADIUS`).
* **Publicación:** Calcula las velocidades de las 4 ruedas (`vFront_left`, `vFront_right`, `vRear_left`, `vRear_right`) y las publica individualmente como `Float64` en sus respectivos tópicos (ej. `/robot/front_left_wheel/cmd_vel`) para que el simulador mueva los motores.

### 2. Cinemática Directa (Cálculo de Odometría)
* **Suscripción:** Se suscribe al tópico `/robot/encoders`, que le envía los "ticks" o pulsos registrados por el sensor de giro de cada una de las 4 ruedas.
* **Procesamiento (`on_encoder_ticks`):** 
  * Calcula cuántos ticks avanzó cada rueda desde la última lectura (`delta_ticks`).
  * Convierte esos ticks a una distancia métrica real usando el perímetro de la rueda (`WHEEL_RADIUS * 2 * PI`) y la resolución del encoder (`ENCODER_TICKS` = 500).
* **Actualización de Posición:**
  * Usando los desplazamientos de las 4 ruedas, calcula el movimiento relativo del centro del robot ($\Delta X_{local}$, $\Delta Y_{local}$, $\Delta \theta$).
  * Luego, rota este desplazamiento local al sistema de coordenadas global usando el ángulo $\theta$ actual del robot y suma el resultado a sus coordenadas globales `x_`, `y_` y `theta_`.
* **Publicación (TF y Odometría):** 
  * Empaqueta esta nueva pose y las velocidades calculadas en un mensaje estándar de ROS 2 (`nav_msgs::msg::Odometry`) y lo publica en `/robot/odometry`.
  * Publica una transformación estática (`TF`) entre el marco de referencia del mundo (`odom`) y el marco de referencia del robot (`base_link`), lo cual es indispensable para visualizar el robot en herramientas como RViz.

---

## En el contexto del proyecto

El flujo completo utilizando Docker y CoppeliaSim es el siguiente:

1. Se inicia el contenedor de Docker con el espacio de trabajo montado como un **volumen** y se abre la escena de CoppeliaSim (`omni_ekf.ttt`).
2. Al ejecutar `ros2 run ej1 pioneer_odometry_node`, se lanza este nodo.
3. Mientras el robot corre en Coppelia, el nodo lee constantemente los encoders simulados para calcular dónde está el robot en el espacio.
4. Simultáneamente, escucha los comandos del teclado (vía `teleop_twist_keyboard`) para traducir la intención de movimiento a velocidades de giro reales de cada una de las 4 ruedas omnidireccionales en la simulación.