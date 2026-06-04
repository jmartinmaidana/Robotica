# AGENT GUIDANCE

This document provides high-signal context for OpenCode agents working on this repository.

## Repository Type

ROS 2 Workspace.

## Build

To build the entire workspace:
```bash
colcon build
```

## Run Tests

To run tests for the entire workspace:
```bash
colcon test
```

## Package-Specific Notes

### `ros2-keyboard`

*   **System Dependency:** Requires `libsdl1.2-dev`. Install with `sudo apt install libsdl1.2-dev`.
*   **Usage:** The keyboard input window *must* be focused to receive key presses.
*   **Example Run Commands:**
    *   Start `keyboard` node:
        ```bash
        ros2 run keyboard keyboard
        ```
    *   Start `keyboard_to_joy.py` node with config:
        ```bash
        ros2 run keyboard keyboard_to_joy.py --ros-args \
          -p config_file_name:=`ros2 pkg prefix keyboard`/share/keyboard/config/example_config.yaml
        ```

### `kfilter`

*   **Installation (Unix/Linux):**
    ```bash
    make
    sudo make install
    ```
    This installs `libkalman.a` to `/usr/local/lib` and includes to `/usr/local/include/kalman`.
*   **Linking:** When using `kfilter`, link with `-lkalman`.
*   **Documentation:** HTML documentation is available at `doc/public/html/index.html` within the `kfilter` package directory.

### `ej1` (Modelo Cinemático y Odometría del Robot Omnidireccional)

*   **Propósito:** Implementa el puente cinemático para un chasis móvil holonómico de 4 ruedas Mecanum (Pioneer).
*   **Cinemática Inversa:**
    *   Suscripción a `/robot/cmd_vel` (`geometry_msgs/msg/Twist`).
    *   Mapea velocidades locales ($V_x$, $V_y$, $\omega$) a velocidades individuales de ruedas mediante la geometría del robot (`WHEEL_RADIUS`, `L_X`, `L_Y`).
    *   Publica consignas individuales `Float64` en los tópicos `/robot/front_left_wheel/cmd_vel`, etc.
*   **Cinemática Directa (Odometría):**
    *   Suscripción a `/robot/encoders` (`robmovil_msgs/msg/MultiEncoderTicks`).
    *   Convierte incrementos de encoders ($\Delta\text{ticks}$) a distancias métricas (resolución de 500 ticks por revolución).
    *   Actualiza de manera incremental la pose global $(x, y, \theta)$ proyectando los desplazamientos locales y acumulándolos.
    *   Publica la odometría en `/robot/odometry` y difunde la transformación estática TF `odom -> base_link`.

### `ej2` (Control a Lazo Cerrado y Seguimiento de Trayectorias)

*   **Propósito:** Implementa el control de tracking y seguimiento a lazo cerrado con selección de objetivos para el robot holonómico.
*   **Estrategia de Persecución (Pure Pursuit):**
    *   La pose objetivo de tracking se obtiene dinámicamente mediante `getPursuitBasedGoal`.
    *   Avanza por los waypoints de la trayectoria cargada mientras la distancia sea menor que `LOOKAHEAD = 0.5` metros.
    *   Finaliza el recorrido de la trayectoria con éxito cuando se llega al último waypoint con una distancia inferior a `TOLERANCE = 0.05` metros.
*   **Controlador de Posición Proporcional (P):**
    *   Suscripción a TF `map -> base_link` para feedback de pose actual.
    *   Calcula el error global ($dx, dy, d\theta$) respecto al objetivo actual y lo rota al marco local del robot (`base_link`):
        $$e_x = dx \cos(\theta) + dy \sin(\theta)$$
        $$e_y = -dx \sin(\theta) + dy \cos(\theta)$$
    *   Genera comandos proporcionales locales de control: $V_x = K_{PX} e_x$, $V_y = K_{PY} e_y$, $\omega = K_{P\theta} d\theta$.
*   **Trayectoria Cuadrada con Orientación "Opuesta al Centro":**
    *   Definida en `trajectory_generator_node.cpp` mediante corners $(\pm 2, \pm 2)$ de $4\text{m}$ de lado.
    *   Mantiene la orientación radial hacia afuera mediante interpolación angular lineal, equivalente a:
        $$\theta = \text{atan2}(y, x) + 45^\circ$$
        Esto alinea el eje delantero del robot radialmente opuesto al origen $(0,0)$.
*   **Logger Experimental:**
    *   Genera archivos `.log` formateados con marcas temporales y coordenadas en tiempo real para graficar la odometría vs. el valor real y evaluar el desempeño del chasis.
