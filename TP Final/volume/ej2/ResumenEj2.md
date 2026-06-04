# Análisis del Ejercicio 2: Control a Lazo Cerrado y Seguimiento de Trayectoria

---

## 1. Requerimientos del Ejercicio (según el PDF)
El Ejercicio 2, **"Control a lazo cerrado y seguimiento de trayectoria"**, se enfoca en explotar las capacidades holonómicas (omnidireccionales) de un robot con 4 ruedas Mecanum para lograr un seguimiento preciso de trayectorias y convergencia punto a punto. Específicamente, los requerimientos son:
1. **Controlador Proporcional (P):** Plantear una ley de control proporcional para cada grado de libertad (DoF) para asegurar que la pose del robot $(x, y, \theta)$ converja a una pose objetivo. Se utiliza como feedback la estimación de pose provista por la odometría/transformaciones.
2. **Generación de Trayectoria Cuadrada:** Seguir una trayectoria cuadrada de $2\text{m}$ de lado (o $4\text{m}$ en coordenadas del mapa, según el gráfico) con la restricción específica de que la orientación del robot debe mantenerse **"opuesta al centro"** (apuntando radialmente hacia afuera del centro de la trayectoria cuadrada).
3. **Selección de Waypoints Basada en Persecución (Pursuit-Based):** Discretizar la trayectoria en intervalos de waypoints e implementar una estrategia de selección de objetivos para avanzar los puntos de consigna de manera sucesiva en función de la cercanía.
4. **Implementación en ROS 2:**
   - Publicar periódicamente comandos de velocidad en `/robot/cmd_vel`.
   - Utilizar la transformación de TF `map -> base_link` como la fuente de feedback.
   - Redefinir dinámicamente la pose objetivo actual.

---

## 2. Arquitectura de la Implementación en `volume/ej2/`

La estructura del paquete `ej2` organiza estas funcionalidades de forma limpia y modular:

```
volume/ej2/
├── include/ej2/
│   ├── TrajectoryFollower.h            # Clase base para seguidores de trayectoria
│   └── KinematicPositionController.h   # Controlador de seguimiento a lazo cerrado derivado
├── src/
│   ├── TrajectoryFollower.cpp
│   ├── KinematicPositionController.cpp # Lógica central: Control P y Pure Pursuit
│   ├── kinematic_position_controller_node.cpp # Nodo ejecutor del seguidor a lazo cerrado
│   ├── trajectory_generator_node.cpp   # Generador de trayectorias de referencia (Cuadrada, Seno, Spline)
│   ├── trajectory_waypoints_node.cpp   # Planificación alternativa manual de waypoints
│   └── logger_node.cpp                 # Utilidad de registro para generar gráficos de reportes
├── launch/
│   └── lazo_abierto.launch.py          # Archivo launch para arrancar los nodos en lazo cerrado
├── CMakeLists.txt
└── package.xml
```

---

## 3. Desglose Técnico del Funcionamiento

### A. Ley de Control y Transformación de Coordenadas
La lógica del controlador proporcional a lazo cerrado está implementada en la función `KinematicPositionController::control(...)` dentro de `KinematicPositionController.cpp`:

1. **Lectura de Feedback por TF:**
   El controlador consulta el buffer de TF para obtener la pose actual del robot en el marco del mapa (`map -> base_link`):
   ```cpp
   geometry_msgs::msg::TransformStamped transform = tfBuffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
   current_x = transform.transform.translation.x;
   current_y = transform.transform.translation.y;
   current_a = tf2::getYaw(transform.transform.rotation);
   ```

2. **Cálculo de Error Global:**
   Se calculan los errores cartesianos globales ($dx, dy$) y el error de orientación ($d\theta$) respecto a la pose objetivo actual (`goal_x, goal_y, goal_a`):
   ```cpp
   double dx = goal_x - current_x;
   double dy = goal_y - current_y;
   double dtheta = angles::normalize_angle(goal_a - current_a);
   ```

3. **Rotación al Marco Local (Mapa $\rightarrow$ Robot):**
   Dado que el robot es omnidireccional, puede trasladarse en cualquier dirección local ($V_x$ adelante/atrás, $V_y$ lateral izquierda/derecha). Sin embargo, los errores $dx$ y $dy$ están expresados en el marco global `map`. Por lo tanto, se proyectan al marco local del robot `base_link` utilizando su orientación actual (`current_a`):
   $$\begin{bmatrix} e_x \\ e_y \end{bmatrix} = \begin{bmatrix} \cos(\theta) & \sin(\theta) \\ -\sin(\theta) & \cos(\theta) \end{bmatrix} \begin{bmatrix} dx \\ dy \end{bmatrix}$$
   ```cpp
   double ex = dx * cos(current_a) + dy * sin(current_a);
   double ey = -dx * sin(current_a) + dy * cos(current_a);
   ```

4. **Ley de Control Proporcional (P):**
   Las velocidades de comando en el marco del robot se calculan de manera proporcional a los errores locales mapeados:
   ```cpp
   vx = K_PX * ex;
   vy = K_PY * ey;
   wz = K_PTHETA * dtheta;
   ```
   *Nota: Las ganancias por defecto están configuradas en `K_PX = 1.0`, `K_PY = 1.0` y `K_PTHETA = 1.0`.*

---

### B. Selección de Consigna por Persecución (Pure Pursuit)
Para seguir la trayectoria de manera continua, `KinematicPositionController` implementa un algoritmo de avance de waypoints en la función `getPursuitBasedGoal(...)`:

- El controlador recorre los puntos de la trayectoria de referencia a partir del último índice visitado (`last_idx`).
- **Métrica de Distancia de Lookahead:** Mientras la distancia euclidiana entre la pose actual del robot y el waypoint sea menor que un umbral de anticipación (`LOOKAHEAD = 0.5` metros), el índice del waypoint objetivo avanza:
  ```cpp
  while (last_idx < (int)trajectory.points.size() - 1) {
    double wpoint_x = trajectory.points[last_idx].transform.translation.x;
    double wpoint_y = trajectory.points[last_idx].transform.translation.y;
    double d = dist2(current_x, current_y, wpoint_x, wpoint_y);
    if (d < LOOKAHEAD) last_idx++;
    else break;
  }
  ```
- **Criterio de Finalización:** Al aproximarse al último punto de la trayectoria, si la distancia al objetivo final es menor que `TOLERANCE = 0.05` metros, la función retorna `false` indicando a la clase base que detenga el robot:
  ```cpp
  if (last_idx >= (int)trajectory.points.size() - 1) {
    double d = dist2(current_x, current_y, x, y);
    if (d < TOLERANCE) return false;
  }
  ```

---

### C. Generación de Trayectoria Cuadrada y Orientación "Opuesta al Centro"
La trayectoria cuadrada se construye en la función `build_square_trajectory(...)` dentro de `trajectory_generator_node.cpp`:

1. **Forma del Cuadrado:**
   Se define una secuencia de esquinas que describen un cuadrado con vértices en las coordenadas $(\pm 2, \pm 2)$, lo que genera un trayecto de $4\text{m}$ de lado (coincidiendo con las etiquetas de coordenadas del mapa de la página 6 del PDF):
   ```cpp
   std::vector<std::tuple<double, double, double>> corners = {
       {2, 2, M_PI / 2}, 
       {2, -2, 0.0}, 
       {-2, -2, -M_PI / 2.0}, 
       {-2, 2, -M_PI}, 
       {2, 2, -3.0 * M_PI / 2.0}
   };
   ```

2. **Interpolación Lineal y de Ángulo:**
   A lo largo de cada lado del cuadrado, la posición $(x, y)$ y el ángulo $\theta$ se interpolan linealmente en función del parámetro `stepping` (por defecto $0.05\text{m}$) a una velocidad de crucero constante de `desired_speed = 0.2` m/s:
   ```cpp
   double theta = start_theta + ratio * (end_theta - start_theta);
   ```

3. **Análisis de la Orientación Radial Outward:**
   Matemáticamente, apuntar directamente hacia afuera del origen $(0,0)$ en cualquier punto $(x,y)$ requiere un ángulo $\gamma = \text{atan2}(y, x)$.
   Analizando la interpolación lineal de los ángulos de las esquinas definidos en la lista:
   - **Lado 1 ($x=2, y: 2 \rightarrow -2$):** $\theta$ va de $\frac{\pi}{2}$ a $0$. En el punto medio $(2,0)$, $\theta = \frac{\pi}{4}$. El ángulo radial de salida es $0$. La diferencia es constantemente de $+\frac{\pi}{4}$.
   - **Lado 2 ($y=-2, x: 2 \rightarrow -2$):** $\theta$ va de $0$ a $-\frac{\pi}{2}$. En el punto medio $(0,-2)$, $\theta = -\frac{\pi}{4}$. El ángulo radial de salida es $-\frac{\pi}{2}$. Diferencia: $+\frac{\pi}{4}$.
   - **Lado 3 ($x=-2, y: -2 \rightarrow 2$):** $\theta$ va de $-\frac{\pi}{2}$ a $-\pi$. En el punto medio $(-2,0)$, $\theta = -\frac{3\pi}{4}$. El ángulo radial de salida es $\pi$. Diferencia: $+\frac{\pi}{4}$ (módulo $2\pi$).

   Es decir, el ángulo interpolado cumple con la relación:
   $$\theta_{\text{interpolado}} = \text{atan2}(y, x) + \frac{\pi}{4}$$
   Este desfase de $+45^\circ$ respecto a la dirección radial orienta las coordenadas del robot ($X_R$) de tal forma que se mantienen apuntando radialmente hacia afuera en todo momento, satisfaciendo exactamente el patrón visual del gráfico de la página 6.

---

### D. Registro de Datos y Validación Experimental (Logger Node)
Para facilitar el análisis y responder a los gráficos solicitados en el informe final:
- El nodo `logger_node.cpp` se suscribe a `/robot/odometry`, `/robot/ground_truth`, y `/goal_pose`.
- Almacena en tiempo real los datos en archivos de texto plano con formato `<segundos> <x> <y> <yaw>`:
  - `<timestamp>_poses.log`
  - `<timestamp>_ground-truth.log`
  - `<timestamp>_goals.log`
- Estos registros permiten graficar y contrastar el desempeño de la odometría estimada contra el valor de verdad del simulador (`ground_truth`) y las poses objetivo.
