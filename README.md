# F1thent
Repositorio de un proyecto en F1thent con el uso de Ros 2 Humble.
# 🏎️ Controlador Autónomo F1Tenth: Follow the Gap (FTG)

Este proyecto implementa un controlador reactivo para un vehículo F1Tenth utilizando **ROS 2 Humble**. El objetivo principal es que el vehículo navegue una pista asignada de forma autónoma, evite colisiones y registre el tiempo por vuelta para optimizar el rendimiento.

## 🧠 1. Descripción del Enfoque: Follow the Gap (FTG)

El controlador utiliza el algoritmo **Follow the Gap (FTG)**, un método reactivo de planificación de trayectoria local que se basa en los datos del sensor Lidar.

El algoritmo busca el **espacio libre más grande** (`Gap`) por delante en la matriz de distancias del Lidar y dirige el vehículo hacia el **centro** de ese espacio, priorizando la evasión de obstáculos.

### Lógica de Control y Tuning

La implementación incluye ajustes de rendimiento (tuning) para estabilizar el vehículo y reducir el zig-zag en las rectas, crucial para la competencia de menor tiempo.

| Constante | Valor | Función |
| :--- | :--- | :--- |
| `MAX_SPEED` | `4.5` | Velocidad máxima en rectas (m/s). |
| `SAFETY_DISTANCE` | `1.0` | Distancia mínima para detectar obstáculos (búfer de seguridad). |
| `STEERING_GAIN` | `0.7` | **Factor de Suavizado (Clave)**: Reduce la sensibilidad del giro para minimizar el comportamiento de zig-zag. |
| `POSE_TOPIC` | `/ego_racecar/odom` | Tópico de odometría (pose del vehículo) que demostró estar activo en la simulación. |

## 💻 2. Estructura del Código (`car_controller_node.py`)

Todo el código de control, la lógica de FTG y el sistema de cronometraje se encuentran encapsulados en un único nodo de ROS 2 llamado **`CarControllerNode`**.

### a) Subsistema ROS 2 y Tópicos

El nodo utiliza los siguientes tópicos y tipos de mensajes:

| Tópico | Tipo de Mensaje | Uso |
| :--- | :--- | :--- |
| `/scan` | `sensor_msgs/msg/LaserScan` | **Suscripción:** Datos del sensor Lidar para la navegación. |
| `/ego_racecar/odom` | `nav_msgs/msg/Odometry` | **Suscripción:** Posición $(X, Y)$ del vehículo para el contador/cronómetro. |
| `/drive` | `ackermann_msgs/msg/AckermannDriveStamped` | **Publicación:** Comandos de velocidad y ángulo de dirección. |

### b) Requisitos de la Competencia (Contador y Cronómetro)

La función `pose_callback` implementa el sistema de cronometraje y conteo de vueltas, cumpliendo con los requisitos de la tarea:

* **Línea de Meta:** Definida por un rango de coordenadas $(X, Y)$ específicas de la pista de Budapest.
* **Detección:** Registra el cruce de la línea usando los datos de odometría (`/ego_racecar/odom`).
* **Salida:** Muestra el tiempo de cada vuelta y el total de vueltas completadas (`Total Vueltas: N / 10`) directamente en la terminal de ejecución.

**Coordenadas de la Línea de Meta (Pista Budapest):**

| Constante | Valor | Significado |
| :--- | :--- | :--- |
| `START_FINISH_X` | `1.915` | Coordenada X central de la línea de meta. |
| `SF_Y_MIN` | `-2.57` | Coordenada Y mínima del segmento de meta. |
| `SF_Y_MAX` | `-0.74` | Coordenada Y máxima del segmento de meta. |

## 🚀 3. Instrucciones de Ejecución

Estas instrucciones asumen que el paquete `f1tenth_ftg` ha sido creado y que el entorno **ROS 2 Humble** está instalado.

### Paso 1: Compilación del Paquete

Navegue a la raíz de su *workspace* (`~/F1Tenth-Repository`) y compile:

```bash
cd ~/F1Tenth-Repository
colcon build
```

### **Paso 2: Lanzamiento del Controlador (Dos Terminales)**

Se requieren dos terminales abiertas.  
En ambas terminales, el primer paso es cargar el entorno de ROS 2 para que los comandos sean reconocidos:

```bash
source install/setup.bash
```
#### Terminal 1: Iniciar el Simulador (Pista)

Ejecute el puente de simulación (asegúrese de que el mapa correcto esté cargado):

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
#### Terminal 2: Iniciar el Nodo de Control

Ejecute su nodo autónomo. Esta terminal mostrará el progreso de la carrera y los tiempos.

```bash
ros2 run f1tenth_ftg car_controller
```
### Paso 3: Monitoreo y Resultados

La **Terminal 2** es donde se mostrará el progreso de su carrera y la evidencia requerida para la tarea (contador y cronómetro).

Una vez que el vehículo cruce la línea de meta, el sistema comenzará a registrar y mostrar el tiempo por vuelta:
```
[INFO] [...] [car_controller_node]: ✅ ¡Vuelta 1 COMPLETA!
[INFO] [...] [car_controller_node]: ⏱️ Tiempo de Vuelta: XX.XX segundos
[INFO] [...] [car_controller_node]: 🏁 Total Vueltas: 1 / 10
```
Listo, siguiendo estos pasos tendrias funcionando tu carro f1tenth con manejo autonomo.
