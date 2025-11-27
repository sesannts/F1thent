import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry 
import time
import math

# ==============================================================================
#                      CONSTANTES Y AJUSTES DEL ALGORITMO (TUNING)
# ==============================================================================

MAX_SPEED = 3.5             # AUMENTADO: Nueva velocidad máxima (m/s)
SAFETY_DISTANCE = 1.0       # REDUCIDO: Búfer de seguridad ligeramente menor para usar mejor la pista
MAX_STEERING_ANGLE = 0.35   
STEERING_GAIN = 0.4         # REDUCIDO SIGNIFICATIVAMENTE: Disminuye la sensibilidad del giro para evitar el zig-zag.

# Tópicos (Dejamos el tópico ACTIVO que encontramos)
SCAN_TOPIC = '/scan' 
DRIVE_TOPIC = '/drive' 
POSE_TOPIC = '/ego_racecar/odom' 

# ==============================================================================
#                         CONSTANTES DEL CRONÓMETRO Y VUELTAS
# ==============================================================================
# Usa tus valores ya verificados:
START_FINISH_X = 1.915      
SF_Y_MIN = -2.57            
SF_Y_MAX = -0.74            
LAP_TRIGGER_BUFFER_X = 1.5 
# ==============================================================================


class CarControllerNode(Node):
    def __init__(self):
        super().__init__('car_controller_node')
        
        self.scan_sub = self.create_subscription(
            LaserScan, SCAN_TOPIC, self.scan_callback, 10
        )
        self.pose_sub = self.create_subscription(
            Odometry, POSE_TOPIC, self.pose_callback, 10
        )
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, DRIVE_TOPIC, 10
        )
        
        # Variables de Lap Counter
        self.lap_count = 0
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.lap_times = []
        self.passed_start_line = False
        
        self.get_logger().info("Car Controller Node inicializado. Esperando datos del Lidar y Pose...")

    # ==========================================================================
    # LÓGICA DE DETECCIÓN DE VUELTA Y TIEMPO (Sin cambios)
    # ==========================================================================
    
    def pose_callback(self, pose_msg):
        """
        Callback para recibir la posición del coche y manejar el contador de vueltas.
        """
        current_x = pose_msg.pose.pose.position.x
        current_y = pose_msg.pose.pose.position.y
        
        # 1. Chequear si la posición X está CERCA de la línea de meta
        is_at_x = abs(current_x - START_FINISH_X) < LAP_TRIGGER_BUFFER_X
        
        # 2. Chequear si la posición Y está en el segmento de meta
        is_in_y_range = SF_Y_MIN <= current_y <= SF_Y_MAX
        
        if is_at_x and is_in_y_range:
            if not self.passed_start_line:
                current_time = self.get_clock().now().nanoseconds / 1e9
                
                if self.lap_count == 0:
                    self.lap_count += 1
                    self.get_logger().info(f"🏁 Vuelta {self.lap_count} iniciada. Tiempo de inicio: {current_time:.2f} s")
                
                else:
                    lap_duration = current_time - self.start_time
                    self.lap_times.append(lap_duration)
                    
                    self.lap_count += 1
                    self.get_logger().info("--------------------------------------------------")
                    self.get_logger().info(f"✅ ¡Vuelta {self.lap_count-1} COMPLETA!")
                    self.get_logger().info(f"⏱️ Tiempo de Vuelta: {lap_duration:.2f} segundos")
                    self.get_logger().info(f"🏁 Total Vueltas: {self.lap_count-1} / 10")
                    self.get_logger().info("--------------------------------------------------")
                    
                    self.start_time = current_time

                self.passed_start_line = True
        else:
            self.passed_start_line = False


    # ==========================================================================
    # LÓGICA DE FOLLOW THE GAP (FTG) - CON ESTABILIZACIÓN
    # ==========================================================================
    
    def scan_callback(self, scan_msg):
        ranges = list(scan_msg.ranges)
        
        # --- PASO 1: Eliminar Obstáculos (Safety Bubble) ---
        processed_ranges = []
        for r in ranges:
            if r < SAFETY_DISTANCE or math.isinf(r) or r == 0.0:
                processed_ranges.append(0.0)
            else:
                processed_ranges.append(r)
        
        # --- PASO 2: Encontrar el Gap Más Grande ---
        max_gap_size = 0
        max_gap_start_index = -1
        current_gap_start = -1
        
        for i, dist in enumerate(processed_ranges):
            if dist > 0.0:
                if current_gap_start == -1:
                    current_gap_start = i
                
                current_gap_size = i - current_gap_start + 1
                
                if current_gap_size > max_gap_size:
                    max_gap_size = current_gap_size
                    max_gap_start_index = current_gap_start
            else:
                current_gap_start = -1
                
        if max_gap_size == 0:
            self.publish_drive_command(0.0, 0.0)
            self.get_logger().warn("Pista bloqueada. Deteniendo el coche.")
            return

        max_gap_end_index = max_gap_start_index + max_gap_size - 1
        center_index = (max_gap_start_index + max_gap_end_index) // 2

        # --- PASO 3: Calcular Ángulo de Conducción ---
        
        angle_range = center_index * scan_msg.angle_increment + scan_msg.angle_min
        
        # Aplicar el gain (Steering Gain < 1.0 reduce el zig-zag)
        steering_angle = angle_range * STEERING_GAIN 
        steering_angle = max(min(steering_angle, MAX_STEERING_ANGLE), -MAX_STEERING_ANGLE)


        # --- PASO 4: Determinar Velocidad (Tuning de Curva/Recta) ---
        
        normalized_angle = abs(steering_angle) / MAX_STEERING_ANGLE
        
        # Usamos una penalización cuadrática (más lento en curvas cerradas)
        speed = MAX_SPEED * (1.0 - normalized_angle**2) 
        
        # GARANTIZAR VELOCIDAD MÍNIMA
        speed = max(min(speed, MAX_SPEED), 1.5) # Mínimo 1.5 m/s

        self.publish_drive_command(steering_angle, speed)


    def publish_drive_command(self, angle, speed):
        """Crea y publica un mensaje AckermannDriveStamped."""
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    car_controller = CarControllerNode()
    rclpy.spin(car_controller)
    car_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()