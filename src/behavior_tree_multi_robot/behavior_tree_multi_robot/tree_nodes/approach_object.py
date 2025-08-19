import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import py_trees
import numpy as np
from rclpy.qos import qos_profile_sensor_data
import math
import time

class ApproachObject(py_trees.behaviour.Behaviour):
    def __init__(self, name="ApproachObject", node=None, 
                 target_distance=0.08, max_linear_speed=0.15, 
                 max_angular_speed=0.5, k_linear=0.8, k_angular=1.2,
                 angle_tolerance=0.05, timeout=15.0):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client()
        self.blackboard.register_key(key="order_info", access=py_trees.common.Access.READ)
        
        self.node = node
        self.target_distance = target_distance  # Distancia objetivo al objeto (metros)
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.k_linear = k_linear
        self.k_angular = k_angular
        self.angle_tolerance = angle_tolerance
        self.timeout = timeout
        
        # Variables de estado
        self.start_time = None
        self.range_reading = None
        self.current_pose = np.zeros(3)  # x, y, theta
        self.initial_yaw = None
        self.object_detected = False
        self.approach_complete = False

    def initialise(self):
        self.robot_ns, self.cubo_ns = self.blackboard.order_info
        self.node.get_logger().info(f"🚀 Iniciando aproximación a objeto para {self.robot_ns}")
        
        # Suscriptores
        self.range_sub = self.node.create_subscription(
            Range,
            f'/{self.robot_ns}/range/unfiltered',
            self.range_callback,
            qos_profile_sensor_data
        )
        
        self.odom_sub = self.node.create_subscription(
            Odometry,
            f'/{self.robot_ns}/odometry/filtered',
            self.odom_callback,
            10
        )
        
        # Publicador
        self.cmd_vel_pub = self.node.create_publisher(
            Twist,
            f'/{self.robot_ns}/cmd_vel',
            10
        )
        
        # Resetear estado
        self.start_time = self.node.get_clock().now().seconds_nanoseconds()[0]
        self.range_reading = None
        self.initial_yaw = None
        self.object_detected = False
        self.approach_complete = False
        
    def range_callback(self, msg):
        self.range_reading = msg.range
        # Detectar si hay un objeto dentro del rango máximo del sensor
        if msg.range < msg.max_range and msg.range > msg.min_range:
            self.object_detected = True

    def odom_callback(self, msg):
        quat = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.current_pose[2] = yaw
        
        # Guardar orientación inicial al primer mensaje
        if self.initial_yaw is None:
            self.initial_yaw = yaw

    def update(self):
        current_time = self.node.get_clock().now().seconds_nanoseconds()[0]
        
        # Verificar timeout
        if current_time - self.start_time > self.timeout:
            self.node.get_logger().warn("⏰ Tiempo de aproximación agotado")
            return py_trees.common.Status.FAILURE
        
        # Verificar si tenemos datos del sensor
        if self.range_reading is None:
            return py_trees.common.Status.RUNNING
            
        # Verificar si se perdió el objeto
        if not self.object_detected:
            self.node.get_logger().warn("❓ Objeto perdido durante la aproximación")
            return py_trees.common.Status.FAILURE
        
        # Crear mensaje de velocidad
        cmd_vel = Twist()
        
        # Control de velocidad lineal (aproximación directa)
        distance_error = self.range_reading - self.target_distance
        
        # Solo avanzar si estamos a más de 5cm del objetivo
        if distance_error > 0.05:
            linear_speed = min(
                self.k_linear * distance_error,
                self.max_linear_speed
            )
            # Reducir velocidad cerca del objetivo
            if distance_error < 0.15:
                linear_speed *= 0.6
            cmd_vel.linear.x = linear_speed
            
            # Control de velocidad angular (mantener dirección inicial)
            if self.initial_yaw is not None:
                angle_error = self.normalize_angle(self.initial_yaw - self.current_pose[2])
                if abs(angle_error) > self.angle_tolerance:
                    angular_speed = np.clip(
                        self.k_angular * angle_error,
                        -self.max_angular_speed,
                        self.max_angular_speed
                    )
                    cmd_vel.angular.z = angular_speed
                    # Reducir velocidad lineal mientras se corrige dirección
                    cmd_vel.linear.x *= 0.7
        
        # Verificar si se alcanzó la distancia objetivo
        elif abs(distance_error) <= 0.05:
            self.node.get_logger().info("🎯 Objetivo alcanzado!")
            self.approach_complete = True
            return py_trees.common.Status.SUCCESS
        
        # Publicar comando de velocidad
        self.cmd_vel_pub.publish(cmd_vel)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        # Detener el robot al terminar
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)
        self.node.get_logger().info("🛑 Comportamiento de aproximación finalizado")
        
        # Limpiar suscripciones
        if hasattr(self, 'range_sub'):
            self.node.destroy_subscription(self.range_sub)
        if hasattr(self, 'odom_sub'):
            self.node.destroy_subscription(self.odom_sub)
        
        time.sleep(3)

    @staticmethod
    def normalize_angle(angle):
        """Normaliza ángulos al rango [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle