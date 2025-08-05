import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import numpy as np
import py_trees
import time


class GoToPosition(py_trees.behaviour.Behaviour):
    def __init__(self, name="GoToPosition", node=None, position='same', goal_tolerance=None):
        super().__init__(name)
        # Declarar puertos de entrada
        self.blackboard = py_trees.blackboard.Client()
        self.blackboard.register_key(key="order_info",access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="other_robot", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="goal_coords",access=py_trees.common.Access.READ        )

        self.node = node
        self.namespace = None
        self.goal = self.blackboard.goal_coords # [x, y]
        self.current_pose = np.zeros(3)  # x, y, theta
        self.current_twist = np.zeros(2)  # Vx, Wz
        self.reached_goal = False

        # Parámetros de control
        self.k_linear = 0.3
        self.k_angular = 1.0
        self.max_linear_speed = 0.35
        self.max_angular_speed = 1.0
        self.goal_tolerance = goal_tolerance
        self.angle_tolerance = 0.08
        self.slowdown_radius = goal_tolerance + 0.1

        # Variables de estado
        self.prev_angular_z = 0.0
        self.ready_to_move = False
        self.moving_toward_goal = False

        self.done_init = False
        self.done_odom = False

        self.position = position
                

    def initialise(self):
        """Solo se ejecuta cuando el nodo se activa por primera vez"""
        if(self.position == 'other'):
            self.namespace = self.blackboard.other_robot
        else:    
            self.namespace,_ = self.blackboard.order_info
        self.goal = self.blackboard.goal_coords # [x, y]
        self.prev_angular_z = 0.0
        self.reached_goal = False
        self.ready_to_move = False
        self.moving_toward_goal = False

        #self.node.get_logger().info(f"Robot seleccionado: {self.namespace}")
        #self.node.get_logger().info(f"Cordenadas: {self.goal}")

        # Suscripción a odometría
        self.odom_sub = self.node.create_subscription(
            Odometry,
            f'/{self.namespace}/odometry/filtered',
            self.odom_callback,
            10
        )

        # Publicador de velocidad
        self.cmd_vel_pub = self.node.create_publisher(
            Twist,
            f'/{self.namespace}/cmd_vel',
            10
        )

        # Temporizador para control a 20Hz
        self.control_timer = self.node.create_timer(0.05, self.control_callback)

        self.done_init = True


        
    
    def odom_callback(self, msg):
        """Actualiza posición y orientación actual del robot."""
        if self.done_init:
            #self.node.get_logger().info(f"Odometría recibida: x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}")
            self.current_pose[0] = msg.pose.pose.position.x
            self.current_pose[1] = msg.pose.pose.position.y

            self.current_twist[0] = msg.twist.twist.linear.x
            self.current_twist[1] = msg.twist.twist.angular.z

            quat = msg.pose.pose.orientation
            _, _, self.current_pose[2] = euler_from_quaternion(
                [quat.x, quat.y, quat.z, quat.w]
            )
            self.done_odom = True

    def control_callback(self):
        """Callback de control a 20Hz"""
        if self.done_init and self.done_odom:
            if self.goal is None or self.reached_goal:
                return
                
            cmd_vel = Twist()
            
            # Cálculo de errores
            dx = self.goal[0] - self.current_pose[0]
            dy = self.goal[1] - self.current_pose[1]
            distance = np.hypot(dx, dy)
            target_angle = np.arctan2(dy, dx)
            angle_error = self.normalize_angle(target_angle - self.current_pose[2])
            
            if distance < self.goal_tolerance:
                # Objetivo alcanzado
                cmd_vel = Twist()
                self.reached_goal = True
                self.node.get_logger().info("Objetivo alcanzado")
            elif not self.moving_toward_goal:
                # Fase de alineación angular
                if abs(angle_error) > self.angle_tolerance:
                    cmd_vel.angular.z = np.clip(
                        self.k_angular * angle_error,
                        -self.max_angular_speed,
                        self.max_angular_speed
                    )
                else:
                    self.moving_toward_goal = True
            else:
                # Fase de movimiento hacia el objetivo
                linear_speed = min(
                    self.k_linear * distance,
                    self.max_linear_speed * min(1.0, distance/self.slowdown_radius)
                )
                angular_speed = np.clip(
                    self.k_angular * angle_error,
                    -self.max_angular_speed,
                    self.max_angular_speed
                )
                
                cmd_vel.linear.x = linear_speed
                cmd_vel.angular.z = 0.8 * angular_speed + 0.2 * self.prev_angular_z
                self.prev_angular_z = cmd_vel.angular.z
            
            self.cmd_vel_pub.publish(cmd_vel)

    def update(self):
        """Método principal del Behavior Tree"""
        if self.done_init:
            if self.goal is None:
                self.node.get_logger().warn("No se ha definido una meta")
                return py_trees.common.Status.FAILURE
                
            if self.reached_goal:
                return py_trees.common.Status.SUCCESS
                
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """Limpieza al terminar el comportamiento"""
        if self.done_init:
            self.node.get_logger().info("Ejecutando parada de seguridad")

            # Intenta detener el robot hasta 10 veces o hasta que esté quieto
            attempts = 0

            while attempts < 10:
                self.cmd_vel_pub.publish(Twist())  # Parada de seguridad
                attempts += 1
                time.sleep(0.1)  # 10 Hz

            
            # Destruye la suscripción
            if hasattr(self, 'odom_sub') and self.odom_sub:
                self.node.destroy_subscription(self.odom_sub)
                self.odom_sub = None  # Marca como destruido

            # Destruye el publicador
            if hasattr(self, 'cmd_vel_pub') and self.cmd_vel_pub:
                self.node.destroy_publisher(self.cmd_vel_pub)
                self.cmd_vel_pub = None  # Marca como destruido

            self.control_timer.cancel()
            self.done_init = False
            self.done_odom = False


    @staticmethod
    def normalize_angle(angle):
        """Normaliza ángulos al rango [-π, π]"""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle