# detect_object.py
import py_trees
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
import math
import numpy as np
import time

class DetectObject(py_trees.behaviour.Behaviour):
    def __init__(self, name="DetectObject", node=None):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client()
        self.blackboard.register_key(key="order_info", access=py_trees.common.Access.READ)

        self.node = node

        # Configurables
        self.step_deg = 20              # Ángulo a girar a cada lado (+/- N grados)
        self.wait_sec = 1               # Tiempo de espera entre movimientos
        self.distance_threshold = 0.15   # Umbral de detección (m)
        self.extra_deg_after_detection = 0  # Grados extra tras detección
        self.kp = 1.8                    # Ganancia del controlador P
        self.max_cycles = 2              # Número máximo de ciclos +N° → -N°

        # Estado
        self.robot_ns = None
        self.cubo_ns = None
        self.latest_range = None
        self.min_range = None
        self.detected = False
        self.state = "IDLE"
        self.cycle_count = 0
        self.direction = -1               # 1 = +N°, -1 = -N°
        self.start_yaw = None
        self.target_yaw = None
        self.current_yaw = None
        self.stable_start_time = None

    def initialise(self):
        self.robot_ns, self.cubo_ns = self.blackboard.order_info
        self.node.get_logger().info(f"🔄 Iniciando detección de objeto para {self.robot_ns}")

        self.node.create_subscription(
            Range,
            f'/{self.robot_ns}/range/unfiltered',
            self.range_callback,
            qos_profile_sensor_data
        )

        self.node.create_subscription(
            Odometry,
            f'/{self.robot_ns}/odometry/filtered',
            self.odom_callback,
            10
        )

        self.cmd_vel_pub = self.node.create_publisher(
            Twist,
            f'/{self.robot_ns}/cmd_vel',
            10
        )

        # Reset de estados
        self.detected = False
        self.cycle_count = 0
        self.direction = 1
        self.state = "ROTATING"
        self.start_yaw = None
        self.target_yaw = None
        self.current_yaw = None
        self.stable_start_time = None

    def range_callback(self, msg):
        self.latest_range = msg.range
        self.min_range = msg.min_range
        # El ultrasonido funciona en paralelo
        if self.min_range <= msg.range < self.distance_threshold:
            self.detected = True

    def odom_callback(self, msg):
        quat = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.current_yaw = yaw

    def update(self):
        # Si detecta en cualquier momento
        if self.detected:
            if abs(self.extra_deg_after_detection) > 0.001:
                self.state = "EXTRA_ROTATING"
                self.target_yaw = self.normalize_angle(
                    self.current_yaw + math.radians(self.extra_deg_after_detection)
                )
                self.node.get_logger().info("🟡 Objeto detectado, girando ángulo extra...")
                return py_trees.common.Status.RUNNING
            else:
                self.cmd_vel_pub.publish(Twist())
                self.node.get_logger().info("✅ Objeto detectado — quedándose en orientación actual.")
                return py_trees.common.Status.SUCCESS

        # Movimiento normal
        if self.state == "ROTATING":
            if self.start_yaw is None and self.current_yaw is not None:
                self.start_yaw = self.current_yaw
                self.target_yaw = self.normalize_angle(
                    self.start_yaw + math.radians(self.direction * self.step_deg)
                )
                self.node.get_logger().info(
                    f"🔁 Giro objetivo: {math.degrees(self.target_yaw):.2f}°"
                )

            if self.current_yaw is not None and self.target_yaw is not None:
                error = self.normalize_angle(self.target_yaw - self.current_yaw)
                if abs(error) < math.radians(2):
                    self.cmd_vel_pub.publish(Twist())
                    self.state = "WAITING"
                    self.stable_start_time = self.node.get_clock().now().nanoseconds / 1e9
                    return py_trees.common.Status.RUNNING

                cmd = Twist()
                cmd.angular.z = np.clip(self.kp * error, -0.6, 0.6)
                self.cmd_vel_pub.publish(cmd)

        elif self.state == "WAITING":
            now = self.node.get_clock().now().nanoseconds / 1e9
            if now - self.stable_start_time >= self.wait_sec:
                if self.direction == -1:
                    self.cycle_count += 1
                    self.node.get_logger().info(f"🔎 Ciclo {self.cycle_count}/{self.max_cycles} completado")
                    if self.cycle_count >= self.max_cycles:
                        self.node.get_logger().info("🛑 Barrido completo sin detección, regresando a orientación inicial.")
                        self.state = "RETURNING_HOME"
                        self.target_yaw = self.start_yaw
                        return py_trees.common.Status.RUNNING
                # Cambiar dirección (+ → - o - → +)
                self.direction *= -1
                self.target_yaw = self.normalize_angle(
                    self.start_yaw + math.radians(self.direction * self.step_deg)
                )
                self.state = "ROTATING"
            else:
                self.cmd_vel_pub.publish(Twist())

        elif self.state == "EXTRA_ROTATING":
            error = self.normalize_angle(self.target_yaw - self.current_yaw)
            if abs(error) < math.radians(1.5):
                self.cmd_vel_pub.publish(Twist())
                self.node.get_logger().info("✅ Objeto detectado y ángulo extra completado.")
                return py_trees.common.Status.SUCCESS
            cmd = Twist()
            cmd.angular.z = np.clip(self.kp * error, -0.5, 0.5)
            self.cmd_vel_pub.publish(cmd)

        elif self.state == "RETURNING_HOME":
            error = self.normalize_angle(self.target_yaw - self.current_yaw)
            if abs(error) < math.radians(1.5):
                self.cmd_vel_pub.publish(Twist())
                self.node.get_logger().info("✅ Regresó a orientación inicial. Éxito sin detección.")
                return py_trees.common.Status.SUCCESS
            cmd = Twist()
            cmd.angular.z = np.clip(self.kp * error, -0.6, 0.6)
            self.cmd_vel_pub.publish(cmd)

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        for _ in range(10):
            self.cmd_vel_pub.publish(Twist())
            time.sleep(0.1)
        self.node.get_logger().info("🛑 Terminando búsqueda de objeto.")

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
